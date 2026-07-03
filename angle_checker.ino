#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#define HEIGHT 320
#define WIDTH 170
#define HEIGHT_DIV (HEIGHT / 2)
#define WIDTH_DIV (WIDTH / 2)
#include <Adafruit_BNO08x.h>

#include <Wire.h>
#include <string>
#define BNO08X_RESET  -1   // no external reset pin
#define BNO08X_INT    -1   // no interrupt pin

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;  // Correction : On utilise une instance de lumière dédiée

  public:
    LGFX(void) {
      {
        auto cfg = _bus_instance.config();
        cfg.spi_host = VSPI_HOST;
        cfg.freq_write = 80000000;
        cfg.freq_read = 16000000;
        cfg.pin_mosi = 23;
        cfg.pin_miso = -1;
        cfg.pin_sclk = 18;
        cfg.pin_dc = 2;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }
      {
        auto cfg = _panel_instance.config();
        cfg.pin_cs = 15;
        cfg.pin_rst = 4;
        cfg.pin_busy = -1;

        cfg.panel_width = WIDTH;
        cfg.panel_height = HEIGHT;
        cfg.offset_x = 35;
        cfg.offset_y = 0;

        _panel_instance.config(cfg);
      }
      {
        auto cfg = _light_instance.config();
        cfg.pin_bl = 32;
        cfg.invert = false;
        _light_instance.config(cfg);
        _panel_instance.setLight(&_light_instance);
      }
      setPanel(&_panel_instance);
    }
};

LGFX lcd;
LGFX_Sprite cubeSprite(&lcd);
LGFX_Sprite leanMeter(&lcd);
Adafruit_BNO08x bno08x = Adafruit_BNO08x();
sh2_SensorValue_t sensorValue;

void drawCube(LGFX_Sprite &canvas,
  float qw, float qx, float qy, float qz,
  int16_t cx, int16_t cy, float scale);

void setReport()
{
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 2500);
}

void  setCursor_Text_Delay(LGFX &lcd, unsigned int x, unsigned int y, const char *str, float dl)
{
  lcd.setCursor(x, y);
  lcd.print(str);
  delay(dl);
}

void setup()
{
  lcd.init();
  lcd.setRotation(1);
  lcd.invertDisplay(true);
  setCursor_Text_Delay(lcd, 0, 0, "[START]\t| LCD: Initialisation Started.", 50);
  setCursor_Text_Delay(lcd, 0, 10, "[OK] \t| LCD: Initialisation Success.", 50);

  setCursor_Text_Delay(lcd, 0, 20, "[START]\t| Wire: Initialisation Started.", 50);
  if (!Wire.begin())
  {
    setCursor_Text_Delay(lcd, 0, 30, "[NOK]\t| Wire: Initialisation Failed.", 50);
    return ;
  }
  setCursor_Text_Delay(lcd, 0, 30, "[OK] \t| Wire: Initialisation Success.", 50);

  setCursor_Text_Delay(lcd, 0, 40, "[START]\t| Serial: Initialisation Started.", 50);
  Serial.begin(115200);
  setCursor_Text_Delay(lcd, 0, 50, "[OK]\t| Serial: Initialisation Success.", 50);
  setCursor_Text_Delay(lcd, 0, 60, "[START]\t| Sprite Cube: Initialisation Started.", 50);
  if (!cubeSprite.createSprite(HEIGHT_DIV, WIDTH))
  {
    setCursor_Text_Delay(lcd, 0, 70, "[NOK]\t| Sprite Cube: Initialisation Failed.", 50);
    Serial.println(F("Failed to allocate sprite memory"));
    return ;
  }
  setCursor_Text_Delay(lcd, 0, 70, "[OK]\t| Sprite Cube: Initialisation Success.", 50);
  setCursor_Text_Delay(lcd, 0, 80, "[START]\t| Sprite leanmeter: Initialisation Started.", 50);
  if (!leanMeter.createSprite(HEIGHT_DIV, WIDTH))
  {
    setCursor_Text_Delay(lcd, 0, 90, "[NOK]\t| Sprite leanmeter: Initialisation Failed.", 50);
    Serial.println(F("Failed to allocate sprite leanmeter memory"));
    return ;
  }
  setCursor_Text_Delay(lcd, 0, 90, "[OK]\t| Sprite leanmeter: Initialisation Success.", 50);
  setCursor_Text_Delay(lcd, 0, 100, "[START]\t| Bno08x: Initialisation Started.", 50);
  while (!bno08x.begin_I2C(0x4B))
  {
    setCursor_Text_Delay(lcd, 0, 110, "[NOK]\t| Bno08x: Initialisation Failed.", 50);
    setCursor_Text_Delay(lcd, 0, 120, "Please Verify connection, alimentation. or add resistors", 50);
    Serial.println("Erreur : Impossible de trouver le composant BNO08x !");
    Serial.println("Vérifiez le câblage, l'alimentation, ou essayez d'ajouter une résistance de Pull-Up.");
    delay(1000);
  }
  setCursor_Text_Delay(lcd, 0, 120, "                                                                                ", 0);
  setCursor_Text_Delay(lcd, 0, 110, "[OK]\t| Bno08x: Initialisation Success.", 100);
  lcd.fillScreen(TFT_WHITE);
  cubeSprite.setTextSize(2);
  cubeSprite.setTextColor(TFT_WHITE);
  leanMeter.setTextSize(2);
  leanMeter.setTextColor(TFT_WHITE);
  setReport();
}

float quaternionToRoll(float w, float x, float y, float z)
{
  float sqr = sq(w), sqi = sq(x), sqj = sq(y), sqk = sq(z);
  return atan2(2.0f*(y*z + x*w), (-sqi - sqj + sqk + sqr));
}

void loop() {
  if (!bno08x.getSensorEvent(&sensorValue))
  {
    Serial.println("No Sensor Event");
    setReport();
    delay(100);
  }
  if (bno08x.wasReset())
  {
    Serial.println("bno085 got reset");
    setReport();
    delay(100);
  }
  switch (sensorValue.sensorId)
  {
    case SH2_GAME_ROTATION_VECTOR:
      float qw = sensorValue.un.gameRotationVector.real;
      float roll = sensorValue.un.gameRotationVector.i;
      float pitch = sensorValue.un.gameRotationVector.j;
      float yaw = sensorValue.un.gameRotationVector.k;
      float mag = sqrt(qw*qw + roll*roll + pitch*pitch + yaw*yaw);
      if (mag > 0)
        qw /= mag; roll /= mag; pitch /= mag; yaw /= mag;
      cubeSprite.fillScreen(TFT_BLACK);
      cubeSprite.setCursor(0, 0);
      cubeSprite.print("cube");
      drawCube(cubeSprite, qw, pitch, yaw, -roll, HEIGHT_DIV >> 1, WIDTH_DIV, 2.2f);
      cubeSprite.pushSprite(0, 0);
      leanMeter.fillScreen(TFT_BLACK);
      leanMeter.setCursor(0, 0);
      leanMeter.setTextSize(2);
      leanMeter.print("lean");
      drawGauge(leanMeter, quaternionToRoll(qw, roll, pitch, yaw), HEIGHT_DIV >> 1, WIDTH_DIV + (54 >> 1));
      leanMeter.pushSprite(HEIGHT_DIV, 0);
      break;
  }
  delay(2.5);
}
