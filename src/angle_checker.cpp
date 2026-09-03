#define HEIGHT 320
#define WIDTH 170
#define HEIGHT_DIV (HEIGHT / 2)
#define WIDTH_DIV (WIDTH / 2)

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <string>
#include <TinyGPS++.h>
#include <ESP32Time.h>

#define SPI_SPEED SD_SCK_MHZ(4)

#define LGFX_USE_V1
#define SD_CS_PIN 5
#define SD_MISO_PIN 12
#define SD_MOSI_PIN 13
#define SD_CLK_PIN 14

#define BNO08X_RESET -1  // no external reset pin
#define BNO08X_INT -1    // no interrupt pin

#define DUMP_SIZE 2048
#define MAX_BUFF_SIZE DUMP_SIZE + 38

#define TARGET_HZ 200
#define DELAY_HZ 1000 / TARGET_HZ
#define QUERY_BNO 1000000 / TARGET_HZ

#define GPS_RX_PIN 17  // Connect to NEO-M10N RX
#define GPS_TX_PIN 16  // Connect to NEO-M10N TX
#define GPS_BAUD_RATE 38400

#define BNO_ROTATION_VECTOR 0x1
#define BNO_ACCELEROMETER 0x2
#define GPS_LONG_LAT 0x4

typedef struct S_buf {
  unsigned char buff[MAX_BUFF_SIZE];
  uint16_t len;
} t_buf;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;  // Correction : On utilise une instance de lumière dédiée

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.pin_mosi = 23;
      cfg.pin_miso = 19;
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

unsigned long       previousMillis = 0;
LGFX                lcd;
LGFX_Sprite         cubeSprite(&lcd);
LGFX_Sprite         leanMeter(&lcd);
Adafruit_BNO08x     bno08x = Adafruit_BNO08x();
t_buf               dump;
FsFile              f;
float               latitude = 0;
float               longitude = 0;
float               altitude = 0;
sh2_SensorValue_t   sensorValue;
unsigned long       lastGpsUpdateMicros = 0;
const unsigned long GPS_UPDATE_INTERVAL_MICROS = 1000000;  // 1 second in microseconds
ESP32Time           rtc(7200);
TinyGPSPlus         gps;
HardwareSerial      gpsSerial(2); 
SPIClass            sdSPI(HSPI);
SdFs                sd;

void drawCube(LGFX_Sprite &canvas,
              float qw, float qx, float qy, float qz,
              int16_t cx, int16_t cy, float scale);

void setReport() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, QUERY_BNO);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, QUERY_BNO);
}

void setCursor_Text_Delay(LGFX &lcd, unsigned int x, unsigned int y, const char *str, float dl) {
  lcd.setCursor(x, y);
  lcd.print(str);
  delay(dl);
}

float quaternionToRoll(float w, float x, float y, float z) {
  float sqr = sq(w), sqi = sq(x), sqj = sq(y), sqk = sq(z);
  return atan2(2.0f * (y * z + x * w), (-sqi - sqj + sqk + sqr));
}

void setup() {
  lcd.init();
  lcd.setRotation(1);
  lcd.invertDisplay(true);
  setCursor_Text_Delay(lcd, 0, 0, "[START]\t| LCD: Initialisation Started.", 200);
  setCursor_Text_Delay(lcd, 0, 10, "[OK] \t| LCD: Initialisation Success.", 200);

  setCursor_Text_Delay(lcd, 0, 20, "[START]\t| Wire: Initialisation Started.", 200);
  if (!Wire.begin()) {
    setCursor_Text_Delay(lcd, 0, 30, "[NOK]\t| Wire: Initialisation Failed.", 200);
    return;
  }
  setCursor_Text_Delay(lcd, 0, 30, "[OK] \t| Wire: Initialisation Success.", 200);

  setCursor_Text_Delay(lcd, 0, 40, "[START]\t| Serial: Initialisation Started.", 200);
  Serial.begin(115200);
  setCursor_Text_Delay(lcd, 0, 50, "[OK]\t| Serial: Initialisation Success.", 200);
  setCursor_Text_Delay(lcd, 0, 60, "[START]\t| Sprite Cube: Initialisation Started.", 200);
  if (!cubeSprite.createSprite(HEIGHT_DIV, WIDTH)) {
    setCursor_Text_Delay(lcd, 0, 70, "[NOK]\t| Sprite Cube: Initialisation Failed.", 200);
    Serial.println(F("Failed to allocate sprite memory"));
    return;
  }
  setCursor_Text_Delay(lcd, 0, 70, "[OK]\t| Sprite Cube: Initialisation Success.", 200);
  setCursor_Text_Delay(lcd, 0, 80, "[START]\t| Sprite leanmeter: Initialisation Started.", 200);
  if (!leanMeter.createSprite(HEIGHT_DIV, WIDTH)) {
    setCursor_Text_Delay(lcd, 0, 90, "[NOK]\t| Sprite leanmeter: Initialisation Failed.", 200);
    Serial.println(F("Failed to allocate sprite leanmeter memory"));
    return;
  }
  setCursor_Text_Delay(lcd, 0, 90, "[OK]\t| Sprite leanmeter: Initialisation Success.", 200);
  setCursor_Text_Delay(lcd, 0, 100, "[START]\t| Bno08x: Initialisation Started.", 100);
  while (!bno08x.begin_I2C(0x4B)) {
    setCursor_Text_Delay(lcd, 0, 110, "[NOK]\t| Bno08x: Initialisation Failed.", 200);
    setCursor_Text_Delay(lcd, 0, 120, "Please Verify connection, alimentation. or add resistors", 200);
    Serial.println("Erreur : Impossible de trouver le composant BNO08x !");
    delay(1000);
  }
  setCursor_Text_Delay(lcd, 0, 120, "                                                                                ", 200);
  setCursor_Text_Delay(lcd, 0, 110, "[OK]\t| Bno08x: Initialisation Success.", 200);
  setCursor_Text_Delay(lcd, 0, 120, "[START]\t| SD: Initialisation Started.", 200);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(50);
  sdSPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
  delay(1000);
  while (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_SPEED, &sdSPI))) {
    setCursor_Text_Delay(lcd, 0, 130, "[NOK]\t| SD: Initialisation Failed.", 50);
    Serial.println("\n====== SD CARD INITIALIZATION FAILED ======");
    sd.initErrorPrint(&Serial); // <-- Prints exact error message and code
    
    Serial.print("SdFat Version: ");
    Serial.println(SD_FAT_VERSION);
    Serial.println("===========================================\n");
    Serial.println("Erreur : Impossible de trouver le composant de la SD CARD !");
    delay(1000);
  }
  setCursor_Text_Delay(lcd, 0, 130, "[OK] \t| SD: Initialisation Success.            ", 50);
  setCursor_Text_Delay(lcd, 0, 140, "[START]\t| GPS: Initialisation Started.", 50);
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  bool gpsSynced = false;
  int date[6] = {0};
  int i = 0;
  char filename[32];
  while (!gpsSynced) {
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
      lcd.setCursor(i, 150);
      if (i >= 500)
      {
        lcd.setCursor(0, 150);
        lcd.print("                                                        ");
        i = 0;
        break ;
      }
      if (gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2000)
        gpsSynced = true;
      i++;
    }
    date[0] = gps.date.year();
    date[1] = gps.date.month();
    date[2] = gps.date.day();
    date[3] = gps.time.hour();
    date[4] = gps.time.minute();
    date[5] = gps.time.second();
    memset(filename, 0, 32);
    snprintf(filename, sizeof(filename), "/%04d_%02d_%02d_%02d_%02d_%02d.txt", date[0], date[1], date[2], date[3], date[4], date[5]);
    lcd.setCursor(0, 150);
    lcd.println(filename);
  }
  rtc.setTime(date[5], date[4], date[3], date[2], date[1], date[0]);
  f = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
  if (!f) {
    Serial.println("could not create the file");
    setCursor_Text_Delay(lcd, 0, 160, "[NOK]\t| SD: n'as pas pu créer le fichier.            .", 250);
    return ((void)Serial.println("Erreur: n'as pas pu créer le fichier"));
  }
  setCursor_Text_Delay(lcd, 0, 160, "[OK]\t| SD: File opened.            .", 250);

  lcd.fillScreen(TFT_WHITE);
  cubeSprite.setTextSize(2);
  cubeSprite.setTextColor(TFT_WHITE);
  leanMeter.setTextSize(2);
  leanMeter.setTextColor(TFT_WHITE);
  memset(dump.buff, 0, MAX_BUFF_SIZE);
  dump.len = 0;
  setReport();
}

void addToDump(void *dest, unsigned short len) {
  if (dump.len + len > MAX_BUFF_SIZE) {
    Serial.println("Warning: Buffer overflow! Data dropped because SD write is slow.");
    return; 
  }

  memcpy(dump.buff + dump.len, dest, len);
  dump.len += len;
}

unsigned char counter = 0;
void dumpInSdCard(void) {
  if (dump.len < DUMP_SIZE) {
    return;
  }

  size_t written = f.write(dump.buff, DUMP_SIZE);
  if (written != DUMP_SIZE) {
    memset(dump.buff, 0, MAX_BUFF_SIZE);
    dump.len = 0;
    return ((void)Serial.println("Erreur: n'as pas pu tout écrire"));
  }

  counter++;
  if (counter >= 10) {
    f.flush();
    counter = 0;
  }
  int16_t remainingBytes = dump.len - DUMP_SIZE;
    if (remainingBytes > 0) {
    memmove(dump.buff, dump.buff + DUMP_SIZE, remainingBytes);
    dump.len = remainingBytes;
  } else {
    dump.len = 0;
  }
}

void loop() {
  uint8_t checkBnoGps = 0;
  if (bno08x.wasReset()) {
    Serial.println("bno085 got reset");
    setReport();
    delay(100);
    return;
  }

  float qw = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float accel_x = 0;
  float accel_y = 0;
  float accel_z = 0;

  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        accel_x = sensorValue.un.linearAcceleration.x;
        accel_y = sensorValue.un.linearAcceleration.y;
        accel_z = sensorValue.un.linearAcceleration.z;
        checkBnoGps |= BNO_ACCELEROMETER;
        break;
      case SH2_GAME_ROTATION_VECTOR:
        qw = sensorValue.un.gameRotationVector.real;
        roll = sensorValue.un.gameRotationVector.i;
        pitch = sensorValue.un.gameRotationVector.j;
        yaw = sensorValue.un.gameRotationVector.k;
        //float mag = sqrt(qw*qw + roll*roll + pitch*pitch + yaw*yaw);
        //if (mag > 0)
        //{ qw /= mag; roll /= mag; pitch /= mag; yaw /= mag; }
        //cubeSprite.fillScreen(TFT_BLACK);
        //cubeSprite.setCursor(0, 0);
        //cubeSprite.print("cube");
        //drawCube(cubeSprite, qw, -pitch, yaw, -roll, HEIGHT_DIV >> 1, WIDTH_DIV, 2.2f);
        //cubeSprite.pushSprite(0, 0);
        //leanMeter.fillScreen(TFT_BLACK);altitude
        //leanMeter.setCursor(0, 0);
        //leanMeter.setTextSize(2);
        //leanMeter.print("lean");
        //drawGauge(leanMeter, quaternionToRoll(qw, roll, pitch, yaw), HEIGHT_DIV >> 1, WIDTH_DIV + (54 >> 1));
        //leanMeter.pushSprite(HEIGHT_DIV, 0);
        checkBnoGps |= BNO_ROTATION_VECTOR;
        break;
    }
    if (checkBnoGps & (BNO_ROTATION_VECTOR | BNO_ACCELEROMETER)) {
      const unsigned long currMillis = micros();
      uint16_t diff = currMillis - previousMillis;
      previousMillis = currMillis;
      while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
      }
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        leanMeter.fillSprite(TFT_BLACK);
        leanMeter.setCursor(0,0);
        leanMeter.print("latitude: ");
        leanMeter.println(latitude, 6);
        leanMeter.print("longitude: ");
        leanMeter.println(longitude, 6);
        leanMeter.pushSprite(0, 0);
        if (gps.altitude.isValid()) {
          altitude = gps.altitude.meters();
          leanMeter.print("altitude: ");
          leanMeter.print(altitude, 2);
          leanMeter.println("m");
        }
      }
      lastGpsUpdateMicros = currMillis;
      addToDump(&diff, sizeof(uint16_t));
      addToDump(&qw, sizeof(float));
      addToDump(&pitch, sizeof(float));
      addToDump(&roll, sizeof(float));
      addToDump(&yaw, sizeof(float));
      addToDump(&accel_x, sizeof(float));
      addToDump(&accel_y, sizeof(float));
      addToDump(&accel_z, sizeof(float));
      addToDump(&latitude, sizeof(float));
      addToDump(&longitude, sizeof(float));
      addToDump(&altitude, sizeof(float));
      if (dump.len >= MAX_BUFF_SIZE - 1) {
        memset(dump.buff, 0, MAX_BUFF_SIZE);
        dump.len = 0;
        printf("destroying buffer\n");
      } else if (dump.len >= DUMP_SIZE) {
        dumpInSdCard();
        printf("Writing in sd card\n");
      }
    }
  }
  delay(DELAY_HZ);
}
