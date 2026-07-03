#include <math.h>

struct Box {
  static constexpr float HALF = 10.0f;
  const float x[8] = {
    -HALF, HALF, HALF, -HALF, -HALF, HALF, HALF, -HALF
  };
  const float y[8] = {
    -HALF, -HALF, HALF, HALF, -HALF, -HALF, HALF, HALF
  };
  const float z[8] = {
    -HALF - 10, -HALF - 10, -HALF - 10, -HALF - 10, HALF + 10, HALF + 10, HALF + 10, HALF + 10
  };
};
static Box box;

const uint8_t CUBE_EDGES[24] = {
  0, 1, 1, 2, 2, 3, 3, 0,
  4, 5, 5, 6, 6, 7, 7, 4,
  0, 4, 1, 5, 2, 6, 3, 7
};

void rotatePointByQuaternion(float x, float y, float z,
  float qw, float qx, float qy, float qz,
  float &rx, float &ry, float &rz)
{
  float t0 = qw * x + qy * z - qz * y;
  float t1 = qw * y + qz * x - qx * z;
  float t2 = qw * z + qx * y - qy * x;
  float t3 = -qx * x - qy * y - qz * z;
  
  rx = t0 * qw + t3 * (-qx) + t1 * (-qz) - t2 * (-qy);
  ry = t1 * qw + t3 * (-qy) + t2 * (-qx) - t0 * (-qz);
  rz = t2 * qw + t3 * (-qz) + t0 * (-qy) - t1 * (-qx);
}

inline void project(float x, float y,
  int16_t &sx, int16_t &sy,
  int16_t cx, int16_t cy, float scale)
{
  sx = static_cast<int16_t>(cx + x * scale);
  sy = static_cast<int16_t>(cy - y * scale);
}


void drawCube(LGFX_Sprite &canvas,
  float qw, float qx, float qy, float qz,
  int16_t cx, int16_t cy, float scale)
{
  float xr[8], yr[8], zr[8];
  for (uint8_t i = 0; i < 8; ++i) {
    rotatePointByQuaternion(
      box.x[i], box.y[i], box.z[i],
      qw, qx, qy, qz,
      xr[i], yr[i], zr[i]);
  }

  for (uint8_t i = 0; i < 24; i += 2) {
    uint8_t a = CUBE_EDGES[i];
    uint8_t b = CUBE_EDGES[i + 1];

    int16_t x0, y0, x1, y1;
    project(xr[a], yr[a], x0, y0, cx, cy, scale);
    project(xr[b], yr[b], x1, y1, cx, cy, scale);

    if (i <= 7)
      canvas.drawLine(x0, y0, x1, y1, TFT_DARKCYAN);
    else if (i <= 14)
      canvas.drawLine(x0, y0, x1, y1, TFT_DARKTURQUOISE);
    else if (i <= 16)
      canvas.drawLine(x0, y0, x1, y1, TFT_RED);
    else if (i <= 18)
      canvas.drawLine(x0, y0, x1, y1, TFT_YELLOW);
    else if (i < 22)
      canvas.drawLine(x0, y0, x1, y1, TFT_WHITE);
    else if (i <= 24)
      canvas.drawLine(x0, y0, x1, y1, TFT_PURPLE);
  }
}



void drawGauge(LGFX_Sprite &leanMeter, float roll, uint16_t cx, uint16_t cy)
{
  const int16_t rOut = 54;
  const int16_t rIn  = 42;
  float rad;

  for (int a = -70; a <= 70; a += 10)
  {
    rad = (a - 90) * DEG_TO_RAD;
    int x1 = cx + cos(rad) * rIn;
    int y1 = cy + sin(rad) * rIn;
    int x2 = cx + cos(rad) * rOut;
    int y2 = cy + sin(rad) * rOut;
    if (a == 0)
      leanMeter.drawLine(x1, y1, x2, y2, TFT_GREEN);
    else if (a == 50 || a == -50)
      leanMeter.drawLine(x1, y1, x2, y2, TFT_RED);
    else
      leanMeter.drawLine(x1, y1, x2, y2, TFT_DARKGREY);
  }

  leanMeter.setTextSize(1);
  leanMeter.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  for (int a = -60; a <= 60; a += 30) {
    if (a == 0)
      continue;
    float rad = (a - 90) * DEG_TO_RAD;
    int tx = cx + cos(rad) * (rOut + 12) - 4;
    int ty = cy + sin(rad) * (rOut + 12) - 4;
    leanMeter.setCursor(tx, ty);
    leanMeter.print(abs(a));
  }
  leanMeter.setCursor(cx - 3, cy - rOut - (rOut - rIn) - 8);
  leanMeter.print("0");
  
  float needle = constrain(roll, -1.308f, 1.308f);
  int nx = cx + sin(needle) * (rOut - 10);
  int ny = cy - cos(needle) * (rOut - 10);
  leanMeter.drawLine(cx, cy, nx, ny, TFT_YELLOW);
  leanMeter.fillCircle(cx, cy, 3, TFT_YELLOW);

  leanMeter.setTextSize(2);
  leanMeter.setTextColor(TFT_WHITE, TFT_BLACK);
  leanMeter.setCursor(cx - 42, cy + 32);
  roll = roll * RAD_TO_DEG;
  char buf[20];
  if (roll >= 0)
    sprintf(buf, "R %5.1f", roll);
  else
    sprintf(buf, "L %5.1f", -roll);
  leanMeter.print(buf);
  leanMeter.print((char)247);
}
