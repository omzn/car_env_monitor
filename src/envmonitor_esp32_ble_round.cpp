#include <SPI.h>
//#include <WiFi.h>
#include <Wire.h>
// #include <OneWire.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SparkFun_MMA8452Q.h>  // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <SparkFun_SCD4x_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x

// #include "BLEDevice.h"
#include "BLE_TPMS.h"
#include "DHT12.h"
#include "LGFX_GC9A01.h"
#include "MazdaTypeBold18pt.h"
#include "MazdaTypeBold24pt.h"
#include "aquatan_eye.h"
#include "bitmaps.h"
#include "debugmacros.h"

#define PIN_TOUCH GPIO_NUM_13
#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22
#define PIN_SDA2 GPIO_NUM_25
#define PIN_SCL2 GPIO_NUM_27
#define PIN_TX2 GPIO_NUM_17
#define PIN_RX2 GPIO_NUM_16

// #define FONT_SANS24_IMG img->setFont(&fonts::FreeSans24pt7b)
#define FONT_SANS24_IMG img->setFont(&Mazda_Type_Bold24pt7b)
// #define FONT_SANS18_IMG img->setFont(&fonts::FreeSans18pt7b)
#define FONT_SANS18_IMG img->setFont(&Mazda_Type_Bold18pt7b)
#define FONT_SANS_IMG img->setFont(&fonts::FreeSans9pt7b)

#define TFT_GRAY96 img->color888(96, 96, 96)
#define TFT_GRAY64 img->color888(64, 64, 64)
#define TFT_GRAY48 img->color888(48, 48, 48)
#define TFT_GRAY32 img->color888(32, 32, 32)
#define TFT_GRAY16 img->color888(16, 16, 16)

#define TPRESS_MIN (200)
#define TPRESS_1ST (225)
#define TPRESS_2ND (250)
#define TPRESS_3RD (275)
#define TPRESS_MAX (300)

#define CO2_MIN (0)
#define CO2_1ST (800)
#define CO2_2ND (1200)
#define CO2_3RD (2000)
#define CO2_MAX (2400)

#define TEMP_MIN (-100)
#define TEMP_1ST (100)
#define TEMP_2ND (200)
#define TEMP_3RD (300)
#define TEMP_4TH (400)
#define TEMP_MAX (500)

#define PRESS_MIN (950)
#define PRESS_1ST (980)
#define PRESS_2ND (1000)
#define PRESS_3RD (1010)
#define PRESS_MAX (1040)

#define HUMID_MIN (0)
#define HUMID_1ST (200)
#define HUMID_2ND (300)
#define HUMID_3RD (400)
#define HUMID_MAX (600)

#define CO2_DEG(v) \
  ((90 + constrain(map(v, CO2_MIN, CO2_MAX, 45, 315), 45, 315)) % 360)
#define TEMP_DEG(v) \
  ((90 + constrain(map(v, TEMP_MIN, TEMP_MAX, 45, 315), 45, 315)) % 360)
#define PRESS_DEG(v) \
  ((90 + constrain(map(v, PRESS_MIN, PRESS_MAX, 45, 315), 45, 315)) % 360)
#define HUMID_DEG(v) \
  ((90 + constrain(map(v, HUMID_MIN, HUMID_MAX, 45, 315), 45, 315)) % 360)

#define TPRESS_DEG_L_UP(v) \
  ((180 + constrain(map(v, TPRESS_MIN, TPRESS_MAX, 25, 85), 25, 85)) % 360)
#define TPRESS_DEG_R_UP(v) \
  ((270 + constrain(map(v, TPRESS_MIN, TPRESS_MAX, 65, 5), 5, 65)) % 360)
#define TPRESS_DEG_L_DOWN(v) \
  ((90 + constrain(map(v, TPRESS_MIN, TPRESS_MAX, 65, 5), 5, 65)) % 360)
#define TPRESS_DEG_R_DOWN(v) \
  ((0 + constrain(map(v, TPRESS_MIN, TPRESS_MAX, 25, 85), 25, 85)) % 360)

#define CO2_COLOR(v) \
  (v < CO2_1ST       \
       ? TFT_CYAN    \
       : (v < CO2_2ND ? TFT_GREEN : (v < CO2_3RD ? TFT_YELLOW : TFT_RED)))
#define CO2_COLOR1(v)                                                     \
  (v < CO2_1ST ? img->color888(0, 191, 191)                               \
               : (v < CO2_2ND ? img->color888(0, 191, 0)                  \
                              : (v < CO2_3RD ? img->color888(191, 191, 0) \
                                             : img->color888(191, 0, 0))))
#define CO2_COLOR2(v)                                                     \
  (v < CO2_1ST ? img->color888(0, 127, 127)                               \
               : (v < CO2_2ND ? img->color888(0, 127, 0)                  \
                              : (v < CO2_3RD ? img->color888(127, 127, 0) \
                                             : img->color888(127, 0, 0))))
#define CO2_COLOR3(v)                                                   \
  (v < CO2_1ST ? img->color888(0, 63, 63)                               \
               : (v < CO2_2ND ? img->color888(0, 63, 0)                 \
                              : (v < CO2_3RD ? img->color888(63, 63, 0) \
                                             : img->color888(63, 0, 0))))
#define CO2_COLOR4(v)                                                   \
  (v < CO2_1ST ? img->color888(0, 32, 32)                               \
               : (v < CO2_2ND ? img->color888(0, 32, 0)                 \
                              : (v < CO2_3RD ? img->color888(32, 32, 0) \
                                             : img->color888(32, 0, 0))))

#define TEMP_COLOR(v)                     \
  (v < TEMP_1ST                           \
       ? TFT_BLUE                         \
       : (v < TEMP_2ND                    \
              ? TFT_CYAN                  \
              : (v < TEMP_3RD ? TFT_GREEN \
                              : (v < TEMP_4TH ? TFT_YELLOW : TFT_RED))))
#define TEMP_COLOR1(v)                                                     \
  (v < TEMP_1ST                                                            \
       ? img->color888(0, 0, 191)                                          \
       : (v < TEMP_2ND                                                     \
              ? img->color888(0, 191, 191)                                 \
              : (v < TEMP_3RD ? img->color888(0, 191, 0)                   \
                              : (v < TEMP_4TH ? img->color888(191, 191, 0) \
                                              : img->color888(191, 0, 0)))))
#define TEMP_COLOR2(v)                                                     \
  (v < TEMP_1ST                                                            \
       ? img->color888(0, 0, 127)                                          \
       : (v < TEMP_2ND                                                     \
              ? img->color888(0, 127, 127)                                 \
              : (v < TEMP_3RD ? img->color888(0, 127, 0)                   \
                              : (v < TEMP_4TH ? img->color888(127, 127, 0) \
                                              : img->color888(127, 0, 0)))))
#define TEMP_COLOR3(v)                                                   \
  (v < TEMP_1ST                                                          \
       ? img->color888(0, 0, 63)                                         \
       : (v < TEMP_2ND                                                   \
              ? img->color888(0, 63, 63)                                 \
              : (v < TEMP_3RD ? img->color888(0, 63, 0)                  \
                              : (v < TEMP_4TH ? img->color888(63, 63, 0) \
                                              : img->color888(63, 0, 0)))))
#define TEMP_COLOR4(v)                                                   \
  (v < TEMP_1ST                                                          \
       ? img->color888(0, 0, 31)                                         \
       : (v < TEMP_2ND                                                   \
              ? img->color888(0, 31, 31)                                 \
              : (v < TEMP_3RD ? img->color888(0, 31, 0)                  \
                              : (v < TEMP_4TH ? img->color888(31, 31, 0) \
                                              : img->color888(31, 0, 0)))))
#define PRESS_COLOR(v)                         \
  (v < PRESS_1ST ? TFT_RED                     \
                 : (v < PRESS_2ND ? TFT_YELLOW \
                                  : (v < PRESS_3RD ? TFT_GREEN : TFT_CYAN)))
#define PRESS_COLOR1(v)                                             \
  (v < PRESS_1ST                                                    \
       ? img->color888(191, 0, 0)                                   \
       : (v < PRESS_2ND ? img->color888(191, 191, 0)                \
                        : (v < PRESS_3RD ? img->color888(0, 191, 0) \
                                         : img->color888(0, 191, 191))))
#define PRESS_COLOR2(v)                                             \
  (v < PRESS_1ST                                                    \
       ? img->color888(127, 0, 0)                                   \
       : (v < PRESS_2ND ? img->color888(127, 127, 0)                \
                        : (v < PRESS_3RD ? img->color888(0, 127, 0) \
                                         : img->color888(0, 127, 127))))
#define PRESS_COLOR3(v)                                            \
  (v < PRESS_1ST                                                   \
       ? img->color888(63, 0, 0)                                   \
       : (v < PRESS_2ND ? img->color888(63, 63, 0)                 \
                        : (v < PRESS_3RD ? img->color888(0, 63, 0) \
                                         : img->color888(0, 63, 63))))
#define PRESS_COLOR4(v)                                            \
  (v < PRESS_1ST                                                   \
       ? img->color888(32, 0, 0)                                   \
       : (v < PRESS_2ND ? img->color888(32, 32, 0)                 \
                        : (v < PRESS_3RD ? img->color888(0, 32, 0) \
                                         : img->color888(0, 32, 32))))

#define HUMID_COLOR(v)                         \
  (v < HUMID_1ST ? TFT_RED                     \
                 : (v < HUMID_2ND ? TFT_YELLOW \
                                  : (v < HUMID_3RD ? TFT_GREEN : TFT_CYAN)))
#define HUMID_COLOR1(v)                                             \
  (v < HUMID_1ST                                                    \
       ? img->color888(191, 0, 0)                                   \
       : (v < HUMID_2ND ? img->color888(191, 191, 0)                \
                        : (v < HUMID_3RD ? img->color888(0, 191, 0) \
                                         : img->color888(0, 191, 191))))
#define HUMID_COLOR2(v)                                             \
  (v < HUMID_1ST                                                    \
       ? img->color888(127, 0, 0)                                   \
       : (v < HUMID_2ND ? img->color888(127, 127, 0)                \
                        : (v < HUMID_3RD ? img->color888(0, 127, 0) \
                                         : img->color888(0, 127, 127))))
#define HUMID_COLOR3(v)                                            \
  (v < HUMID_1ST                                                   \
       ? img->color888(63, 0, 0)                                   \
       : (v < HUMID_2ND ? img->color888(63, 63, 0)                 \
                        : (v < HUMID_3RD ? img->color888(0, 63, 0) \
                                         : img->color888(0, 63, 63))))
#define HUMID_COLOR4(v)                                            \
  (v < HUMID_1ST                                                   \
       ? img->color888(31, 0, 0)                                   \
       : (v < HUMID_2ND ? img->color888(31, 31, 0)                 \
                        : (v < HUMID_3RD ? img->color888(0, 31, 0) \
                                         : img->color888(0, 31, 31))))

#define TPRESS_COLOR(v) \
  (v < TPRESS_1ST       \
       ? TFT_GOLD       \
       : (v < TPRESS_2ND ? TFT_CYAN : (v < TPRESS_3RD ? TFT_CYAN : TFT_GOLD)))
#define TPRESS_COLOR1(v)                                     \
  (v < TPRESS_1ST                                            \
       ? img->color888(191, 215 - 54, 0)                     \
       : (v < TPRESS_2ND                                     \
              ? img->color888(0, 191, 191)                   \
              : (v < TPRESS_3RD ? img->color888(0, 191, 191) \
                                : img->color888(191, 215 - 54, 0))))
#define TPRESS_COLOR2(v)                                     \
  (v < TPRESS_1ST                                            \
       ? img->color888(127, 215 - 54 - 54, 0)                \
       : (v < TPRESS_2ND                                     \
              ? img->color888(0, 127, 127)                   \
              : (v < TPRESS_3RD ? img->color888(0, 127, 127) \
                                : img->color888(127, 215 - 54 - 54, 0))))
#define TPRESS_COLOR3(v)                                   \
  (v < TPRESS_1ST                                          \
       ? img->color888(63, 215 - 54 - 54 - 54, 0)          \
       : (v < TPRESS_2ND                                   \
              ? img->color888(0, 63, 63)                   \
              : (v < TPRESS_3RD ? img->color888(0, 63, 63) \
                                : img->color888(127, 215 - 54 - 54 - 54, 0))))
#define TPRESS_COLOR4(v)                               \
  (v < TPRESS_1ST                                      \
       ? img->color888(31, 215 - 54 - 54 - 54 - 27, 0) \
       : (v < TPRESS_2ND                               \
              ? img->color888(0, 31, 31)               \
              : (v < TPRESS_3RD                        \
                     ? img->color888(0, 31, 31)        \
                     : img->color888(31, 215 - 54 - 54 - 54 - 27, 0))))

#define SENSOR_HIST (90)
#define NUM_OF_VIEWS (4)

#define ACCEL_RANGE (0.20)

SCD4x sensorCO2;
DHT12 sensorTempHumid;
Adafruit_BMP280 sensorPressure;

uint32_t seq = 0;  // remember number of boots in RTC Memory
uint32_t stable = 0;
uint8_t view = 0;
uint8_t p_view = 0;

uint32_t last_touched = 0;
uint8_t btnint = 0;

// 準備したクラスのインスタンスを作成します。
static LGFX_MiniKit_GC9A01_0 tft0;
static LGFX_MiniKit_GC9A01_1 tft1;
static LGFX_Sprite img120x120(&tft1);
static LGFX_Sprite img140x60(&tft1);
static LGFX_Sprite img160x72(&tft1);
Adafruit_SSD1306 oled1(64, 32, &Wire, -1);
Adafruit_SSD1306 oled2(64, 32, &Wire1, -1);

MMA8452Q accel;  // create instance of the MMA8452 class

NimBLEScan *pBLEScan;
BLEtpms tpms[4];
float prev_tpress[4] = {0};

Preferences prefs;

AquatanEye eye_left(&oled1);
AquatanEye eye_right(&oled2);
AquatanEyes eyes(&eye_left, &eye_right);

float temperature_hist[SENSOR_HIST];
float pressure_hist[SENSOR_HIST] = {0};
float humidity_hist[SENSOR_HIST] = {0};
float co2_hist[SENSOR_HIST] = {0};
int temperature_hist_p = 0;
int pressure_hist_p = 0;
int humidity_hist_p = 0;
int co2_hist_p = 0;

uint8_t use_accel = 0;
float y_accel_center;
float z_accel_center;
int face_shape = 0;

enum { FACE_NORMAL = 0, FACE_GOOD, FACE_DIRTY, FACE_NODATA, FACE_GURUGURU };

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice *advertisedDevice) {
        Serial.printf("Found %s with RSSI %d \n", //millis() - lastAdv,
         advertisedDevice->getAddress().toString().c_str(),
         advertisedDevice->getRSSI());
    // Serial.printf("Advertised Device: %s \n RSSI: %d \n",
    // advertisedDevice->toString().c_str(), advertisedDevice->getRSSI());
    int tire = -1;
    if (advertisedDevice->haveManufacturerData() == true) {
      std::string data = advertisedDevice->getManufacturerData();
      if (BLEtpms::isManufacturerId(data)) {
        tire = BLEtpms::tire_id(data);
        if (tire >= 0) {
          tpms[tire].scan(data);
          tpms[tire].updated(true);
          Serial.printf(">>> tire %d, p: %.1f, t: %.1f, b: %.0f\n", tire,
                        tpms[tire].pressure(), tpms[tire].temp() / 100.0,
                        tpms[tire].battery());
        }
      }
    }
  }
};

String format_digit(float f, int digits, int decimal = 0) {
  int divnum = pow(10, digits - 1 + decimal);
  int zeroflag = 1;
  int negativeflag = 0;
  String s = "";
  int num = (int)(f * pow(10, decimal));
  if (num < 0) {
    num = -num;
    negativeflag = 1;
  }
  //  Serial.print("num=");
  //  Serial.println(num);
  for (int i = 0; i < digits + decimal; i++) {
    if (num / divnum == 0) {
      if (zeroflag == 1) {
        if (i == digits - 1) {
          zeroflag = 0;
          s += "0";
          if (decimal > 0) {
            s += ".";
          }
        } else {
          s += " ";
        }
      } else {
        s += "0";
        if (i == digits - 1 && decimal > 0) {
          s += ".";
        }
      }
    } else {
      s += String(num / divnum);
      if (i == digits - 1 && decimal > 0) {
        s += ".";
      }
      zeroflag = 0;
    }
    num %= divnum;
    divnum /= 10;
    //    Serial.println(s);
  }
  if (negativeflag) {
    int i = s.lastIndexOf(' ');
    if (i >= 0) {
      s.setCharAt(i, '-');
    } else {
      s = '-' + s;
    }
  }
  return s;
}

void handleTouch() {
  if (millis() - last_touched > 500) {
    btnint = 1;
    last_touched = millis();
  }
}

void drawBmp(LGFX_Device *sp, unsigned char *data, int16_t x, int16_t y,
             int16_t w, int16_t h) {
  uint16_t row, col, buffidx = 0;
  for (col = 0; col < w; col++) {  // For each scanline...
    for (row = 0; row < h; row++) {
      uint16_t c = pgm_read_word(data + buffidx);
      c = ((c >> 8) & 0x00ff) | ((c << 8) & 0xff00);  // swap back and fore
      sp->drawPixel(col + x, row + y, c);
      buffidx += 2;
    }  // end pixel
  }
}

void tpmsValueBox(LGFX_Sprite *img, int x, int y, BLEtpms *tp) {
  img->fillSprite(TFT_TRANSPARENT);
  //  img->setTextColor(TEMP_COLOR(temp * 10));
  img->setTextColor(TFT_WHITE);
  if (tp->pressure() > 0) {
    String str = format_digit((tp->pressure() / 100.0) + 0.05, 2,
                              1);  // 小数点第2位を四捨五入
    String sint = str.substring(0, str.length() - 1);
    String sdecimal = str.substring(str.length() - 1);
    FONT_SANS24_IMG;
    int intWidth = img->textWidth(sint);
    int intHeight = img->fontHeight();
    FONT_SANS18_IMG;
    int decimalWidth = img->textWidth(sdecimal);
    int decimalHeight = img->fontHeight();
    FONT_SANS24_IMG;
    img->drawString(sint, img->width() / 2 - ((intWidth + decimalWidth) / 2),
                    0);
    FONT_SANS18_IMG;
    img->drawString(
        sdecimal, img->width() / 2 - ((intWidth + decimalWidth) / 2) + intWidth,
        intHeight - decimalHeight - 2);
  } else {
    FONT_SANS24_IMG;
    img->setTextColor(TFT_SILVER);
    String str = "---";
    img->drawString(str, img->width() / 2 - img->textWidth(str) / 2, 0);
  }
  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void tpmsViewLeftUp(LGFX_Sprite *img, int x, int y, BLEtpms *tp) {
  img->fillSprite(TFT_TRANSPARENT);
  FONT_SANS_IMG;
  img->setTextColor(TFT_SILVER);
  int inc = ((TPRESS_MAX - TPRESS_MIN) / 10);
  for (int i = TPRESS_MIN; i < TPRESS_MAX; i += inc) {
    if (tp->pressure() >= i && tp->pressure() < i + inc) {
      img->fillArc(119, 119, 113, 118, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TPRESS_COLOR(i));
      img->fillArc(119, 119, 109, 113, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TPRESS_COLOR1(i));
      img->fillArc(119, 119, 105, 109, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TPRESS_COLOR2(i));
      img->fillArc(119, 119, 102, 105, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TPRESS_COLOR3(i));
      img->drawArc(119, 119, 101, 119, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_RED);
      img->fillArc(119, 119, 100, 102, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TPRESS_COLOR4(i));

      img->fillArc(119, 119, 70, 101, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(119, 119, 75, 101, TPRESS_DEG_L_UP(i) + 1,
                   (TPRESS_DEG_L_UP(i + inc) - 3) % 360, TFT_GRAY96);
      img->fillArc(119, 119, 75, 110, TPRESS_DEG_L_UP(i) + 2,
                   (TPRESS_DEG_L_UP(i + inc) - 3) % 360, TFT_WHITE);

      //      img->drawArc(119, 119, 103, 118, TPRESS_DEG_L_UP(i),
      //                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_WHITE);

    } else {
      img->fillArc(119, 119, 113, 118, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(119, 119, 109, 113, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY48);
      img->fillArc(119, 119, 105, 109, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY32);
      img->fillArc(119, 119, 102, 105, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY16);
      img->drawArc(119, 119, 101, 119, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(119, 119, 100, 102, TPRESS_DEG_L_UP(i),
                   (TPRESS_DEG_L_UP(i + inc) - 2) % 360, TFT_BLACK);
    }
  }
  if (tp->pressure() > 0.0) {
    img->fillRoundRect(10, 93, 20, 12, 3,
                       tp->battery() > 60
                           ? TFT_DARKGREEN
                           : (tp->battery() > 50 ? TFT_GOLD : TFT_RED));
    img->drawRoundRect(10, 93, 20, 12, 3, TFT_LIGHTGRAY);
    img->drawRoundRect(11, 94, 18, 10, 3, TFT_LIGHTGRAY);
    img->fillRect(8, 97, 2, 4, TFT_LIGHTGRAY);
  }
  img->pushSprite(x, y, TFT_TRANSPARENT);
  tp->updated(false);
}

void tpmsViewLeftDown(LGFX_Sprite *img, int x, int y, BLEtpms *tp) {
  img->fillSprite(TFT_TRANSPARENT);
  FONT_SANS_IMG;
  img->setTextColor(TFT_SILVER);
  int inc = ((TPRESS_MAX - TPRESS_MIN) / 10);
  for (int i = TPRESS_MIN; i < TPRESS_MAX; i += inc) {
    if (tp->pressure() >= i && tp->pressure() < i + inc) {
      img->fillArc(119, 0, 113, 118, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TPRESS_COLOR(i));
      img->fillArc(119, 0, 109, 113, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TPRESS_COLOR1(i));
      img->fillArc(119, 0, 105, 109, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TPRESS_COLOR2(i));
      img->fillArc(119, 0, 102, 105, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TPRESS_COLOR3(i));
      img->drawArc(119, 0, 101, 119, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_RED);
      img->fillArc(119, 0, 100, 102, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TPRESS_COLOR4(i));

      img->fillArc(119, 0, 70, 101, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY64);
      img->fillArc(119, 0, 75, 101, (TPRESS_DEG_L_DOWN(i + inc) + 3) % 360,
                   TPRESS_DEG_L_DOWN(i) - 1, TFT_GRAY96);
      img->fillArc(119, 0, 75, 110, (TPRESS_DEG_L_DOWN(i + inc) + 3) % 360,
                   TPRESS_DEG_L_DOWN(i) - 2, TFT_WHITE);
    } else {
      img->fillArc(119, 0, 113, 118, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY64);
      img->fillArc(119, 0, 109, 113, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY48);
      img->fillArc(119, 0, 105, 109, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY32);
      img->fillArc(119, 0, 102, 105, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY16);
      img->drawArc(119, 0, 101, 119, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_GRAY64);
      img->fillArc(119, 0, 100, 102, (TPRESS_DEG_L_DOWN(i + inc) + 2) % 360,
                   TPRESS_DEG_L_DOWN(i), TFT_BLACK);
    }
  }
  if (tp->pressure() > 0.0) {
    img->fillRoundRect(10, 7, 20, 12, 3,
                       tp->battery() > 60
                           ? TFT_DARKGREEN
                           : (tp->battery() > 50 ? TFT_GOLD : TFT_RED));
    img->drawRoundRect(10, 7, 20, 12, 3, TFT_LIGHTGRAY);
    img->drawRoundRect(11, 8, 18, 10, 3, TFT_LIGHTGRAY);
    img->fillRect(8, 11, 2, 4, TFT_LIGHTGRAY);
  }
  img->pushSprite(x, y, TFT_TRANSPARENT);
  tp->updated(false);
}

void tpmsViewRightUp(LGFX_Sprite *img, int x, int y, BLEtpms *tp) {
  img->fillSprite(TFT_TRANSPARENT);
  FONT_SANS_IMG;
  img->setTextColor(TFT_SILVER);
  int inc = ((TPRESS_MAX - TPRESS_MIN) / 10);
  for (int i = TPRESS_MIN; i < TPRESS_MAX; i += inc) {
    if (tp->pressure() >= i && tp->pressure() < i + inc) {
      img->fillArc(0, 119, 113, 118, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TPRESS_COLOR(i));
      img->fillArc(0, 119, 109, 113, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TPRESS_COLOR1(i));
      img->fillArc(0, 119, 105, 109, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TPRESS_COLOR2(i));
      img->fillArc(0, 119, 102, 105, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TPRESS_COLOR3(i));
      img->drawArc(0, 119, 101, 119, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_RED);
      img->fillArc(0, 119, 100, 102, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TPRESS_COLOR4(i));

      img->fillArc(0, 119, 70, 101, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY64);
      img->fillArc(0, 119, 75, 101, (TPRESS_DEG_R_UP(i + inc) + 3) % 360,
                   TPRESS_DEG_R_UP(i) - 1, TFT_GRAY96);
      img->fillArc(0, 119, 75, 110, (TPRESS_DEG_R_UP(i + inc) + 3) % 360,
                   TPRESS_DEG_R_UP(i) - 2, TFT_WHITE);
    } else {
      img->fillArc(0, 119, 113, 118, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY64);
      img->fillArc(0, 119, 109, 113, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY48);
      img->fillArc(0, 119, 105, 109, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY32);
      img->fillArc(0, 119, 102, 105, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY16);
      img->drawArc(0, 119, 101, 119, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_GRAY64);
      img->fillArc(0, 119, 100, 102, (TPRESS_DEG_R_UP(i + inc) + 2) % 360,
                   TPRESS_DEG_R_UP(i), TFT_BLACK);
    }
  }
  if (tp->pressure() > 0.0) {
    img->fillRoundRect(90, 93, 20, 12, 3,
                       tp->battery() > 60
                           ? TFT_DARKGREEN
                           : (tp->battery() > 50 ? TFT_GOLD : TFT_RED));
    img->drawRoundRect(90, 93, 20, 12, 3, TFT_LIGHTGRAY);
    img->drawRoundRect(89, 94, 18, 10, 3, TFT_LIGHTGRAY);
    img->fillRect(88, 97, 2, 4, TFT_LIGHTGRAY);
  }
  img->pushSprite(x, y, TFT_TRANSPARENT);
  tp->updated(false);
}

void tpmsViewRightDown(LGFX_Sprite *img, int x, int y, BLEtpms *tp) {
  img->fillSprite(TFT_TRANSPARENT);
  FONT_SANS_IMG;
  img->setTextColor(TFT_SILVER);
  int inc = ((TPRESS_MAX - TPRESS_MIN) / 10);
  for (int i = TPRESS_MIN; i < TPRESS_MAX; i += inc) {
    if (tp->pressure() >= i && tp->pressure() < i + inc) {
      img->fillArc(0, 0, 113, 118, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TPRESS_COLOR(i));
      img->fillArc(0, 0, 109, 113, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TPRESS_COLOR1(i));
      img->fillArc(0, 0, 105, 109, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TPRESS_COLOR2(i));
      img->fillArc(0, 0, 102, 105, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TPRESS_COLOR3(i));
      img->drawArc(0, 0, 101, 119, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_RED);
      img->fillArc(0, 0, 100, 102, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TPRESS_COLOR4(i));

      img->fillArc(0, 0, 70, 101, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(0, 0, 75, 101, TPRESS_DEG_R_DOWN(i) + 1,
                   (TPRESS_DEG_R_DOWN(i + inc) - 3) % 360, TFT_GRAY96);
      img->fillArc(0, 0, 75, 110, TPRESS_DEG_R_DOWN(i) + 2,
                   (TPRESS_DEG_R_DOWN(i + inc) - 3) % 360, TFT_WHITE);
    } else {
      img->fillArc(0, 0, 113, 118, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(0, 0, 109, 113, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY48);
      img->fillArc(0, 0, 105, 109, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY32);
      img->fillArc(0, 0, 102, 105, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY16);
      img->drawArc(0, 0, 101, 119, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_GRAY64);
      img->fillArc(0, 0, 100, 102, TPRESS_DEG_R_DOWN(i),
                   (TPRESS_DEG_R_DOWN(i + inc) - 2) % 360, TFT_BLACK);
    }
  }
  if (tp->pressure() > 0.0) {
    img->fillRoundRect(90, 7, 20, 12, 3,
                       tp->battery() > 60
                           ? TFT_DARKGREEN
                           : (tp->battery() > 50 ? TFT_GOLD : TFT_RED));
    img->drawRoundRect(90, 7, 20, 12, 3, TFT_LIGHTGRAY);
    img->drawRoundRect(89, 8, 18, 10, 3, TFT_LIGHTGRAY);
    img->fillRect(88, 11, 2, 4, TFT_LIGHTGRAY);
  }
  img->pushSprite(x, y, TFT_TRANSPARENT);
  tp->updated(false);
}

void temperatureView(LGFX_Device *img, int x, int y, float temp) {
  int count = 0, area = 0;
  img->startWrite();
  // img120x120.clearDisplay();
  int inc = ((TEMP_MAX - TEMP_MIN) / 30);
  for (int i = TEMP_MIN; i < TEMP_MAX; i += inc) {
    area = count < 5 ? 0 : (count < 15 ? 1 : (count < 25 ? 2 : 3));
    if (count == 0 || count == 5 || count == 15 || count == 25) {
      img120x120.fillSprite(TFT_TRANSPARENT);
    }
    if (temp * 10 < i + inc && temp * 10 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_RED);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR4(i));
    } else if (temp * 10 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TEMP_COLOR4(i));

    } else {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY48);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY32);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY16);
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, TEMP_DEG(i), (TEMP_DEG(i + inc) - 2) % 360,
                         TFT_BLACK);
    }
    img120x120.pushSprite(area < 2 ? 0 : 120, (area > 0 && area < 3) ? 0 : 120,
                          TFT_TRANSPARENT);
    count++;
  }
  drawBmp(img, (unsigned char *)icons[0], 119 - 32, 239 - 64, 64, 64);
  img->endWrite();
}

void humidityView(LGFX_Device *img, int x, int y, float humid) {
  int count = 0, area = 0;
  img->startWrite();
  int inc = ((HUMID_MAX - HUMID_MIN) / 30);
  for (int i = HUMID_MIN; i < HUMID_MAX; i += inc) {
    area = count < 5 ? 0 : (count < 15 ? 1 : (count < 25 ? 2 : 3));
    if (count == 0 || count == 5 || count == 15 || count == 25) {
      img120x120.fillSprite(TFT_TRANSPARENT);
    }
    if (humid * 10 < i + inc && humid * 10 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_RED);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR4(i));
    } else if (humid * 10 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         HUMID_COLOR4(i));
    } else {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY48);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY32);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY16);
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, HUMID_DEG(i), (HUMID_DEG(i + inc) - 2) % 360,
                         TFT_BLACK);
    }
    img120x120.pushSprite(area < 2 ? 0 : 120, (area > 0 && area < 3) ? 0 : 120,
                          TFT_TRANSPARENT);
    count++;
  }
  drawBmp(img, (unsigned char *)icons[1], 119 - 32, 239 - 64, 64, 64);
  img->endWrite();
}

void pressureView(LGFX_Device *img, int x, int y, float pressure) {
  int count = 0, area = 0;
  img->startWrite();
  int inc = ((PRESS_MAX - PRESS_MIN) / 30);
  for (int i = PRESS_MIN; i < PRESS_MAX; i += inc) {
    area = count < 5 ? 0 : (count < 15 ? 1 : (count < 25 ? 2 : 3));
    if (count == 0 || count == 5 || count == 15 || count == 25) {
      img120x120.fillSprite(TFT_TRANSPARENT);
    }
    if (pressure < i + inc && pressure >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_RED);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR4(i));
    } else if (pressure >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         PRESS_COLOR4(i));
    } else {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY48);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY32);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY16);
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, PRESS_DEG(i), (PRESS_DEG(i + inc) - 2) % 360,
                         TFT_BLACK);
    }
    img120x120.pushSprite(area < 2 ? 0 : 120, (area > 0 && area < 3) ? 0 : 120,
                          TFT_TRANSPARENT);
    count++;
  }
  drawBmp(img, (unsigned char *)icons[2], 119 - 32, 239 - 64, 64, 64);
  img->endWrite();
}

void co2View(LGFX_Device *img, int x, int y, float co2) {
  int count = 0, area = 0;
  img->startWrite();
  int inc = ((CO2_MAX - CO2_MIN) / 30);
  for (int i = CO2_MIN; i < CO2_MAX; i += inc) {
    area = count < 5 ? 0 : (count < 15 ? 1 : (count < 25 ? 2 : 3));
    if (count == 0 || count == 5 || count == 15 || count == 25) {
      img120x120.fillSprite(TFT_TRANSPARENT);
    }
    if (co2 < i + inc && co2 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_RED);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR4(i));
    } else if (co2 >= i) {
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR1(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR2(i));
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR3(i));
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         CO2_COLOR4(i));
    } else {  // co2 < i
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         113, 118, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         109, 113, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY48);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         105, 109, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY32);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         102, 105, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY16);
      img120x120.drawArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         101, 119, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_GRAY64);
      img120x120.fillArc(area < 2 ? 119 : 0, (area > 0 && area < 3) ? 119 : 0,
                         100, 102, CO2_DEG(i), (CO2_DEG(i + inc) - 2) % 360,
                         TFT_BLACK);
    }
    img120x120.pushSprite(img, area < 2 ? 0 : 120,
                          (area > 0 && area < 3) ? 0 : 120, TFT_TRANSPARENT);
    count++;
  }
  drawBmp(img, (unsigned char *)icons[3], 119 - 32, 239 - 64, 64, 64);
  img->endWrite();
}

void temperatureValueBox(LGFX_Sprite *img, int x, int y, float temp) {
  img->fillSprite(TFT_BLACK);
  //  img->setTextColor(TEMP_COLOR(temp * 10));
  img->setTextColor(TFT_WHITE);
  String str = format_digit(temp, 3, 1);
  String sint = str.substring(0, str.length() - 1);
  String sdecimal = str.substring(str.length() - 1);
  FONT_SANS24_IMG;
  int intWidth = img->textWidth(sint);
  int intHeight = img->fontHeight();
  FONT_SANS18_IMG;
  int decimalWidth = img->textWidth(sdecimal);
  int decimalHeight = img->fontHeight();
  FONT_SANS24_IMG;
  img->drawString(sint, img->width() / 2 - ((intWidth + decimalWidth) / 2), 0);
  FONT_SANS18_IMG;
  img->drawString(sdecimal,
                  img->width() / 2 - ((intWidth + decimalWidth) / 2) + intWidth,
                  intHeight - decimalHeight - 2);

  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void humidityValueBox(LGFX_Sprite *img, int x, int y, float humid) {
  img->fillSprite(TFT_BLACK);
  //  img->setTextColor(HUMID_COLOR(humid * 10));
  img->setTextColor(TFT_WHITE);
  String str = format_digit(humid, 3, 1);
  String sint = str.substring(0, str.length() - 1);
  String sdecimal = str.substring(str.length() - 1);
  FONT_SANS24_IMG;
  int intWidth = img->textWidth(sint);
  int intHeight = img->fontHeight();
  FONT_SANS18_IMG;
  int decimalWidth = img->textWidth(sdecimal);
  int decimalHeight = img->fontHeight();
  FONT_SANS24_IMG;
  img->drawString(sint, img->width() / 2 - ((intWidth + decimalWidth) / 2), 0);
  FONT_SANS18_IMG;
  img->drawString(sdecimal,
                  img->width() / 2 - ((intWidth + decimalWidth) / 2) + intWidth,
                  intHeight - decimalHeight - 2);
  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void pressureValueBox(LGFX_Sprite *img, int x, int y, float press) {
  img->fillSprite(TFT_BLACK);
  FONT_SANS24_IMG;
  //  img->setTextColor(PRESS_COLOR(press));
  img->setTextColor(TFT_WHITE);
  String str = format_digit(press, 4, 0);
  img->drawString(str, img->width() / 2 - img->textWidth(str) / 2, 0);
  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void co2ValueBox(LGFX_Sprite *img, int x, int y, float co2) {
  img->fillSprite(TFT_BLACK);
  FONT_SANS24_IMG;
  //  img->setTextColor(CO2_COLOR(co2));
  img->setTextColor(TFT_WHITE);
  String str = format_digit(co2, 4);
  img->drawString(str, img->width() / 2 - img->textWidth(str) / 2, 0);

  img->pushSprite(&tft0, x, y, TFT_TRANSPARENT);
}

void temperatureGraphBox(LGFX_Sprite *img, int x, int y) {
  int width = 160;
  int height = 72;
  img->fillSprite(TFT_BLACK);

  // find min max
  float max = -999, min = 1000, mid;
  for (int i = 0; i < SENSOR_HIST; i++) {
    if (temperature_hist[i] != -1000 && temperature_hist[i] > max) {
      max = temperature_hist[i];
    }
    if (temperature_hist[i] != -1000 && temperature_hist[i] < min) {
      min = temperature_hist[i];
    }
  }
  int imax = (max + 1) / 2 * 2;
  int imin = (min - 1) / 2 * 2;
  int imid = (imax + imin) / 2;
  int pos = 0;
  for (int i = 0; i < SENSOR_HIST; i++) {
    long t = temperature_hist[(temperature_hist_p + i) % SENSOR_HIST] * 10;
    if (t != -1000 * 10) {
      img->drawGradientVLine(
          15 + pos, (height - map(t, imin * 10, imax * 10, 0, height)),
          map(t, imin * 10, imax * 10, 0, height), TEMP_COLOR(t), TFT_BLACK);
      pos++;
    }
  }

  FONT_SANS_IMG;
  img->setTextColor(TFT_LIGHTGRAY);
  String str = String(imax);
  img->drawString(String(imax), img->width() - img->textWidth(str) - 5, 0);
  str = String(imid);
  img->drawString(String(imid), img->width() - img->textWidth(str) - 5,
                  img->height() / 2 - img->fontHeight() / 2);
  str = String(imin);
  img->drawString(String(imin), img->width() - img->textWidth(str) - 5,
                  img->height() - img->fontHeight());

  img->drawLine(10, 0, width - 50, 0, TFT_GRAY64);
  img->drawLine(10, height - 1, width - 50, height - 1, TFT_GRAY64);
  img->drawLine(10, height / 2, width - 50, height / 2, TFT_GRAY64);

  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void humidityGraphBox(LGFX_Sprite *img, int x, int y) {
  int width = 160;
  int height = 72;
  img->fillSprite(TFT_BLACK);

  // find min max
  float max = 0, min = 100, mid;
  for (int i = 0; i < SENSOR_HIST; i++) {
    if (humidity_hist[i] != 0 && humidity_hist[i] > max) {
      max = humidity_hist[i];
    }
    if (humidity_hist[i] != 0 && humidity_hist[i] < min) {
      min = humidity_hist[i];
    }
  }
  int imax = (max + 1) / 2 * 2;
  int imin = (min - 1) / 2 * 2;
  int imid = (imax + imin) / 2;
  int pos = 0;
  for (int i = 0; i < SENSOR_HIST; i++) {
    long t = humidity_hist[(humidity_hist_p + i) % SENSOR_HIST] * 10;
    if (t != 0) {
      img->drawGradientVLine(
          15 + pos, height - map(t, imin * 10, imax * 10, 0, height),
          map(t, imin * 10, imax * 10, 0, height), HUMID_COLOR(t), TFT_BLACK);
      pos++;
    }
  }

  FONT_SANS_IMG;
  img->setTextColor(TFT_LIGHTGRAY);
  String str = String(imax);
  img->drawString(String(imax), img->width() - img->textWidth(str) - 5, 0);
  str = String(imid);
  img->drawString(String(imid), img->width() - img->textWidth(str) - 5,
                  img->height() / 2 - img->fontHeight() / 2);
  str = String(imin);
  img->drawString(String(imin), img->width() - img->textWidth(str) - 5,
                  img->height() - img->fontHeight());

  img->drawLine(10, 0, width - 50, 0, TFT_GRAY64);
  img->drawLine(10, height - 1, width - 50, height - 1, TFT_GRAY64);
  img->drawLine(10, height / 2, width - 50, height / 2, TFT_GRAY64);

  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void pressureGraphBox(LGFX_Sprite *img, int x, int y) {
  int width = 160;
  int height = 72;
  img->fillSprite(TFT_BLACK);

  // find min max
  float max = -10000, min = 10000, mid;
  for (int i = 0; i < SENSOR_HIST; i++) {
    if (pressure_hist[i] != 0 && pressure_hist[i] > max) {
      max = pressure_hist[i];
    }
    if (pressure_hist[i] != 0 && pressure_hist[i] < min) {
      min = pressure_hist[i];
    }
  }
  int imax = ((int)max + 2);
  int imin = ((int)min - 2);
  int imid = (imax + imin) / 2;
  int pos = 0;
  for (int i = 0; i < SENSOR_HIST; i++) {
    long t = (long)pressure_hist[(pressure_hist_p + i) % SENSOR_HIST];
    if (t != 0) {
      img->drawGradientVLine(15 + pos, height - map(t, imin, imax, 0, height),
                             map(t, imin, imax, 0, height), PRESS_COLOR(t),
                             TFT_BLACK);
      pos++;
    }
  }

  FONT_SANS_IMG;
  img->setTextColor(TFT_LIGHTGRAY);
  String str = String(imax);
  img->drawString(String(imax), img->width() - img->textWidth(str) - 5, 0);
  str = String(imid);
  img->drawString(String(imid), img->width() - img->textWidth(str) - 5,
                  img->height() / 2 - img->fontHeight() / 2);
  str = String(imin);
  img->drawString(String(imin), img->width() - img->textWidth(str) - 5,
                  img->height() - img->fontHeight());

  img->drawLine(10, 0, width - 50, 0, TFT_GRAY64);
  img->drawLine(10, height - 1, width - 50, height - 1, TFT_GRAY64);
  img->drawLine(10, height / 2, width - 50, height / 2, TFT_GRAY64);

  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void co2GraphBox(LGFX_Sprite *img, int x, int y) {
  int width = 160;
  int height = 72;
  img->fillSprite(TFT_BLACK);

  // find min max
  float max = -10000, min = 10000, mid;
  for (int i = 0; i < SENSOR_HIST; i++) {
    if (co2_hist[i] != 0 && co2_hist[i] > max) {
      max = co2_hist[i];
    }
    if (co2_hist[i] != 0 && co2_hist[i] < min) {
      min = co2_hist[i];
    }
  }
  int imax = ((int)max + 9) / 10 * 10;
  int imin = ((int)min - 9) / 10 * 10;
  int imid = (imax + imin) / 2;
  int pos = 0;
  for (int i = 0; i < SENSOR_HIST; i++) {
    long t = (long)co2_hist[(co2_hist_p + i) % SENSOR_HIST];
    if (t != 0) {
      img->drawGradientVLine(15 + pos, height - map(t, imin, imax, 0, height),
                             map(t, imin, imax, 0, height), CO2_COLOR(t),
                             TFT_BLACK);
      pos++;
    }
  }

  FONT_SANS_IMG;
  img->setTextColor(TFT_LIGHTGRAY);
  String str = String(imax);
  img->drawString(String(imax), img->width() - img->textWidth(str) - 5, 0);
  str = String(imid);
  img->drawString(String(imid), img->width() - img->textWidth(str) - 5,
                  img->height() / 2 - img->fontHeight() / 2);
  str = String(imin);
  img->drawString(String(imin), img->width() - img->textWidth(str) - 5,
                  img->height() - img->fontHeight());

  img->drawLine(10, 0, width - 50, 0, TFT_GRAY64);
  img->drawLine(10, height - 1, width - 50, height - 1, TFT_GRAY64);
  img->drawLine(10, height / 2, width - 50, height / 2, TFT_GRAY64);

  img->pushSprite(&tft0, x, y, TFT_TRANSPARENT);
}

void messageBox(LGFX_Sprite *img, int x, int y, String message) {
  img->fillSprite(TFT_TRANSPARENT);
  img->fillRoundRect(0, 0, 160, 72, 5, TFT_NAVY);
  img->drawRoundRect(0, 0, 160, 72, 5, TFT_WHITE);
  img->drawRoundRect(1, 1, 158, 70, 5, TFT_WHITE);
  FONT_SANS_IMG;
  img->setTextColor(TFT_WHITE);
  img->drawString(message, 5, 25);
  img->pushSprite(x, y, TFT_TRANSPARENT);
}

void drawView0(int x, int y, int v, float co2) {
  tft0.startWrite();
  co2View(&tft0, x, y, co2);
  co2ValueBox(&img140x60, x + 50, y + 46, co2);
  co2GraphBox(&img160x72, x + 40, y + 100);
  tft0.endWrite();
}

void drawView1(int x, int y, int v, float temp, float humid, float press) {
  tft1.startWrite();
  if (v == 0) {
    temperatureView(&tft1, x, y, temp);
    temperatureValueBox(&img140x60, x + 50, y + 46, temp);
    temperatureGraphBox(&img160x72, x + 40, y + 100);
  } else if (v == 1) {
    humidityView(&tft1, x, y, humid);
    humidityValueBox(&img140x60, x + 50, y + 46, humid);
    humidityGraphBox(&img160x72, x + 40, y + 100);
  } else if (v == 2) {
    pressureView(&tft1, x, y, press);
    pressureValueBox(&img140x60, x + 50, y + 46, press);
    pressureGraphBox(&img160x72, x + 40, y + 100);
  } else if (v == 3) {
    Serial.printf("tpms 0 p: %d pp: %d \n", int(tpms[0].pressure() / 10),
                  int(prev_tpress[0] / 10));
    Serial.printf("tpms 1 p: %d pp: %d \n", int(tpms[1].pressure() / 10),
                  int(prev_tpress[1] / 10));
    Serial.printf("tpms 2 p: %d pp: %d \n", int(tpms[2].pressure() / 10),
                  int(prev_tpress[2] / 10));
    Serial.printf("tpms 3 p: %d pp: %d \n", int(tpms[3].pressure() / 10),
                  int(prev_tpress[3] / 10));
    if (p_view != view ||
        int(tpms[0].pressure() / 10) != int(prev_tpress[0] / 10) ||
        int(tpms[1].pressure() / 10) != int(prev_tpress[1] / 10) ||
        int(tpms[2].pressure() / 10) != int(prev_tpress[2] / 10) ||
        int(tpms[3].pressure() / 10) != int(prev_tpress[3] / 10)) {
      tft1.fillScreen(TFT_BLACK);
      drawBmp(&tft1, (unsigned char *)icons[4], 120 - 32, 120 - 32, 64, 64);
      tpmsViewLeftUp(&img120x120, x, y, &(tpms[0]));
      tpmsValueBox(&img140x60, x + 5, y + 60, &(tpms[0]));
      tpmsViewLeftDown(&img120x120, x, y + 120, &(tpms[2]));
      tpmsValueBox(&img140x60, x + 5, y + 120 + 15, &(tpms[2]));
      tpmsViewRightUp(&img120x120, x + 120, y, &(tpms[1]));
      tpmsValueBox(&img140x60, x + 120 - 35, y + 60, &(tpms[1]));
      tpmsViewRightDown(&img120x120, x + 120, y + 120, &(tpms[3]));
      tpmsValueBox(&img140x60, x + 120 - 35, y + 120 + 15, &(tpms[3]));
      prev_tpress[0] = tpms[0].pressure();
      prev_tpress[1] = tpms[1].pressure();
      prev_tpress[2] = tpms[2].pressure();
      prev_tpress[3] = tpms[3].pressure();
      p_view = view;
    }
  }
  tft1.endWrite();
}

void drawScreen(float temp, float humid, float press, float co2) {
  drawView0(0, 0, view, co2);
  drawView1(0, 0, view, temp, humid, press);
}

void changeFace(int face) {
  if (face_shape != face) {
    face_shape = face;
    if (face == FACE_GOOD) {
      eyes.shape(SHAPE_LT, SHAPE_GT);
    } else if (face == FACE_DIRTY) {
      eyes.shape(SHAPE_ZITO);
    } else if (face == FACE_NODATA) {
      eyes.shape(SHAPE_HORI);
    } else if (face == FACE_GURUGURU) {
      eyes.shape(SHAPE_GURUGURU);
    } else {
      eyes.shape(SHAPE_NORMAL);
      face_shape = FACE_NORMAL;
    }
  }
}

void setup() {
  tpms[0].tire_id(TIRE_FL);
  tpms[1].tire_id(TIRE_FR);
  tpms[2].tire_id(TIRE_RL);
  tpms[3].tire_id(TIRE_RR);

  /* Dummy data */
  tpms[0].temp_raw(-10000);
  tpms[0].pressure_raw(0);
  tpms[0].battery_raw(95);

  tpms[1].temp_raw(-10000);
  tpms[1].pressure_raw(0);
  tpms[1].battery_raw(60);

  tpms[2].temp_raw(-10000);
  tpms[2].pressure_raw(0);
  tpms[2].battery_raw(75);

  tpms[3].temp_raw(-10000);
  tpms[3].pressure_raw(0);
  tpms[3].battery_raw(90);
  // */

//  WiFi.mode(WIFI_OFF);

  pinMode(PIN_TOUCH, INPUT);
  Serial.begin(115200);

  Wire.begin();
  Wire1.begin(PIN_SDA2, PIN_SCL2);

  if (!sensorPressure.begin(BMP280_ADDRESS_ALT)) {
    DPRINTLN("BMP280 is not found");
  } else {
    DPRINTLN("BMP280 is found");
  }

  if (!sensorCO2.begin()) {
    DPRINTLN("SCD40 is not found");
  } else {
    DPRINTLN("SCD40 is found");
  }

  //  sensorTemp.begin();

  if (accel.begin() == false) {
    DPRINTLN("No accelaration sensor detected.");
    use_accel = 0;
  } else {
    use_accel = 1;
    float y_accel = 0.0, z_accel = 0.0;
    for (int i = 0; i < 50; i++) {
      y_accel += accel.getCalculatedY();
      z_accel += accel.getCalculatedZ();
      delay(20);
    }
    y_accel_center = y_accel / 50.0;
    z_accel_center = z_accel / 50.0;
  }
  DPRINT("Y:");
  DPRINTLN(y_accel_center);
  DPRINT("Z:");
  DPRINTLN(z_accel_center);
  prefs.begin("aquatan", false);
  view = prefs.getUShort("view", 0);

  for (int i = 0; i < SENSOR_HIST; i++) {
    temperature_hist[i] = -1000;
    //    co2_hist[i] = 400 + i * 10;
    co2_hist[i] = 0;
    pressure_hist[i] = 0;
    humidity_hist[i] = 0;
  }

  tft0.init();
  delay(10);
  tft1.init();
  delay(10);
  tft0.setBrightness(255);
  tft0.setRotation(3);
  tft1.setRotation(1);

  img120x120.setColorDepth(16);
  img120x120.createSprite(120, 120);
  img140x60.setColorDepth(8);
  img140x60.createSprite(140, 60);
  img160x72.setColorDepth(16);
  img160x72.createSprite(160, 72);

  oled1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled2.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled1.setRotation(0);
  oled1.clearDisplay();
  oled1.setTextColor(SSD1306_WHITE);
  oled2.setRotation(2);
  oled2.clearDisplay();
  oled2.setTextColor(SSD1306_WHITE);
  //  oled.print("Hello, world");
  oled1.display();
  oled2.display();

  eyes.begin(0, 0, 0, 0);
  eyes.shape(SHAPE_NORMAL);
  eyes.mode(EYE_IDLE);

  drawView0(0, 0, 0, 0);
  drawView1(0, 0, 0, 0, 0, 0);

  DPRINTLN("LCD Initialized");

  String msg = "Initializing";
  messageBox(&img160x72, 40, 100, msg);

  // a4:cf:12:6e:3a:f2 -> fakeTPMS
  NimBLEAddress tpms_fake_addr("A4:CF:12:6E:3A:F2", 0);
  NimBLEAddress tpms_cache_addr("A4:CF:12:6E:12:2E", 0);
   
  NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DATA);
  NimBLEDevice::setScanDuplicateCacheSize(20);
  NimBLEDevice::init("");
  NimBLEDevice::whiteListAdd(tpms_fake_addr);
  NimBLEDevice::whiteListAdd(tpms_cache_addr);
//  for (int i = 0; i < 4; i++) {
//    NimBLEAddress tpms_addr(tpms[i].macaddress().c_str(), 0);
//    NimBLEDevice::whiteListAdd(tpms_addr);
//  }
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(),
                                         false);
  pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL);
  pBLEScan->setActiveScan(false);  // Set active scanning, this will get more
                                  // data from the advertiser.
  pBLEScan->setInterval(
      100);  // How often the scan occurs / switches channels; in milliseconds,
  pBLEScan->setWindow(
      99);  // How long to scan during the interval; in milliseconds.
  pBLEScan->setMaxResults(
      0);  // do not store the scan results, use callback only.

  attachInterrupt(PIN_TOUCH, handleTouch, RISING);
}

void loop() {
  bool valid_data = false;
  static float temp, humid, press, co2, temp0, temp1, temp2;
  static uint32_t prev_millis = 0;
  static uint32_t face_millis = 0;
  static int prev_move_z = 0, prev_move_y = 0;

  if (pBLEScan->isScanning() == false) {
    DPRINTLN("BLE scannning...");
    pBLEScan->start(0, nullptr, false);
  }

  if (millis() - prev_millis > 5000 && !eyes.isBlinking()) {
    prev_millis = millis();
    //    temp0 = sensorTempHumid.readTemperature();
    //    delay(100);
    humid = sensorTempHumid.readHumidity();
    delay(100);
    press = (sensorPressure.readPressure() / 100.0);
    temp2 = sensorPressure.readTemperature();
    delay(100);
    co2 = sensorCO2.getCO2();
    temp1 = sensorCO2.getTemperature();
    delay(100);
    if (seq > 3) {
      stable = 1;
    } else if (!stable) {
      stable = 0;
    }
    seq++;
    //    Serial.printf(">>> seq: %d, t0: %.1f, t1: %.1f, t2: %.1f, h: %.1f, p:
    //    %.1f c: %.1f\r\n", seq, temp0, temp1, temp2, humid, press, co2);
    temp = temp1;

    if (humid == 0 || press < 850 || co2 == 0 || co2 > 5000) {
      valid_data = false;
    } else {
      valid_data = true;
    }
    if (valid_data && stable) {
      temperature_hist[temperature_hist_p++] = temp;
      humidity_hist[humidity_hist_p++] = humid;
      pressure_hist[pressure_hist_p++] = press;
      co2_hist[co2_hist_p++] = co2;
      temperature_hist_p %= SENSOR_HIST;
      humidity_hist_p %= SENSOR_HIST;
      pressure_hist_p %= SENSOR_HIST;
      co2_hist_p %= SENSOR_HIST;
    }
  }

  if (btnint) {
    DPRINTLN("touched");
    btnint = 0;
    changeFace(FACE_GOOD);
    eyes.mode(EYE_BLINK);
    face_millis = millis();
    if (stable) {
      view++;
      view %= NUM_OF_VIEWS;
      prefs.putUShort("view", view);
      tft1.fillScreen(TFT_BLACK);
      drawScreen(temp, humid, press, co2);
      p_view = view;
    }
  }

  if (!stable) {
    // drawScreen(0, 0, 0, 0);
    String msg = "Waiting data.";
    for (int i = 0; i < (seq > 3 ? 3 : seq); i++) {
      msg += ".";
    }

    messageBox(&img160x72, 40, 100, msg);
    //      eyes.color(LEDCOLOR_MAGENTA);
    changeFace(FACE_NODATA);
  } else if (valid_data) {
    drawScreen(temp, humid, press, co2);
    // if (temp < 30 && temp > 20 && co2 < 800 && humid > 30) {
    //         eyes.color(LEDCOLOR_GREEN);
    //  changeFace(FACE_GOOD);
    //} else
    // if (co2 > 1200) {
    //        eyes.color(LEDCOLOR_RED);
    // changeFace(FACE_DIRTY);
    //} else
    if (press < 990 || temp > 35 || co2 > 1200) {
      //        eyes.color(LEDCOLOR_BLUE);
      changeFace(FACE_GURUGURU);
    } else {
      //        eyes.color(LEDCOLOR_CYAN);
      changeFace(FACE_NORMAL);
    }
  }

  if (use_accel && accel.available()) {  // Wait for new data from accelerometer
    // Acceleration of x, y, and z directions in g units

    /*
        DPRINT(accel.getCalculatedX(), 3);
        DPRINT("\t");
        DPRINT(accel.getCalculatedY(), 3); // left -> nega right -> posi
        DPRINT("\t");
        DPRINT(accel.getCalculatedZ(), 3); // up -> nega down -> posi
        DPRINTLN();
    */

    float y_accel = accel.getCalculatedY();
    int move_y = y_accel < y_accel_center - ACCEL_RANGE
                     ? -1
                     : (y_accel > y_accel_center + ACCEL_RANGE ? 1 : 0);
    float z_accel = accel.getCalculatedZ();
    int move_z = z_accel < z_accel_center - ACCEL_RANGE
                     ? -1
                     : (z_accel > z_accel_center + ACCEL_RANGE ? 1 : 0);
    if (move_y != prev_move_y || move_z != prev_move_z) {
      eyes.mode(EYE_MOVE);
      eyes.setMoveTarget(move_z, move_y);
    }
    prev_move_y = move_y;
    prev_move_z = move_z;
  }

  if (face_millis > 0 && millis() - face_millis > 1500) {
    changeFace(face_shape);
    face_millis = 0;
  }

  eyes.randomBlinking(0);
  eyes.move();
  eyes.blink();
}
