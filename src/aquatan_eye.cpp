#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

#include "aquatan_eye.h"

static const unsigned char eye_bmp[6][BLINK_FRAME][96] = {
    {{B00000000, B00000000, B11111111, B11111111, B00000000, B00000000,
      B00000000, B00111111, B11111111, B11111111, B11111100, B00000000,
      B00000001, B11111111, B11111111, B11111111, B11111111, B10000000,
      B00001111, B11111111, B11111111, B11111111, B11111111, B11110000,
      B00111111, B11111111, B11111111, B11111111, B11111111, B11111100,
      B01111111, B11111111, B11111111, B11111111, B11111111, B11111110,
      B11111111, B11111111, B11111111, B11111111, B11111111, B11111111,
      B11111111, B11111111, B11111111, B11111111, B11111111, B11111111,
      B11111111, B11111111, B11111111, B11111111, B11111111, B11111111,
      B11111111, B11111111, B11111111, B11111111, B11111111, B11111111,
      B01111111, B11111111, B11111111, B11111111, B11111111, B11111110,
      B00111111, B11111111, B11111111, B11111111, B11111111, B11111100,
      B00001111, B11111111, B11111111, B11111111, B11111111, B11110000,
      B00000001, B11111111, B11111111, B11111111, B11111111, B10000000,
      B00000000, B00111111, B11111111, B11111111, B11111100, B00000000,
      B00000000, B00000000, B11111111, B11111111, B00000000, B00000000},
     {B00000000, B00000000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00000001, B11111111, B11111111, B10000000, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11100000, B00000000,
      B00000000, B00011111, B11111111, B11111111, B11111000, B00000000,
      B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B11111111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B11111111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B11111111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B11111111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B01111111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B00011111, B11111111, B11111111, B11111000, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11100000, B00000000,
      B00000000, B00000001, B11111111, B11111111, B10000000, B00000000,
      B00000000, B00000000, B00011111, B11111000, B00000000, B00000000},
     {B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}},
    {{B00000111, B00000000, B11111111, B11111111, B00000000, B00000000,
      B00000011, B10011111, B11111111, B11111111, B11111100, B00000000,
      B00000011, B11111111, B11111111, B11111111, B11111111, B10000000,
      B00000011, B11111111, B11111111, B11111111, B11111111, B11110000,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111100,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111110,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111111,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111111,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111111,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111111,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111110,
      B00000001, B11111111, B11111111, B11111111, B11111111, B11111100,
      B00000011, B11111111, B11111111, B11111111, B11111111, B11110000,
      B00000011, B11111111, B11111111, B11111111, B11111111, B10000000,
      B00000011, B10011111, B11111111, B11111111, B11111100, B00000000,
      B00000111, B00000000, B11111111, B11111111, B00000000, B00000000},
     {B00000000, B00011000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00001101, B11111111, B11111111, B10000000, B00000000,
      B00000000, B00001111, B11111111, B11111111, B11100000, B00000000,
      B00000000, B00001111, B11111111, B11111111, B11111000, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111111, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B00000111, B11111111, B11111111, B11111110, B00000000,
      B00000000, B00001111, B11111111, B11111111, B11111000, B00000000,
      B00000000, B00001111, B11111111, B11111111, B11100000, B00000000,
      B00000000, B00001101, B11111111, B11111111, B10000000, B00000000,
      B00000000, B00011000, B00011111, B11111000, B00000000, B00000000},
     {B00000000, B00000000, B00011001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00001011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00011001, B10000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}},
    {{B00000111, B11000000, B00000000, B00000000, B00000011, B11100000,
      B00000011, B11100000, B00000000, B00000000, B00000111, B11000000,
      B00000001, B11110000, B00000000, B00000000, B00001111, B10000000,
      B00000000, B11111000, B00000000, B00000000, B00011111, B00000000,
      B00000000, B01111100, B00000000, B00000000, B00111110, B00000000,
      B00000000, B00111110, B00000000, B00000000, B01111100, B00000000,
      B00000000, B00011111, B00000000, B00000000, B11111000, B00000000,
      B00000000, B00001111, B10000000, B00000001, B11110000, B00000000,
      B00000000, B00000111, B11000000, B00000011, B11100000, B00000000,
      B00000000, B00000011, B11100000, B00000111, B11000000, B00000000,
      B00000000, B00000001, B11110000, B00001111, B10000000, B00000000,
      B00000000, B00000000, B11111000, B00011111, B00000000, B00000000,
      B00000000, B00000000, B01111100, B00111110, B00000000, B00000000,
      B00000000, B00000000, B00111110, B01111100, B00000000, B00000000,
      B00000000, B00000000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000},
     {B00000000, B00111100, B00000000, B00000000, B00111100, B00000000,
      B00000000, B00011110, B00000000, B00000000, B01111000, B00000000,
      B00000000, B00011110, B00000000, B00000000, B01111000, B00000000,
      B00000000, B00001111, B00000000, B00000000, B11110000, B00000000,
      B00000000, B00000111, B10000000, B00000001, B11100000, B00000000,
      B00000000, B00000111, B10000000, B00000001, B11100000, B00000000,
      B00000000, B00000011, B11000000, B00000011, B11000000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000},
     {B00000000, B00000000, B11100000, B00000111, B00000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000000, B01110000, B00001110, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B00111000, B00011100, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B00011100, B00111000, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00001110, B01110000, B00000000, B00000000,
      B00000000, B00000000, B00001110, B01110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}},
    {{B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00000000, B00111110, B01111100, B00000000, B00000000,
      B00000000, B00000000, B01111100, B00111110, B00000000, B00000000,
      B00000000, B00000000, B11111000, B00011111, B00000000, B00000000,
      B00000000, B00000001, B11110000, B00001111, B10000000, B00000000,
      B00000000, B00000011, B11100000, B00000111, B11000000, B00000000,
      B00000000, B00000111, B11000000, B00000011, B11100000, B00000000,
      B00000000, B00001111, B10000000, B00000001, B11110000, B00000000,
      B00000000, B00011111, B00000000, B00000000, B11111000, B00000000,
      B00000000, B00111110, B00000000, B00000000, B01111100, B00000000,
      B00000000, B01111100, B00000000, B00000000, B00111110, B00000000,
      B00000000, B11111000, B00000000, B00000000, B00011111, B00000000,
      B00000001, B11110000, B00000000, B00000000, B00001111, B10000000,
      B00000011, B11100000, B00000000, B00000000, B00000111, B11000000,
      B00000111, B11000000, B00000000, B00000000, B00000011, B11100000},
     {B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000011, B11000000, B00000011, B11000000, B00000000,
      B00000000, B00000111, B10000000, B00000001, B11100000, B00000000,
      B00000000, B00000111, B10000000, B00000001, B11100000, B00000000,
      B00000000, B00001111, B00000000, B00000000, B11110000, B00000000,
      B00000000, B00011110, B00000000, B00000000, B01111000, B00000000,
      B00000000, B00011110, B00000000, B00000000, B01111000, B00000000,
      B00000000, B00111100, B00000000, B00000000, B00111100, B00000000},
     {B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001110, B01110000, B00000000, B00000000,
      B00000000, B00000000, B00001110, B01110000, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00011110, B01111000, B00000000, B00000000,
      B00000000, B00000000, B00011100, B00111000, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B00111100, B00111100, B00000000, B00000000,
      B00000000, B00000000, B00111000, B00011100, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B01111000, B00011110, B00000000, B00000000,
      B00000000, B00000000, B01110000, B00001110, B00000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000000, B11110000, B00001111, B00000000, B00000000,
      B00000000, B00000000, B11100000, B00000111, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}},
    {{B00000000, B00001111, B11111111, B11111111, B00000000, B00000000,
      B00000000, B01111111, B11111111, B11111111, B11111000, B00000000,
      B00000011, B11111110, B00000000, B00001111, B11111111, B00000000,
      B00000111, B11100000, B00000000, B00000000, B01111111, B11000000,
      B00011111, B00000000, B00011111, B11111000, B00000111, B11100000,
      B00011110, B00000000, B11111111, B11111111, B11100000, B11110000,
      B00111100, B00000111, B11110000, B00011111, B11111000, B01111000,
      B01111000, B00001111, B11000000, B00000000, B01111110, B00111100,
      B01111000, B00001110, B00000000, B00000110, B00011111, B00011100,
      B01111100, B00001110, B00000000, B00001110, B00000111, B00011110,
      B01111110, B00011111, B11111111, B11111110, B00000011, B00001110,
      B00111111, B00000011, B11111111, B11111100, B00000011, B00001110,
      B00001111, B11000000, B00000000, B00000000, B00000111, B00011100,
      B00000111, B11100000, B00000000, B00000000, B00001111, B00111000,
      B00000011, B11111111, B11111111, B11111111, B11111110, B00111000,
      B00000000, B11111111, B11111111, B11111111, B11111100, B00000000},
     {B00000000, B00000000, B00011111, B11111000, B00000000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000111, B00011111, B11111000, B11100000, B00000000,
      B00000000, B00011100, B11110000, B00001111, B00111000, B00000000,
      B00000000, B01110011, B00001111, B11110000, B11001110, B00000000,
      B00000000, B01101110, B11111000, B00011111, B01110110, B00000000,
      B00000000, B11101101, B11110111, B11101111, B10110111, B00000000,
      B00000000, B11101101, B11101111, B11110111, B10110111, B00000000,
      B00000000, B11101101, B11101111, B11110111, B10110111, B00000000,
      B00000000, B11101110, B11110111, B11101111, B01110111, B00000000,
      B00000000, B01110011, B01111000, B00011110, B11001110, B00000000,
      B00000000, B01111001, B10001111, B11110001, B10011110, B00000000,
      B00000000, B00011110, B11110000, B00001111, B01111000, B00000000,
      B00000000, B00000111, B00011111, B11111000, B11100000, B00000000,
      B00000000, B00000001, B11100000, B00000111, B10000000, B00000000,
      B00000000, B00000000, B00011111, B11111000, B00000000, B00000000},
     {B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}},
    {{B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000},
     {B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000}}//,
/*    {{B00000000, B00000000, B11111111, B11111111, B00000000, B00000000,
      B00000000, B00111111, B11111111, B11111100, B00111100, B00000000,
      B00000001, B11111111, B11111111, B11110000, B11111111, B10000000,
      B00001111, B11111111, B11111111, B11000011, B11111111, B11110000,
      B00111111, B11111111, B11111111, B00001111, B11111111, B11111100,
      B01111111, B11111111, B11111100, B00111111, B11111111, B11111110,
      B11111111, B11111111, B11110000, B11111111, B11111111, B11111111,
      B11111111, B11111111, B11000011, B11111111, B11111111, B11111111,
      B11111111, B11111111, B00001111, B11111111, B11111111, B11111111,
      B11111111, B11111100, B00111111, B11111111, B11111111, B11111111,
      B01111111, B11110000, B11111111, B11111111, B11111111, B11111110,
      B00111111, B11000011, B11111111, B11111111, B11111111, B11111100,
      B00001111, B00001111, B11111111, B11111111, B11111111, B11110000,
      B00000000, B11111111, B11111111, B11111111, B11111111, B10000000,
      B00000000, B00111111, B11111111, B11111111, B11111100, B00000000,
      B00000000, B00000000, B11111111, B11111111, B00000000, B00000000},
     {B00000000, B00000000, B00011111, B00111000, B00000000, B00000000,
      B00000000, B00000001, B11111111, B00111111, B10000000, B00000000,
      B00000000, B00000111, B11111111, B00111111, B11100000, B00000000,
      B00000000, B00011111, B11111111, B00111111, B11111000, B00000000,
      B00000000, B01111111, B11111110, B01111111, B11111110, B00000000,
      B00000000, B01111111, B11111110, B01111111, B11111110, B00000000,
      B00000000, B11111111, B11111110, B01111111, B11111111, B00000000,
      B00000000, B11111111, B11111110, B01111111, B11111111, B00000000,
      B00000000, B11111111, B11111100, B11111111, B11111111, B00000000,
      B00000000, B11111111, B11111100, B11111111, B11111111, B00000000,
      B00000000, B01111111, B11111100, B11111111, B11111110, B00000000,
      B00000000, B01111111, B11111100, B11111111, B11111110, B00000000,
      B00000000, B00011111, B11111001, B11111111, B11111000, B00000000,
      B00000000, B00000111, B11111001, B11111111, B11100000, B00000000,
      B00000000, B00000001, B11111001, B11111111, B10000000, B00000000,
      B00000000, B00000000, B00011001, B11111000, B00000000, B00000000},
     {B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00001111, B11110000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000111, B11100000, B00000000, B00000000,
      B00000000, B00000000, B00000011, B11000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000},
     {B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000,
      B00000000, B00000000, B00000001, B10000000, B00000000, B00000000}}*/
      };

AquatanEye::AquatanEye(Adafruit_SSD1306 *d) {
  disp = d;
  move_diff_x = target_dir_x = current_dir_x = 0;
  move_diff_y = target_dir_y = current_dir_y = 0;
}

void AquatanEye::begin() {
//  if (!disp->begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
//    Serial.println(F("SSD1306 allocation failed"));
//    for (;;)
//      ; // Don't proceed, loop forever
//  }
//  disp->clearDisplay();
  show(0, 0, 0);
}

void AquatanEye::begin(int w_adj, int h_adj) {
  width_adj = w_adj;
  height_adj = h_adj;
  begin();
}

int AquatanEye::mode() { return _mode; }

void AquatanEye::mode(int m) {
  if (m >= EYE_IDLE && m < EYE_LAST) {
    _mode = m;
    if (_mode == EYE_BLINK) {
      _blink = BLINK_CLOSE;
    }
  }
}

int AquatanEye::shape() { return _shape; }

void AquatanEye::shape(int s) {
  if (s >= SHAPE_NORMAL && s < SHAPE_LAST) {
    _shape = s;
    show(blink_frame, current_dir_x * 8, current_dir_y * 8);
  }
}

void AquatanEye::setMoveTarget(int x, int y) {
  target_dir_x = x;
  target_dir_y = y;
  move_diff_x = (target_dir_x - current_dir_x) * 8 / MOVE_FRAME;
  move_diff_y = (target_dir_y - current_dir_y) * 8 / MOVE_FRAME;
}

int AquatanEye::move() {
  if (mode() == EYE_MOVE) {
    move_frame++;
    show(0, current_dir_x * 8 + move_diff_x * move_frame,
         current_dir_y * 8 + move_diff_y * move_frame);
    if (move_frame >= MOVE_FRAME - 1) {
      mode(EYE_IDLE);
      move_frame = 0;
      current_dir_x = target_dir_x;
      current_dir_y = target_dir_y;
    }
  }
  return mode();
}

int AquatanEye::blink() {
  if (mode() == EYE_BLINK) {
    if (_blink == BLINK_CLOSE) {
      blink_frame++;
      if (blink_frame >= BLINK_FRAME - 1) {
        _blink = BLINK_OPEN;
      }
    } else if (_blink == BLINK_OPEN) {
      blink_frame--;
      if (blink_frame <= 0) {
        _blink = BLINK_NONE;
        mode(EYE_IDLE);
      }
    }
    show(blink_frame, current_dir_x * 8, current_dir_y * 8);
  }
  return mode();
}

int AquatanEye::blink_st() { return _blink; }

void AquatanEye::show(int frame, int diff_x, int diff_y) {
  // (128-48)/2 + 30 = 70,
  int center_w = (disp->width() - EYE_WIDTH) / 2  + width_adj;
  int center_h = (disp->height() - EYE_HEIGHT) / 2 + height_adj;

  disp->clearDisplay();
  disp->drawBitmap(center_w + diff_x, center_h + diff_y, eye_bmp[_shape][frame],
                   EYE_WIDTH, EYE_HEIGHT, WHITE);
  disp->display();
}

// AquatanEyes ---------------------------------------------------------------

AquatanEyes::AquatanEyes(AquatanEye *l, AquatanEye *r) {
  _left = l;
  _right = r;
}

void AquatanEyes::begin() {
  _left->begin();
  _right->begin();
}

void AquatanEyes::begin(int left_w_adj, int left_h_adj, int right_w_adj,
                        int right_h_adj) {
  _left->begin(left_w_adj, left_h_adj);
  _right->begin(right_w_adj, right_h_adj);
}

void AquatanEyes::move() {
  if ((millis() - _movetimer) > (1000 / fps)) {
    _movetimer = millis();
    _mode = (_left->move() | _right->move());
  }
}
void AquatanEyes::blink() {
  if ((millis() - _blinktimer) > (1000 / fps)) {
    _blinktimer = millis();
    _mode = (_left->blink() | _right->blink());
  }
}

void AquatanEyes::show(int frame, int diff_x, int diff_y) {
  _left->show(frame, diff_x, diff_y);
  _right->show(frame, diff_x, diff_y);
}

int AquatanEyes::mode() { return _mode; }

void AquatanEyes::mode(int m) {
  if (m >= EYE_IDLE && m < EYE_LAST) {
    _mode = m;
    if (_mode == EYE_WINK) {
      _left->mode(EYE_BLINK);
    } else {
      _left->mode(m);
      _right->mode(m);
    }
  }
}

int AquatanEyes::shape() { return _shape; }

void AquatanEyes::shape(int s) {
  if (s >= SHAPE_NORMAL && s < SHAPE_LAST) {
    _shape = s;
    _left->shape(s);
    _right->shape(s);
  }
}

void AquatanEyes::shape(int l, int r) {
  if (l >= SHAPE_NORMAL && l < SHAPE_LAST && r >= SHAPE_NORMAL &&
      r < SHAPE_LAST) {
    _shape = l;
    _left->shape(l);
    _right->shape(r);
  }
}

void AquatanEyes::setMoveTarget(int x, int y) {
  _left->setMoveTarget(x, y);
  _right->setMoveTarget(x, y);
}

void AquatanEyes::randomBlinking(int move) {
  static uint32_t prev_millis = 0;
  if (millis() - prev_millis > 1000) {
    prev_millis = millis();
    if (mode() != EYE_SLEEP) {
      if (mode() == EYE_IDLE) {
        int r = random(60);
        mode(r < 20 ? EYE_BLINK : EYE_IDLE);
        if (move == 1 && random(100) < 20) {
          mode(EYE_MOVE);
          int r1 = random(100);
          int r2 = random(100);
          setMoveTarget(r1 > 67 ? -1 : (r1 > 33 ? 1 : 0),
                        r2 > 67 ? -1 : (r2 > 33 ? 1 : 0));
        }
      }
    }
  }
}

int AquatanEyes::isBlinking() {
  return (mode() == EYE_BLINK) ? 1 : 0;
}