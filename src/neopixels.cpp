#include "neopixels.h"

NeoPixels::NeoPixels(NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> *leds) {
  _leds = leds;
  _frame_max = FPS;
  _blink_timer = 0;
}

void NeoPixels::addPixel(uint8_t id) {
  _pixel_array[_num_pixels] = id;
  _num_pixels++;
}

void NeoPixels::mode(LED_mode_t m) {
  _mode = m;
  _frame = 0;
}

void NeoPixels::period(float t) {
  _frame_max = (uint16_t)(t * FPS);
}

LED_mode_t NeoPixels::mode() { return _mode; }

void NeoPixels::color(RgbColor color) { _color = color; }

void NeoPixels::randomBlinking() {
  static uint32_t random_timer = 0;
  if (millis() - random_timer > 1000) {
    random_timer = millis();
    if (mode() == LED_ON) {
      int r = random(60);
      mode(r < 20 ? LED_FADE : LED_ON);
    }
  }
}

void NeoPixels::update() {
  uint16_t col_r, col_g, col_b;
  if ((millis() - _blink_timer) > (1000 / FPS)) {
    _blink_timer = millis();
    if (mode() == LED_FADE) {
      float framesin = 1 - sin((float)_frame * (PI / _frame_max));
      col_r = (uint16_t)((float)_color.R * framesin + 0.5);
      col_g = (uint16_t)((float)_color.G * framesin + 0.5);
      col_b = (uint16_t)((float)_color.B * framesin + 0.5);
    } else if (mode() == LED_BLINK) {
      if (_frame < _frame_max / 2) {
        col_r = col_g = col_b = 0;
      } else {
        col_r = _color.R;
        col_g = _color.G;
        col_b = _color.B;
      }
    } else if (mode() == LED_ON) {
      col_r = _color.R;
      col_g = _color.G;
      col_b = _color.B;
    } else if (mode() == LED_OFF) {
      col_r = col_g = col_b = 0;
    }
    for (int i = 0; i < _num_pixels; i++) {
      _leds->SetPixelColor(_pixel_array[i], RgbColor(col_r, col_g, col_b));
    }
    _leds->Show();
    _frame++;
    if (_frame == _frame_max) {
      if (mode() != LED_OFF) {
        mode(LED_ON); 
      }
    }
    _frame %= _frame_max;
  }
}
