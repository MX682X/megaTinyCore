// This is a demonstration on how to use an input device to trigger changes on your neo pixels.
// You should wire a momentary push button to connect from ground to a digital IO pin.  When you
// press the button it will change to a new pixel animation.  Note that you need to press the
// button once to start the first animation!

#include <tinyNeoPixel.h>

#define BUTTON_PIN   PIN_PA2    // Digital IO pin connected to the button.  This will be
// driven with a pull-up resistor so the switch should
// pull the pin to ground momentarily.  On a high -> low
// transition the button press logic will execute.
// These pins were chosen because they are present on all parts; this sketch is used for automated testing.
// Since 8-pin parts with UPDI enabled have only 5 available IO pins (PA1, 2, 3, 6, and 7), and PA3 is the
// external clock pin (we detect and give an error when using external clock source if you try to use the
// clock pin for anything else, since that doesn't work). Serial in turn is on either PA6/7 or PA1/2 (alt) so unless we
// want to make life hard on people modifying the sketch and maybe adding serial debug logging, PA1 and PA2 are the best
// choices for the pin in this sketch as an example and automated testing sketch.

// You can use any I/O pin that is not being overridden by some peripheral for either purpose.
// No pin is inherently better or worse than any other for either of these purposes; it's all about what
// other things you need pins for, and whether any of them are picky about which pins are used.

#define PIXEL_PIN    PIN_PA1    // Digital IO pin connected to the NeoPixels

#define PIXEL_COUNT 16

// Parameter 1 = number of pixels in strip,
// Parameter 2 = pin number (most are valid)
// Parameter 3 = color order (NEO_RGB, NEO_GRB, etc).
// Unlike the Adafruit library there's no 400 kHz option. I don't think I have ever seen 400 kHz pixels, and they may no longer be made.
// Because the optimizer's hands are tied when working with classes it would impose undue burdens on all users to support it, plus it would
// need the asm routines adapted to every supported clock speed... all for the sake of a part that seems extinct in the wild.


tinyNeoPixel strip = tinyNeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB);

bool oldState = HIGH;
int showType = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIXEL_PIN, OUTPUT);
  // strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Get current button state.
  bool newState = digitalRead(BUTTON_PIN);

  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      showType++;
      if (showType > 9) {
        showType = 0;
      }
      startShow(showType);
    }
  }

  // Set the last button state to the old state.
  oldState = newState;
}

void startShow(int i) {
  switch (i) {
    case 0: colorWipe(strip.Color(0, 0, 0), 50);    // Black/off
      break;
    case 1: colorWipe(strip.Color(255, 0, 0), 50);  // Red
      break;
    case 2: colorWipe(strip.Color(0, 255, 0), 50);  // Green
      break;
    case 3: colorWipe(strip.Color(0, 0, 255), 50);  // Blue
      break;
    case 4: theaterChase(strip.Color(127, 127, 127), 50); // White
      break;
    case 5: theaterChase(strip.Color(127,   0,   0), 50); // Red
      break;
    case 6: theaterChase(strip.Color(0,   0, 127), 50);   // Blue
      break;
    case 7: rainbow(20);
      break;
    case 8: rainbowCycle(20);
      break;
    case 9: theaterChaseRainbow(50);
      break;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { // do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  // turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      // turn every third pixel off
      }
    }
  }
}

// Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel((i + j) % 255)); // turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      // turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
