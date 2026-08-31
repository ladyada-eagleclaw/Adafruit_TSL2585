/**************************************************************************
  TSL2585 live light dashboard for the Adafruit ESP32-S2 TFT Feather.

  Connect a TSL2585 breakout to the Feather's STEMMA QT port. The display
  shows photopic, infrared, and factory-calibrated UVA measurements normalized
  to 1x gain. The logarithmic bars make both dim and bright changes visible.

  Adafruit invests time and resources providing this open source code.
  Please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution.
 **************************************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_TSL2585.h>
#include <SPI.h>

#if !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)

void setup() {
  Serial.begin(115200);
  Serial.println("This example requires an Adafruit ESP32-S2 TFT Feather.");
}

void loop() {
  delay(10);
}

#else

Adafruit_TSL2585 tsl2585;
Adafruit_ST7789 display(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

const uint16_t photopicColor = 0xFFE0;
const uint16_t infraredColor = 0xF980;
const uint16_t uvaColor = 0xA81F;

void drawDashboard(const tsl2585_data_t& data);
void drawReading(const char* label, float value, bool saturated, int16_t y,
                 uint16_t color);
void drawMessage(const char* line1, const char* line2, uint16_t color);
uint16_t getBarWidth(float value);

void setup() {
  Serial.begin(115200);

  // The Feather's TFT and STEMMA QT connector share a switched power supply.
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, LOW);
  delay(10);

  display.init(135, 240);
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);
  digitalWrite(TFT_BACKLITE, HIGH);

  drawMessage("TSL2585 LIGHT", "Looking for sensor...", ST77XX_CYAN);

  if (!tsl2585.begin()) {
    Serial.println("Could not find a TSL2585. Check the STEMMA QT cable.");
    drawMessage("TSL2585 NOT FOUND", "Check STEMMA QT cable", ST77XX_RED);
    while (true) {
      delay(10);
    }
  }

  if (!tsl2585.setIntegrationTime(50)) {
    Serial.println("Could not set the integration time.");
    drawMessage("TSL2585 ERROR", "Could not configure sensor", ST77XX_RED);
    while (true) {
      delay(10);
    }
  }

  Serial.println("Adafruit TSL2585 Feather ESP32-S2 TFT demo");
  Serial.println("Photopic\tInfrared\tUVA");
}

void loop() {
  if (!tsl2585.dataReady()) {
    delay(20);
    return;
  }

  tsl2585_data_t data;
  if (tsl2585.readData(&data)) {
    drawDashboard(data);

    Serial.print(data.photopic_1x, 1);
    Serial.print('\t');
    Serial.print(data.infrared_1x, 1);
    Serial.print('\t');
    Serial.println(data.uva_1x, 1);
  }

  delay(20);
}

void drawDashboard(const tsl2585_data_t& data) {
  canvas.fillScreen(ST77XX_BLACK);

  canvas.setTextWrap(false);
  canvas.setTextSize(2);
  canvas.setTextColor(ST77XX_WHITE);
  canvas.setCursor(4, 3);
  canvas.print("TSL2585 LIGHT");

  canvas.fillCircle(229, 9, 4, ST77XX_GREEN);
  canvas.drawFastHLine(0, 20, 240, 0x4208);

  drawReading("PHOTOPIC", data.photopic_1x, data.photopic_saturated, 25,
              photopicColor);
  drawReading("INFRARED", data.infrared_1x, data.infrared_saturated, 57,
              infraredColor);
  drawReading("UVA", data.uva_1x, data.uva_saturated, 89, uvaColor);

  canvas.setTextSize(1);
  canvas.setCursor(4, 126);
  if (data.photopic_saturated || data.infrared_saturated ||
      data.uva_saturated) {
    canvas.setTextColor(ST77XX_RED);
    canvas.print("SATURATED");
  } else {
    canvas.setTextColor(0xBDF7);
    canvas.print("1x normalized counts | AGC | 50 ms");
  }

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}

void drawReading(const char* label, float value, bool saturated, int16_t y,
                 uint16_t color) {
  canvas.setTextSize(1);
  canvas.setTextColor(color);
  canvas.setCursor(4, y);
  canvas.print(label);

  canvas.setTextSize(2);
  canvas.setTextColor(saturated ? ST77XX_RED : ST77XX_WHITE);
  canvas.setCursor(73, y - 3);
  canvas.print(value, 1);

  canvas.drawRect(4, y + 13, 232, 8, 0x4208);
  uint16_t width = getBarWidth(value);
  if (width > 0) {
    canvas.fillRect(6, y + 15, width, 4, color);
  }
}

void drawMessage(const char* line1, const char* line2, uint16_t color) {
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setTextWrap(false);
  canvas.setTextColor(color);
  canvas.setTextSize(2);
  canvas.setCursor(8, 38);
  canvas.print(line1);
  canvas.setTextSize(1);
  canvas.setTextColor(ST77XX_WHITE);
  canvas.setCursor(8, 72);
  canvas.print(line2);
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}

uint16_t getBarWidth(float value) {
  // A fixed logarithmic scale keeps the display stable while showing useful
  // movement over the sensor's wide dynamic range. 131070 is the largest
  // possible 16-bit result normalized from the sensor's minimum 0.5x gain.
  const float maxLogValue = 5.1175;
  float fraction = log10f(value + 1.0) / maxLogValue;

  if (fraction < 0.0) {
    fraction = 0.0;
  } else if (fraction > 1.0) {
    fraction = 1.0;
  }

  return (uint16_t)(fraction * 228.0);
}

#endif
