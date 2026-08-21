#include <Adafruit_TSL2585.h>

#define GPIO_PIN 3
#define UVA_LED_PIN 4

const uint32_t PIN_CHANGE_TIMEOUT_MS = 100;

Adafruit_TSL2585 tsl2585;

bool waitForPinState(uint16_t pin, uint8_t state, uint32_t timeout_ms);
void haltWithFailure(const __FlashStringHelper *message);
void haltWithSuccess();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println("TSL2585 D3 GPIO hardware test");

  pinMode(GPIO_PIN, INPUT_PULLUP);
  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println("D3 is an input; D4 UVA LED is safely off");

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println("Begin succeeded");

  Serial.println();
  if (!tsl2585.setGPIOOutput(true)) {
    haltWithFailure(F("Releasing the sensor GPIO failed"));
  }
  if (!waitForPinState(GPIO_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D3 did not read high when GPIO was released"));
  }
  Serial.println("D3 read high when the sensor GPIO was released");

  if (!tsl2585.setGPIOOutput(false)) {
    haltWithFailure(F("Pulling the sensor GPIO low failed"));
  }
  if (!waitForPinState(GPIO_PIN, LOW, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D3 did not read low when GPIO was pulled low"));
  }
  Serial.println("D3 read low when the sensor GPIO was pulled low");

  if (!tsl2585.setGPIOOutput(true)) {
    haltWithFailure(F("Re-releasing the sensor GPIO failed"));
  }
  if (!waitForPinState(GPIO_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D3 did not return high when GPIO was released"));
  }
  Serial.println("D3 returned high when the sensor GPIO was released");

  haltWithSuccess();
}

void loop() {}

bool waitForPinState(uint16_t pin, uint8_t state, uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  while ((millis() - start_ms) < timeout_ms) {
    if (digitalRead(pin) == state) {
      return true;
    }
    delay(1);
  }
  return false;
}

void haltWithFailure(const __FlashStringHelper *message) {
  tsl2585.setGPIOOutput(true);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.print("FAIL: ");
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  tsl2585.setGPIOOutput(true);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println();
  Serial.println("ALL GPIO TESTS PASSED");
  while (true) {
    delay(100);
  }
}
