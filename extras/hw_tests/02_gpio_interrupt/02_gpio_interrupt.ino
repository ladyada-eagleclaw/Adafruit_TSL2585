#include <Adafruit_TSL2585.h>

#define INT_PIN 2
#define GPIO_PIN 3
#define UVA_LED_PIN 4

const uint32_t PIN_CHANGE_TIMEOUT_MS = 100;
const uint32_t INTERRUPT_TIMEOUT_MS = 1000;

Adafruit_TSL2585 tsl2585;
volatile bool interrupt_seen = false;

bool waitForPinState(uint16_t pin, uint8_t state, uint32_t timeout_ms);
void sensorInterruptHandler();
void haltWithFailure(const __FlashStringHelper *message);
void haltWithSuccess();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println("TSL2585 D2 INT and D3 GPIO hardware test");

  pinMode(INT_PIN, INPUT);
  pinMode(GPIO_PIN, INPUT_PULLUP);
  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println("D2 and D3 are inputs; D4 UVA LED is safely off");

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

  Serial.println();
  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling ALS interrupts failed"));
  }
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the initial ALS interrupt failed"));
  }
  if (!waitForPinState(INT_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D2 INT was not high while interrupts were disabled"));
  }
  Serial.println("D2 INT is high while ALS interrupts are disabled");

  // Every 16-bit result is below 0xFFFFFF, so this low threshold guarantees
  // an out-of-range photopic result without depending on room lighting.
  if (!tsl2585.setALSThresholds(
          TSL2585_CHANNEL_PHOTOPIC, TSL2585_MAX_INTERRUPT_THRESHOLD,
          TSL2585_MAX_INTERRUPT_THRESHOLD, 1)) {
    haltWithFailure(F("Configuring forced ALS interrupt thresholds failed"));
  }
  Serial.println("Forced out-of-range photopic thresholds configured");

  interrupt_seen = false;
  attachInterrupt(digitalPinToInterrupt(INT_PIN), sensorInterruptHandler,
                  FALLING);

  if (!tsl2585.enableALSInterrupt(true)) {
    haltWithFailure(F("Enabling ALS interrupts failed"));
  }

  uint32_t interrupt_start_ms = millis();
  while (!interrupt_seen &&
         (millis() - interrupt_start_ms) < INTERRUPT_TIMEOUT_MS) {
    delay(1);
  }
  if (!interrupt_seen) {
    haltWithFailure(F("D2 did not capture the ALS interrupt falling edge"));
  }
  Serial.println("D2 captured the ALS interrupt falling edge");

  if (digitalRead(INT_PIN) != LOW) {
    haltWithFailure(F("D2 was not low after the ALS interrupt"));
  }
  Serial.println("D2 read low while the ALS interrupt was active");

  if (!tsl2585.alsInterruptActive()) {
    haltWithFailure(F("Sensor AINT status was not set"));
  }
  Serial.println("Sensor AINT status is set");

  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling the active ALS interrupt failed"));
  }
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the active ALS interrupt failed"));
  }
  if (!waitForPinState(INT_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D2 did not return high after clearing the interrupt"));
  }
  Serial.println("D2 returned high after the ALS interrupt was cleared");

  if (tsl2585.alsInterruptActive()) {
    haltWithFailure(F("Sensor AINT status remained set after clearing"));
  }
  Serial.println("Sensor AINT status cleared");

  detachInterrupt(digitalPinToInterrupt(INT_PIN));
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

void sensorInterruptHandler() {
  interrupt_seen = true;
}

void haltWithFailure(const __FlashStringHelper *message) {
  detachInterrupt(digitalPinToInterrupt(INT_PIN));
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  tsl2585.setGPIOOutput(true);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.print("FAIL: ");
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  tsl2585.setGPIOOutput(true);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println();
  Serial.println("ALL GPIO AND INTERRUPT TESTS PASSED");
  while (true) {
    delay(100);
  }
}
