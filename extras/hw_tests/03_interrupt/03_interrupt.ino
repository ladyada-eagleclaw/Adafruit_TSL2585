#include <Adafruit_TSL2585.h>

#define INT_PIN 2
#define UVA_LED_PIN 4

const uint32_t PIN_CHANGE_TIMEOUT_MS = 100;
const uint32_t INTERRUPT_TIMEOUT_MS = 1000;
const uint8_t SAMPLES_PER_LEVEL = 6;
// The 24-bit threshold registers align 16-bit full counts at the MSB.
const uint8_t ALS_THRESHOLD_COUNT_SHIFT = 8;
const float MINIMUM_UVA_INCREASE_COUNTS = 25;
const float MINIMUM_UVA_INCREASE_FRACTION = 0.20F;

const tsl2585_gain_t UVA_TEST_GAINS[] = {
    TSL2585_GAIN_128X, TSL2585_GAIN_64X, TSL2585_GAIN_32X,
    TSL2585_GAIN_16X,  TSL2585_GAIN_8X,  TSL2585_GAIN_4X,
    TSL2585_GAIN_2X,   TSL2585_GAIN_1X,   TSL2585_GAIN_0_5X};

Adafruit_TSL2585 tsl2585;
volatile bool interrupt_seen = false;

bool waitForPinState(uint16_t pin, uint8_t state, uint32_t timeout_ms);
bool readAverageUVA(bool led_on, tsl2585_gain_t expected_gain, float *average,
                    bool *saturated);
void sensorInterruptHandler();
void haltWithFailure(const __FlashStringHelper *message);
void haltWithSuccess();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println(F("TSL2585 D2 UVA interrupt hardware test"));
  Serial.println(
      F("Aim the D4 365 nm LED directly at the sensor before testing."));

  pinMode(INT_PIN, INPUT);
  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println(F("D2 is an input; D4 UVA LED is safely off"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.setIntegrationTime(50)) {
    haltWithFailure(F("Setting the integration time failed"));
  }
  Serial.println(F("50 ms integration time set"));

  if (!tsl2585.setGain(TSL2585_CHANNEL_PHOTOPIC, TSL2585_GAIN_4X) ||
      !tsl2585.setGain(TSL2585_CHANNEL_IR, TSL2585_GAIN_4X)) {
    haltWithFailure(F("Setting photopic and IR gains failed"));
  }
  Serial.println(F("Photopic and IR gains set to 4x"));

  float led_off_average = 0;
  float led_on_average = 0;
  tsl2585_gain_t interrupt_gain = TSL2585_GAIN_0_5X;
  bool gain_selected = false;

  for (uint8_t gain_index = 0;
       gain_index < sizeof(UVA_TEST_GAINS) / sizeof(UVA_TEST_GAINS[0]);
       gain_index++) {
    tsl2585_gain_t gain = UVA_TEST_GAINS[gain_index];
    if (!tsl2585.setGain(TSL2585_CHANNEL_UVA, gain)) {
      haltWithFailure(F("Setting the UVA test gain failed"));
    }

    bool led_off_saturated;
    bool led_on_saturated;
    if (!readAverageUVA(false, gain, &led_off_average,
                        &led_off_saturated)) {
      haltWithFailure(F("Could not read the LED-off UVA samples"));
    }
    if (!readAverageUVA(true, gain, &led_on_average, &led_on_saturated)) {
      haltWithFailure(F("Could not read the LED-on UVA samples"));
    }
    digitalWrite(UVA_LED_PIN, LOW);

    Serial.println();
    Serial.print(F("UVA gain code 0x"));
    Serial.println((uint8_t)gain, HEX);
    Serial.print(F("LED off average: "));
    Serial.println(led_off_average, 1);
    Serial.print(F("LED on average:  "));
    Serial.println(led_on_average, 1);

    if (led_off_saturated || led_on_saturated) {
      Serial.println(
          F("UVA channel saturated; reducing the gain and retrying"));
      continue;
    }

    float increase = led_on_average - led_off_average;
    float required_increase =
        led_off_average * MINIMUM_UVA_INCREASE_FRACTION;
    if (required_increase < MINIMUM_UVA_INCREASE_COUNTS) {
      required_increase = MINIMUM_UVA_INCREASE_COUNTS;
    }
    Serial.print(F("Measured increase: "));
    Serial.print(increase, 1);
    Serial.print(F(" counts; required: "));
    Serial.print(required_increase, 1);
    Serial.println(F(" counts"));

    if (increase < required_increase) {
      haltWithFailure(
          F("UVA did not increase enough; check D4, polarity, resistor, and aim"));
    }

    interrupt_gain = gain;
    gain_selected = true;
    Serial.println(F("PASS: 365 nm LED produced a clear UVA response"));
    break;
  }

  if (!gain_selected) {
    haltWithFailure(F("UVA remained saturated at every gain setting"));
  }

  uint16_t uva_count_threshold =
      (uint16_t)(led_off_average +
                 ((led_on_average - led_off_average) / 2));
  uint32_t high_threshold =
      (uint32_t)uva_count_threshold << ALS_THRESHOLD_COUNT_SHIFT;
  Serial.println();
  Serial.print(F("UVA interrupt threshold: "));
  Serial.print(uva_count_threshold);
  Serial.print(F(" counts; register value: "));
  Serial.println(high_threshold);

  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling ALS interrupts failed"));
  }

  if (!tsl2585.setALSThresholds(TSL2585_CHANNEL_UVA, 0, high_threshold, 1)) {
    haltWithFailure(F("Configuring the UVA interrupt threshold failed"));
  }
  Serial.println(F("UVA high threshold configured"));

  bool final_led_off_saturated;
  if (!readAverageUVA(false, interrupt_gain, &led_off_average,
                      &final_led_off_saturated)) {
    haltWithFailure(F("Could not confirm the final LED-off UVA level"));
  }
  if (final_led_off_saturated || led_off_average >= uva_count_threshold) {
    haltWithFailure(F("LED-off UVA level was not below the threshold"));
  }
  Serial.print(F("PASS: final LED-off average "));
  Serial.print(led_off_average, 1);
  Serial.println(F(" is below the threshold"));

  // Changing thresholds can latch AINT even while pin routing is disabled.
  // Clear it only after fresh LED-off measurements are inside the new window.
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the initial ALS interrupt failed"));
  }
  if (!waitForPinState(INT_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D2 INT was not high while interrupts were disabled"));
  }
  Serial.println(F("D2 INT is high while ALS interrupts are disabled"));

  interrupt_seen = false;
  attachInterrupt(digitalPinToInterrupt(INT_PIN), sensorInterruptHandler,
                  FALLING);

  if (!tsl2585.enableALSInterrupt(true)) {
    haltWithFailure(F("Enabling ALS interrupts failed"));
  }
  Serial.println(F("UVA interrupt enabled while D4 remains off"));

  delay(100);
  if (interrupt_seen || digitalRead(INT_PIN) != HIGH ||
      tsl2585.alsInterruptActive()) {
    haltWithFailure(F("UVA interrupt asserted while D4 was off"));
  }
  Serial.println(F("PASS: D2 remained high while the UVA LED was off"));

  digitalWrite(UVA_LED_PIN, HIGH);
  Serial.println(F("D4 UVA LED turned on"));

  uint32_t interrupt_start_ms = millis();
  while (!interrupt_seen &&
         (millis() - interrupt_start_ms) < INTERRUPT_TIMEOUT_MS) {
    delay(1);
  }
  if (!interrupt_seen) {
    haltWithFailure(F("D2 did not capture the ALS interrupt falling edge"));
  }
  Serial.println(F("D2 captured the ALS interrupt falling edge"));

  if (digitalRead(INT_PIN) != LOW) {
    haltWithFailure(F("D2 was not low after the ALS interrupt"));
  }
  Serial.println(F("D2 read low while the ALS interrupt was active"));

  if (!tsl2585.alsInterruptActive()) {
    haltWithFailure(F("Sensor AINT status was not set"));
  }
  Serial.println(F("Sensor AINT status is set"));

  tsl2585_data_t interrupt_data;
  if (!tsl2585.readData(&interrupt_data)) {
    haltWithFailure(F("Could not read the interrupting UVA measurement"));
  }
  if (interrupt_data.uva_gain != interrupt_gain ||
      interrupt_data.uva <= uva_count_threshold ||
      interrupt_data.uva_saturated) {
    haltWithFailure(F("UVA interrupt measurement did not exceed the threshold"));
  }
  Serial.print(F("PASS: UVA interrupt count "));
  Serial.print(interrupt_data.uva);
  Serial.print(F(" exceeded threshold "));
  Serial.println(uva_count_threshold);

  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling the active ALS interrupt failed"));
  }
  digitalWrite(UVA_LED_PIN, LOW);
  delay(100);
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the active ALS interrupt failed"));
  }
  if (!waitForPinState(INT_PIN, HIGH, PIN_CHANGE_TIMEOUT_MS)) {
    haltWithFailure(F("D2 did not return high after clearing the interrupt"));
  }
  Serial.println(F("D2 returned high after the ALS interrupt was cleared"));

  if (tsl2585.alsInterruptActive()) {
    haltWithFailure(F("Sensor AINT status remained set after clearing"));
  }
  Serial.println(
      F("Sensor AINT status cleared; D4 UVA LED is safely off"));

  detachInterrupt(digitalPinToInterrupt(INT_PIN));
  haltWithSuccess();
}

void loop() {}

bool readAverageUVA(bool led_on, tsl2585_gain_t expected_gain, float *average,
                    bool *saturated) {
  digitalWrite(UVA_LED_PIN, led_on ? HIGH : LOW);
  delay(100);

  tsl2585_data_t data;
  if (!tsl2585.readData(&data)) {
    return false;
  }

  uint32_t sum = 0;
  *saturated = false;
  for (uint8_t sample = 0; sample < SAMPLES_PER_LEVEL; sample++) {
    if (!tsl2585.readData(&data) || data.uva_gain != expected_gain) {
      return false;
    }
    sum += data.uva;
    if (data.uva_saturated) {
      *saturated = true;
    }
  }

  *average = sum / (float)SAMPLES_PER_LEVEL;
  return true;
}

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
  digitalWrite(UVA_LED_PIN, LOW);
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  digitalWrite(UVA_LED_PIN, LOW);
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  Serial.println();
  Serial.println(F("ALL UVA INTERRUPT TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
