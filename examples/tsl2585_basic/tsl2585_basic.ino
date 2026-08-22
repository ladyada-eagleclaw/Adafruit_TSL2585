#include <Adafruit_TSL2585.h>

Adafruit_TSL2585 tsl2585;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println("TSL2585 photopic, infrared, and UVA sensor test");

  if (!tsl2585.begin()) {
    Serial.println("Could not find a TSL2585. Check the wiring and I2C address.");
    while (true) {
      delay(10);
    }
  }

  Serial.print("Device ID: 0x");
  Serial.println(tsl2585.getDeviceID(), HEX);
  Serial.print("Revision ID: 0x");
  Serial.println(tsl2585.getRevisionID(), HEX);
  Serial.print("Factory UVA calibration byte: ");
  Serial.println(tsl2585.getUVCalibration());

  // Register-based results support integration times from 0.25 ms to 90 ms.
  if (!tsl2585.setIntegrationTime(50)) {
    Serial.println("Could not set the integration time.");
    while (true) {
      delay(10);
    }
  }
  Serial.print("Integration time: ");
  Serial.print(tsl2585.getIntegrationTime(), 2);
  Serial.println(" ms");
}

void loop() {
  if (!tsl2585.dataReady()) {
    delay(100);
    return;
  }

  tsl2585_data_t data;
  if (tsl2585.readData(&data)) {
    Serial.print("Photopic: ");
    Serial.print(data.photopic);
    Serial.print("    IR: ");
    Serial.print(data.infrared);
    Serial.print("    UVA raw: ");
    Serial.print(data.uva);
    Serial.print("    UVA calibrated: ");
    Serial.print(data.uva_calibrated, 1);

    if (data.photopic_saturated || data.infrared_saturated ||
        data.uva_saturated) {
      Serial.print("    saturated");
    }
    Serial.println();
  }

  delay(100);
}
