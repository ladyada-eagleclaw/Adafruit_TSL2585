/*!
 * @file Adafruit_TSL2585.cpp
 *
 * @mainpage Adafruit TSL2585 ambient light, infrared, and UVA sensor library
 *
 * @section intro_sec Introduction
 *
 * Arduino driver for the ams OSRAM TSL2585 / TSL25853 optical sensor.
 *
 * @section author Author
 *
 * Written by Limor Fried and the Adafruit team for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 */

#include "Adafruit_TSL2585.h"

/*! @brief Construct a new TSL2585 driver. */
Adafruit_TSL2585::Adafruit_TSL2585() {}

/*! @brief Release the allocated I2C device. */
Adafruit_TSL2585::~Adafruit_TSL2585() {
  if (i2c_dev != nullptr) {
    delete i2c_dev;
  }
}

/*!
 * @brief Initialize the I2C connection and configure three-channel ALS.
 * @param i2c_addr The sensor I2C address. The TSL25853 uses 0x39.
 * @param wire The I2C bus to use.
 * @return True when the device was identified and configured.
 */
bool Adafruit_TSL2585::begin(uint8_t i2c_addr, TwoWire* wire) {
  if (i2c_dev != nullptr) {
    delete i2c_dev;
    i2c_dev = nullptr;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  if (!i2c_dev->begin()) {
    return false;
  }

  if (!readRegister(TSL2585_REG_ID, &_device_id, 1) ||
      _device_id != TSL2585_DEVICE_ID) {
    return false;
  }
  if (!readRegister(TSL2585_REG_REV_ID, &_revision_id, 1) ||
      !readRegister(TSL2585_REG_AUX_ID, &_auxiliary_id, 1) ||
      !readRegister(TSL2585_REG_UV_CALIB, &_uv_calibration, 1)) {
    return false;
  }

  return configure();
}

/*!
 * @brief Check whether the configured I2C device acknowledges.
 * @return True when the sensor acknowledges its I2C address.
 */
bool Adafruit_TSL2585::isConnected() {
  return i2c_dev != nullptr && i2c_dev->detected();
}

/*!
 * @brief Power the oscillator and start continuous ALS measurements.
 * @return True when both I2C writes succeeded.
 */
bool Adafruit_TSL2585::enable() {
  if (!writeRegister(TSL2585_REG_ENABLE, TSL2585_ENABLE_PON)) {
    return false;
  }

  delay(1);
  if (!writeRegister(TSL2585_REG_ENABLE,
                     TSL2585_ENABLE_PON | TSL2585_ENABLE_AEN)) {
    return false;
  }

  _enabled = true;
  return true;
}

/*!
 * @brief Stop measurements and power down the oscillator.
 * @return True when the I2C write succeeded.
 */
bool Adafruit_TSL2585::disable() {
  if (!writeRegister(TSL2585_REG_ENABLE, 0)) {
    return false;
  }

  _enabled = false;
  return true;
}

/*!
 * @brief Set ALS integration time for the register-result path.
 * @param milliseconds Integration time from 0.25 ms through 90 ms.
 * @return True when the value was in range and written successfully.
 */
bool Adafruit_TSL2585::setIntegrationTime(float milliseconds) {
  if (milliseconds < 0.25F || milliseconds > 90.0F) {
    return false;
  }

  uint16_t sample_count = (uint16_t)(milliseconds * 4.0F + 0.5F);
  uint16_t register_value = sample_count - 1;
  bool was_enabled = _enabled;

  if (was_enabled && !disable()) {
    return false;
  }
  if (!writeRegister(TSL2585_REG_ALS_NR_SAMPLES0, register_value, 2)) {
    if (was_enabled) {
      enable();
    }
    return false;
  }

  _als_samples = register_value;
  if (was_enabled) {
    return enable();
  }
  return true;
}

/*!
 * @brief Get the configured ALS integration time.
 * @return Integration time in milliseconds.
 */
float Adafruit_TSL2585::getIntegrationTime() {
  return (_als_samples + 1) * 0.25F;
}

/*!
 * @brief Set the manual gain for one optical channel.
 * @param channel The photopic, infrared, or UVA channel.
 * @param gain Gain from 0.5x through 4096x.
 * @return True when the channel and gain were valid and the write succeeded.
 */
bool Adafruit_TSL2585::setGain(tsl2585_channel_t channel, tsl2585_gain_t gain) {
  if ((uint8_t)channel > TSL2585_CHANNEL_UVA ||
      (uint8_t)gain > TSL2585_GAIN_4096X) {
    return false;
  }

  tsl2585_gain_t previous_gain = _gains[channel];
  bool was_enabled = _enabled;
  _gains[channel] = gain;

  if (was_enabled && !disable()) {
    _gains[channel] = previous_gain;
    return false;
  }

  bool success;
  if (channel == TSL2585_CHANNEL_UVA) {
    success = writeRegister(TSL2585_REG_STEP0_GAIN_H, (uint8_t)gain);
  } else {
    uint8_t gains = (uint8_t)_gains[TSL2585_CHANNEL_PHOTOPIC] |
                    ((uint8_t)_gains[TSL2585_CHANNEL_IR] << 4);
    success = writeRegister(TSL2585_REG_STEP0_GAIN_L, gains);
  }

  if (!success) {
    _gains[channel] = previous_gain;
  }
  if (was_enabled && !enable()) {
    return false;
  }
  return success;
}

/*!
 * @brief Get the configured manual gain for one channel.
 * @param channel The photopic, infrared, or UVA channel.
 * @return The configured gain, or 0.5x for an invalid channel.
 */
tsl2585_gain_t Adafruit_TSL2585::getGain(tsl2585_channel_t channel) {
  if ((uint8_t)channel > TSL2585_CHANNEL_UVA) {
    return TSL2585_GAIN_0_5X;
  }
  return _gains[channel];
}

/*!
 * @brief Check whether a new ALS measurement is available.
 * @return True when ALS_DATA_VALID is set.
 */
bool Adafruit_TSL2585::dataReady() {
  uint8_t status2;
  if (!readRegister(TSL2585_REG_STATUS2, &status2, 1)) {
    return false;
  }
  return (status2 & TSL2585_STATUS2_DATA_VALID) != 0;
}

/*!
 * @brief Wait for and read one coherent three-channel measurement.
 *
 * STATUS2 is read before ALS_STATUS as required by the device. The complete
 * ALS_STATUS through ALS_STATUS3 block is then read in one transaction so the
 * channel values, saturation flags, and actual gains belong to one cycle.
 *
 * @param data Destination for the result.
 * @param timeout_ms Maximum wait for fresh data in milliseconds.
 * @return True when fresh data was read successfully.
 */
bool Adafruit_TSL2585::readData(tsl2585_data_t* data, uint32_t timeout_ms) {
  if (data == nullptr || !_enabled) {
    return false;
  }

  uint32_t start_ms = millis();
  while (!dataReady()) {
    if ((millis() - start_ms) >= timeout_ms) {
      return false;
    }
    delay(1);
  }

  uint8_t status2;
  if (!readRegister(TSL2585_REG_STATUS2, &status2, 1)) {
    return false;
  }

  uint8_t result[TSL2585_ALS_RESULT_BLOCK_SIZE];
  if (!readRegister(TSL2585_REG_ALS_STATUS, result,
                    TSL2585_ALS_RESULT_BLOCK_SIZE)) {
    return false;
  }

  uint8_t als_status = result[0];
  data->photopic = (uint16_t)result[1] | ((uint16_t)result[2] << 8);
  data->infrared = (uint16_t)result[3] | ((uint16_t)result[4] << 8);
  data->uva = (uint16_t)result[5] | ((uint16_t)result[6] << 8);
  data->uva_calibrated = calibrateUVA(data->uva);

  data->photopic_gain = (tsl2585_gain_t)(result[7] & TSL2585_GAIN_MASK);
  data->infrared_gain = (tsl2585_gain_t)((result[7] >> 4) & TSL2585_GAIN_MASK);
  data->uva_gain = (tsl2585_gain_t)(result[8] & TSL2585_GAIN_MASK);

  bool digital_saturation = (status2 & TSL2585_STATUS2_DIGITAL_SATURATION) != 0;
  data->photopic_saturated =
      digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_PHOTOPIC_SATURATION) != 0;
  data->infrared_saturated =
      digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_IR_SATURATION) != 0;
  data->uva_saturated = digital_saturation ||
                        (als_status & TSL2585_ALS_STATUS_UVA_SATURATION) != 0;

  return true;
}

/*!
 * @brief Apply the factory OTP correction to a raw UVA count.
 * @param raw_uva Raw UVA full counts from the coherent result block.
 * @return Factory-corrected UVA counts.
 */
float Adafruit_TSL2585::calibrateUVA(uint16_t raw_uva) {
  float correction = 1.0F - ((_uv_calibration - 127.0F) / 100.0F);
  if (correction <= 0.0F) {
    return raw_uva;
  }
  return raw_uva / correction;
}

/*! @return The cached TSL2585 device identification byte. */
uint8_t Adafruit_TSL2585::getDeviceID() {
  return _device_id;
}

/*! @return The cached silicon revision identification byte. */
uint8_t Adafruit_TSL2585::getRevisionID() {
  return _revision_id;
}

/*! @return The cached auxiliary identification byte. */
uint8_t Adafruit_TSL2585::getAuxiliaryID() {
  return _auxiliary_id;
}

/*! @return The cached factory UVA calibration byte, where 127 is nominal. */
uint8_t Adafruit_TSL2585::getUVCalibration() {
  return _uv_calibration;
}

/*! @brief Configure the recommended one-step, three-channel ALS sequence. */
bool Adafruit_TSL2585::configure() {
  if (!disable()) {
    return false;
  }

  _als_samples = TSL2585_DEFAULT_ALS_SAMPLES;
  _gains[TSL2585_CHANNEL_PHOTOPIC] = TSL2585_GAIN_128X;
  _gains[TSL2585_CHANNEL_IR] = TSL2585_GAIN_128X;
  _gains[TSL2585_CHANNEL_UVA] = TSL2585_GAIN_128X;

  if (!writeRegister(TSL2585_REG_MEAS_MODE0, TSL2585_MEAS_MODE0_FULL_COUNTS) ||
      !writeRegister(TSL2585_REG_MEAS_MODE1,
                     TSL2585_MEAS_MODE1_MSB_POSITION_12) ||
      !writeRegister(TSL2585_REG_SAMPLE_TIME0, TSL2585_SAMPLE_TIME_250_US, 2) ||
      !writeRegister(TSL2585_REG_ALS_NR_SAMPLES0, _als_samples, 2) ||
      !writeRegister(TSL2585_REG_MEAS_SEQR_FD_0, TSL2585_SEQUENCER_DISABLED) ||
      !writeRegister(TSL2585_REG_MEAS_SEQR_ALS_FD_1, TSL2585_SEQUENCER_STEP0) ||
      !writeRegister(TSL2585_REG_MEAS_SEQR_APERS, TSL2585_SEQUENCER_STEP0) ||
      !writeRegister(TSL2585_REG_MEAS_SEQR_RESIDUAL_0,
                     TSL2585_SEQUENCER_DISABLED) ||
      !writeRegister(TSL2585_REG_MEAS_SEQR_RESIDUAL_1,
                     TSL2585_SEQUENCER_DISABLED) ||
      !writeRegister(TSL2585_REG_STEP0_GAIN_L, TSL2585_DEFAULT_GAIN_L) ||
      !writeRegister(TSL2585_REG_STEP0_GAIN_H, TSL2585_DEFAULT_GAIN_H) ||
      !writeRegister(TSL2585_REG_STEP0_SMUX_L, TSL2585_RECOMMENDED_SMUX_L) ||
      !writeRegister(TSL2585_REG_STEP0_SMUX_H, TSL2585_RECOMMENDED_SMUX_H)) {
    return false;
  }

  return enable();
}

/*! @brief Read one or more consecutive registers with BusIO. */
bool Adafruit_TSL2585::readRegister(uint8_t address, uint8_t* buffer,
                                    uint8_t length) {
  if (i2c_dev == nullptr || buffer == nullptr || length == 0) {
    return false;
  }
  Adafruit_BusIO_Register reg(i2c_dev, address, length, LSBFIRST);
  return reg.read(buffer, length);
}

/*! @brief Write a one- or two-byte little-endian register with BusIO. */
bool Adafruit_TSL2585::writeRegister(uint8_t address, uint32_t value,
                                     uint8_t width) {
  if (i2c_dev == nullptr || width == 0 || width > 2) {
    return false;
  }
  Adafruit_BusIO_Register reg(i2c_dev, address, width, LSBFIRST);
  return reg.write(value);
}
