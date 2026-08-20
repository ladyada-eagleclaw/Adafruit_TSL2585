# Adafruit TSL2585 Library [![Build Status](https://github.com/adafruit/Adafruit_TSL2585/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_TSL2585/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_TSL2585/html/index.html)

This is the Adafruit Arduino library for the ams OSRAM TSL2585 / TSL25853
ambient light, infrared, and UVA sensor.

The TSL2585 uses I2C to communicate. The sensor provides three independently
amplified optical channels, factory UVA calibration, and a programmable
measurement sequencer.

## About the TSL2585

The TSL2585 is a miniature optical sensor with:

* A photopic channel shaped for the human eye response
* A 315 nm to 400 nm UVA channel
* A near-infrared channel
* Per-channel gain from 0.5x to 4096x
* Programmable integration time
* A fixed I2C address of `0x39`

## Installation

To install, use the Arduino Library Manager and search for "Adafruit TSL2585".

## Dependencies

* [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)

## Contributing

Contributions are welcome. Please open an issue or pull request on GitHub.

## Documentation and Doxygen

Documentation is generated with Doxygen. Examples are included in the
`examples` folder, and deterministic hardware tests are in `extras/hw_tests`.

## About this driver

Written by Limor Fried and the Adafruit team for Adafruit Industries.

MIT license. See `LICENSE` for more information. All text above must be
included in any redistribution.
