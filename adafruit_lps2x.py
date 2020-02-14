# The MIT License (MIT)
#
# Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
`adafruit_lps2x`
================================================================================

Library for the ST LPS2X family of pressure sensors

* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `LPS25HB Breakout <https://www.adafruit.com/products/45XX>`_

**Software and Dependencies:**
 * Adafruit CircuitPython firmware for the supported boards:
    https://circuitpythohn.org/downloads
 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LPS2X.git"
from micropython import const
import adafruit_bus_device.i2c_device as i2cdevice
from adafruit_register.i2c_struct import ROUnaryStruct
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_bit import RWBit

_WHO_AM_I = const(0x0F)
_CTRL_REG1 = const(0x20)
_CTRL_REG2 = const(0x21)
_PRESS_OUT_XL = const(0x28 | 0x80)  # | 0x80 to set auto increment on multi-byte read
_TEMP_OUT_L = const(0x2B | 0x80)  # | 0x80 to set auto increment on multi-byte read

_LPS25_CHIP_ID = 0xBD
_LPS25_DEFAULT_ADDRESS = 0x5D


class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        "creates CV entires"
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        "Returns true if the given value is a member of the CV"
        return value in cls.string


class Rate(CV):
    """Options for ``data_rate``

    +-----------------------+------------------------------------------------------------------+
    | Rate                  | Description                                                      |
    +-----------------------+------------------------------------------------------------------+
    | ``Rate.ONE_SHOT``     | Setting `data_rate` to ``Rate.ONE_SHOT`` takes a single pressure |
    |                       | and temperature measurement                                      |
    +-----------------------+------------------------------------------------------------------+
    | ``Rate.RATE_1_HZ``    | 1 Hz                                                             |
    +-----------------------+------------------------------------------------------------------+
    | ``Rate.RATE_7_HZ``    | 7 Hz                                                             |
    +-----------------------+------------------------------------------------------------------+
    | ``Rate.RATE_12_5_HZ`` | 12.5 Hz                                                          |
    +-----------------------+------------------------------------------------------------------+
    | ``Rate.RATE_25_HZ``   | 25 Hz                                                            |
    +-----------------------+------------------------------------------------------------------+

    """

    pass  # pylint: disable=unnecessary-pass


Rate.add_values(
    (
        ("RATE_ONE_SHOT", 0, 0, None),
        ("RATE_1_HZ", 1, 1, None),
        ("RATE_7_HZ", 2, 7, None),
        ("RATE_12_5_HZ", 3, 12.5, None),
        ("RATE_25_HZ", 4, 25, None),
    )
)


class LPS2X:  # pylint: disable=too-many-instance-attributes
    """Library for the ST LPS2x family of pressure sensors

        :param ~busio.I2C i2c_bus: The I2C bus the LPS25HB is connected to.
        :param address: The I2C device address for the sensor. Default is ``0x5d`` but will accept
            ``0x5c`` when the ``SDO`` pin is connected to Ground.

    """

    _chip_id = ROUnaryStruct(_WHO_AM_I, "<B")
    _reset = RWBit(_CTRL_REG2, 2)
    enabled = RWBit(_CTRL_REG1, 7)
    """Controls the power down state of the sensor. Setting to `False` will shut the sensor down"""
    _data_rate = RWBits(3, _CTRL_REG1, 4)
    _raw_temperature = ROUnaryStruct(_TEMP_OUT_L, "<h")
    _raw_pressure = ROBits(24, _PRESS_OUT_XL, 0, 3)

    def __init__(self, i2c_bus, address=_LPS25_DEFAULT_ADDRESS):
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, address)
        if not self._chip_id in [_LPS25_CHIP_ID]:
            raise RuntimeError(
                "Failed to find LPS25HB! Found chip ID 0x%x" % self._chip_id
            )

        self.reset()
        self.enabled = True
        self.data_rate = Rate.RATE_25_HZ  # pylint:disable=no-member

    def reset(self):
        """Reset the sensor, restoring all configuration registers to their defaults"""
        self._reset = True
        # wait for the reset to finish
        while self._reset:
            pass

    @property
    def pressure(self):
        """The current pressure measurement in hPa"""
        raw = self._raw_pressure

        if raw & (1 << 23) != 0:
            raw = raw - (1 << 24)
        return raw / 4096.0

    @property
    def temperature(self):
        """The current temperature measurement in degrees C"""
        raw_temperature = self._raw_temperature
        return (raw_temperature / 480) + 42.5

    @property
    def data_rate(self):
        """The rate at which the sensor measures ``pressure`` and ``temperature``. ``data_rate``
        shouldbe set to one of the values of ``adafruit_lps2x.DataRate``. Note that setting
        ``data_rate``to ``Rate.ONE_SHOT`` places the sensor into a low-power shutdown mode where
        measurements toupdate ``pressure`` and ``temperature`` are only taken when
        ``take_measurement`` is called."""
        return self._data_rate

    @data_rate.setter
    def data_rate(self, value):
        if not Rate.is_valid(value):
            raise AttributeError("data_rate must be a `Rate`")

        self._data_rate = value
