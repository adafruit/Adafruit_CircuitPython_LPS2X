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

Library for the ST LPS2x family of pressure sensors

* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `LPS25HW Breakout <https://www.adafruit.com/products/4258>`_

**Software and Dependencies:**
 * Adafruit CircuitPython firmware for the supported boards:
    https://circuitpythohn.org/downloads
 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LPS2x.git"
# The MIT License (MIT)
#
# Copyright (c) 2019 Bryan Siepert for Adafruit Industries
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

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LPS2XW.git"
from micropython import const
import adafruit_bus_device.i2c_device as i2cdevice
from adafruit_register.i2c_struct import UnaryStruct
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_bit import RWBit

# pylint: disable=bad-whitespace
# _INTERRUPT_CFG	= const(0x0B)
# _THS_P_L	    = const(0x0C)
# _THS_P_H	    = const(0x0D)
_WHO_AM_I = const(0x0F)
_CTRL_REG1 = const(0x20)
_CTRL_REG2 = const(0x21)
# _CTRL_REG3    	= const(0x2)
# _FIFO_CTRL    	= const(0x14)
# _REF_P_XL    	= const(0x15)
# _REF_P_L    	= const(0x16)
# _REF_P_H    	= const(0x17)
# _RPDS_L	    	= const(0x18)
# _RPDS_H	    	= const(0x19)
# _RES_CONF    	= const(0x1A)
# _INT_SOURCE	    = const(0x25)
# _FIFO_STATUS	= const(0x26)
# _STATUS		    = const(0x27)
_PRESS_OUT_XL = const(0x28 | 0x80)  # 0x28 | 0x80 to set auto increment
# _PRESS_OUT_XL	= const(0x28)
_PRESS_OUT_L = const(0x29)
_PRESS_OUT_H = const(0x2A)
_TEMP_OUT_L = const(0x2B)
# _TEMP_OUT_H	    = const(0x2C)
# _LPFP_RES	    = const(0x33)

_LPS25_CHIP_ID = 0xBD
_LPS25_DEFAULT_ADDRESS = 0x5D
# pylint: enable=bad-whitespace
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


class LPS25HW:  # pylint: disable=too-many-instance-attributes
    """Library for the ST LPS2x family of pressure sensors

        :param ~busio.I2C i2c_bus: The I2C bus the LPS34HW is connected to.
        :param address: The I2C device address for the sensor. Default is ``0x5d`` but will accept
            ``0x5c`` when the ``SDO`` pin is connected to Ground.

    """

    _chip_id = UnaryStruct(_WHO_AM_I, "<B")
    _reset = RWBit(_CTRL_REG2, 2)
    enabled = RWBit(_CTRL_REG1, 7)
    """Controls the power down state of the sensor. Setting to `False` will shut the sensor down"""
    _data_rate = RWBits(3, _CTRL_REG1, 4)

    _raw_temperature = ROBits(16, _TEMP_OUT_L, 0, 2)
    _raw_pressure = ROBits(24, _PRESS_OUT_XL, 0, 3)
    # _reference_pressure = RWBits(24, _REF_P_XL, 0, 3)
    # _pressure_offset = RWBits(16, _RPDS_L, 0, 2)

    # _block_updates = RWBit(_CTRL_REG1, 1)

    # _one_shot = RWBit(_CTRL_REG2, 0)

    # registers for configuring INT pin behavior
    # _interrupt_cfg = UnaryStruct(_CTRL_REG3, "<B") # to read all values for latching?

    # # INT status registers
    # _interrupt_active = RWBit(_INT_SOURCE, 2)
    # _pressure_low = RWBit(_INT_SOURCE, 1)
    # _pressure_high = RWBit(_INT_SOURCE, 0)

    # _auto_zero = RWBit(_INTERRUPT_CFG, 5)
    # _reset_zero = RWBit(_INTERRUPT_CFG, 4)

    # _interrupts_enabled = RWBit(_INTERRUPT_CFG, 3)
    # _interrupt_latch = RWBit(_INTERRUPT_CFG, 2)
    # _interrupt_low = RWBit(_INTERRUPT_CFG, 1)
    # _interrupt_high = RWBit(_INTERRUPT_CFG, 0)

    # _reset_filter = ROBits(8, _LPFP_RES, 0, 1)

    # _pressure_threshold = UnaryStruct(_THS_P_L, "<H")

    # low_pass_enabled = RWBit(_CTRL_REG1, 3)
    # """True if the low pass filter is enabled. Setting to `True` will reduce the sensor bandwidth
    # from ``data_rate/2`` to ``data_rate/9``, filtering out high-frequency noise."""

    # low_pass_config = RWBit(_CTRL_REG1, 2)
    # """Setting to `True` will reduce the sensor bandwidth
    # from ``data_rate/9`` to ``data_rate/20``, filtering out high-frequency noise."""
    def __init__(self, i2c_bus, address=_LPS25_DEFAULT_ADDRESS):
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, address)
        if not self._chip_id in [_LPS25_CHIP_ID]:
            raise RuntimeError("Failed to find LPS25HW! Chip ID 0x%x" % self._chip_id)

        self.reset()
        self.enabled = True
        self.data_rate = Rate.RATE_25_HZ  # pylint:disable=no-member
        print("poopietest")
        # self._block_updates = True
        # self._interrupt_latch = True

    def reset(self):
        """Reset the sensor, restoring all configuration registers to their defaults"""
        self._reset = True
        # wait for the reset to finish
        while self._reset:
            pass

    @property
    def pressure(self):
        """The current pressure measurement in hPa"""
        # reset the filter to prevent spurious readings
        # self._reset_filter # pylint: disable=pointless-statement

        raw = self._raw_pressure

        if raw & (1 << 23) != 0:
            raw = raw - (1 << 24)
        return raw / 4096.0

    @property
    def temperature(self):
        """The current temperature measurement in degrees C"""
        raw_temperature = self._raw_temperature
        # if raw_temperature & 0x8000:
        #     raw_temperature = data - 0xffff
        return 42.5 + raw_temperature / 480

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

    # def read_pressure(self):
    #     """ returns pressure in hPa """
    #     data = int.from_bytes(self.i2c.mem_read(3, self.address, LPS_PRESSURE_OUT | 0x80))
    #     if data & 0x80000000:
    #         data = data - 0xffffffff
    #     return data / 4096

    # def read_temperature(self):
    #     """ return temperature in degrees celsius """
    #     data = int.from_bytes(self.i2c.mem_read(2, self.address, LPS_TEMP_OUT | 0x80))
    #     if data & 0x8000:
    #         data = data - 0xffff
    #     return 42.5 + data/480

    # def take_measurement(self):
    #     """Update the value of ``pressure`` and ``temperature`` by taking a single measurement.
    #         Only meaningful if ``data_rate`` is set to ``ONE_SHOT``"""
    #     self._one_shot = True
    #     while self._one_shot:
    #         pass

    # def zero_pressure(self):
    #     """Set the current pressure as zero and report the ``pressure`` relative to it"""
    #     self._auto_zero = True
    #     while self._auto_zero:
    #         pass

    # def reset_pressure(self):
    #     """Reset ``pressure`` to be reported as the measured absolute value"""
    #     self._reset_zero = True

    # @property
    # def pressure_threshold(self):
    #     """The high presure threshold. Use ``high_threshold_enabled`` or
    #     ``high_threshold_enabled``to use it"""
    #     return self._pressure_threshold / 16

    # @pressure_threshold.setter
    # def pressure_threshold(self, value):
    #     """The high value threshold"""
    #     self._pressure_threshold = (value * 16)

    # @property
    # def high_threshold_enabled(self):
    #     """Set to `True` or `False` to enable or disable the high pressure threshold"""
    #     return self._interrupts_enabled and self._interrupt_high

    # @high_threshold_enabled.setter
    # def high_threshold_enabled(self, value):
    #     self._interrupts_enabled = value
    #     self._interrupt_high = value

    # @property
    # def low_threshold_enabled(self):
    #     """Set to `True` or `False` to enable or disable the low pressure threshold. **Note the
    #     low pressure threshold only works in relative mode**"""
    #     return self._interrupts_enabled and self._interrupt_low

    # @low_threshold_enabled.setter
    # def low_threshold_enabled(self, value):
    #     self._interrupts_enabled = value
    #     self._interrupt_low = value

    # @property
    # def high_threshold_exceeded(self):
    #     """Returns `True` if the pressure high threshold has been exceeded. Must be enabled by
    #     setting ``high_threshold_enabled`` to `True` and setting a ``pressure_threshold``."""
    #     return self._pressure_high

    # @property
    # def low_threshold_exceeded(self):
    #     """Returns `True` if the pressure low threshold has been exceeded. Must be enabled by
    #     setting ``high_threshold_enabled`` to `True` and setting a ``pressure_threshold``."""
    #     return self._pressure_low
