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

# define LPS2X_I2CADDR_DEFAULT 0x5D ///< LPS2X default i2c address
# define LPS2X_WHOAMI 0x0F          ///< Chip ID register

# define LPS22HB_CHIP_ID 0xB1   ///< LPS22 default device id from WHOAMI
# define LPS22_THS_P_L_REG 0x0C ///< Pressure threshold value for int
# define LPS22_CTRL_REG1 0x10   ///< First control register. Includes BD & ODR
# define LPS22_CTRL_REG2 0x11   ///< Second control register. Includes SW Reset
# define LPS22_CTRL_REG3 0x12 ///< Third control register. Includes interrupt polarity

# define LPS25HB_CHIP_ID 0xBD ///< LPS25HB default device id from WHOAMI
# define LPS25_CTRL_REG1 0x20 ///< First control register. Includes BD & ODR
# define LPS25_CTRL_REG2 0x21 ///< Second control register. Includes SW Reset
# define LPS25_CTRL_REG3 0x22 ///< Third control register. Includes interrupt polarity
# define LPS25_CTRL_REG4 0x23 ///< Fourth control register. Includes DRDY INT control
# define LPS25_INTERRUPT_CFG 0x24 ///< Interrupt control register
# define LPS25_THS_P_L_REG 0xB0   ///< Pressure threshold value for int

# define LPS2X_PRESS_OUT_XL(0x28 | 0x80) ///< | 0x80 to set auto increment on multi-byte read
# define LPS2X_TEMP_OUT_L  (0x2B | 0x80) ///< | 0x80 to set auto increment on


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


# typedef enum {
#   LPS25_RATE_ONE_SHOT,
#   LPS25_RATE_1_HZ,
#   LPS25_RATE_7_HZ,
#   LPS25_RATE_12_5_HZ,
#   LPS25_RATE_25_HZ,
# } lps25_rate_t;

# /**
#  * @brief
#  *
#  * Allowed values for `setDataRate`.
#  */
# typedef enum {
#   LPS22_RATE_ONE_SHOT,
#   LPS22_RATE_1_HZ,
#   LPS22_RATE_10_HZ,
#   LPS22_RATE_25_HZ,
#   LPS22_RATE_50_HZ,
#   LPS22_RATE_75_HZ,
# } lps22_rate_t;


class LPS2X:  # pylint: disable=too-many-instance-attributes
    """Base class ST LPS2x family of pressure sensors

        :param ~busio.I2C i2c_bus: The I2C bus the sensor is connected to.
        :param address: The I2C device address for the sensor. Default is ``0x5d`` but will accept
            ``0x5c`` when the ``SDO`` pin is connected to Ground.

    """

    _chip_id = ROUnaryStruct(_WHO_AM_I, "<B")
    _reset = RWBit(_CTRL_REG2, 2)
    _data_rate = RWBits(3, _CTRL_REG1, 4)
    _raw_temperature = ROUnaryStruct(_TEMP_OUT_L, "<h")
    _raw_pressure = ROBits(24, _PRESS_OUT_XL, 0, 3)

    def __init__(self, i2c_bus, address=_LPS25_DEFAULT_ADDRESS):
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, address)
        if not self._chip_id in [_LPS25_CHIP_ID]:
            raise RuntimeError(
                "Failed to find LPS2X! Found chip ID 0x%x" % self._chip_id
            )

        self.reset()
        self.initialize()

    def initialize(self):  # pylint: disable=no-self-use
        """Configure the sensor with the default settings. For use after calling `reset()`"""
        raise RuntimeError(
            "LPS2X Base class cannot be instantiated directly. Use LPS22 or LPS25 instead"
        )  # override in subclass

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


# /**
#  * @brief
#  *
#  * Allowed values for `setDataRate`.
#  */
# typedef enum {
#   LPS22_RATE_ONE_SHOT,
#   LPS22_RATE_1_HZ,
#   LPS22_RATE_10_HZ,
#   LPS22_RATE_25_HZ,
#   LPS22_RATE_50_HZ,
#   LPS22_RATE_75_HZ,
# } lps22_rate_t;


class LPS25(LPS2X):
    """Library for the ST LPS25 pressure sensors

        :param ~busio.I2C i2c_bus: The I2C bus the LPS25HB is connected to.
        :param address: The I2C device address for the sensor. Default is ``0x5d`` but will accept
            ``0x5c`` when the ``SDO`` pin is connected to Ground.

    """

    enabled = RWBit(_CTRL_REG1, 7)
    """Controls the power down state of the sensor. Setting to `False` will shut the sensor down"""

    def __init__(self, i2c_bus, address=_LPS25_DEFAULT_ADDRESS):

        Rate.add_values(
            (
                # leave these for backwards compatibility? nah
                ("LPS25_RATE_ONE_SHOT", 0, 0, None),
                ("LPS25_RATE_1_HZ", 1, 1, None),
                ("LPS25_RATE_7_HZ", 2, 7, None),
                ("LPS25_RATE_12_5_HZ", 3, 12.5, None),
                ("LPS25_RATE_25_HZ", 4, 25, None),
            )
        )

        super().__init__(i2c_bus, address)
        self.initialize()

    def initialize(self):
        """Configure the sensor with the default settings. For use after calling `reset()`"""
        self.enabled = True
        self.data_rate = Rate.LPS25_RATE_25_HZ  # pylint:disable=no-member


# """
#  class Adafruit_LPS2X {
# public:
#   Adafruit_LPS2X();
#   ~Adafruit_LPS2X();

#   bool begin_I2C(uint8_t i2c_addr = LPS2X_I2CADDR_DEFAULT,
#                  TwoWire *wire = &Wire, int32_t sensor_id = 0);

#   bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
#                  int32_t sensor_id = 0);
#   bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
#                  int8_t mosi_pin, int32_t sensor_id = 0);

#   void setPresThreshold(uint16_t hPa_delta);
#   bool getEvent(sensors_event_t *pressure, sensors_event_t *temp);
#   void reset(void);

#   Adafruit_Sensor *getTemperatureSensor(void);
#   Adafruit_Sensor *getPressureSensor(void);

# protected:
#   /**! @brief The subclasses' hardware initialization function
#      @param sensor_id The unique sensor id we want to assign it
#      @returns True on success, false if something went wrong! **/
#   virtual bool _init(int32_t sensor_id) = 0;

#   void _read(void);

#   float _temp,   ///< Last reading's temperature (C)
#       _pressure; ///< Last reading's pressure (hPa)

#   uint16_t _sensorid_pressure, ///< ID number for pressure
#       _sensorid_temp;          ///< ID number for temperature
#   float temp_scaling = 1;      ///< Different chips have different scalings
#   uint8_t inc_spi_flag =
#       0; ///< If this chip has a bitflag for incrementing SPI registers


#   Adafruit_BusIO_Register *ctrl1_reg = NULL;   ///< The first control register
#   Adafruit_BusIO_Register *ctrl2_reg = NULL;   ///< The second control register
#   Adafruit_BusIO_Register *ctrl3_reg = NULL;   ///< The third control register
#   Adafruit_BusIO_Register *threshp_reg = NULL; ///< Pressure threshold

# private:
#   friend class Adafruit_LPS2X_Temp;     ///< Gives access to private members to
#                                         ///< Temp data object
#   friend class Adafruit_LPS2X_Pressure; ///< Gives access to private
#                                         ///< members to Pressure data
#                                         ///< object


# };

# /** Specific subclass for LPS25 variant */
# class Adafruit_LPS25 : public Adafruit_LPS2X {
# public:
#   lps25_rate_t getDataRate(void);
#   void setDataRate(lps25_rate_t data_rate);
#   void powerDown(bool power_down);
#   void configureInterrupt(bool activelow, bool opendrain,
#                           bool pres_high = false, bool pres_low = false);

# protected:
#   bool _init(int32_t sensor_id);
# };

# /** Specific subclass for LPS22 variant */
# class Adafruit_LPS22 : public Adafruit_LPS2X {
# public:
#   lps22_rate_t getDataRate(void);
#   void setDataRate(lps22_rate_t data_rate);
#   void configureInterrupt(bool activelow, bool opendrain, bool data_ready,
#                           bool pres_high = false, bool pres_low = false,
#                           bool fifo_full = false, bool fifo_watermark = false,
#                           bool fifo_overflow = false);

# protected:
#   bool _init(int32_t sensor_id);
# };

# #endif

# """
