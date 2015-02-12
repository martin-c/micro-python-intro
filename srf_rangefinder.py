"""
A utility library for interfacing with the SRF-08 and SRF-10 ultrasonic
rangefinders.
http://www.robot-electronics.co.uk/htm/srf08tech.shtml
http://www.robot-electronics.co.uk/htm/srf10tech.htm

Utilizes I2C library for reads and writes.

The MIT License (MIT)
Copyright (c) 2015 Martin Clemons
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""
from pyb import I2C


class SRF_RANGE_UNITS:
    """ SRF-XX rangefinder constants. """
    IN = 0x50
    CM = 0x51
    US = 0x52


class SRFBase(object):
    """
    A base class for SRF08 and SRF10 rangefinders.
    Essentially a SRF-xx rangefinder emulates a 24xx series EEPROM and implements
    a number of user readable and writable registers. These registers map to
    the specific hardware functions and readings from the rangefinder.
    Since the '08 and '10 are very similar in their functionality this class
    serves as a base implementation which can be overridden to form a class
    for a specific sensor.

    """
    def __init__(self, *args, **kwargs):
        """
        If any arguments are present self.init() is called.

        """
        super(SRFBase, self).__init__()
        self.i2c = None
        self.bus_addr = None
        self.rxb = bytearray(4)
        if len(args) > 0:
            self.init(*args, **kwargs)

    def init(self, *args, **kwargs):
        """
        Initialize a SRF sensor instance.
        There are two options for parameters passed when calling this function:
        1. Pass the initialization parameters for an pyb.I2C object.
           The initialization parameters will be used to initialize a new I2C
           instance which will be used to communicate with the sensor.
           If bus_address has not been set, a bus scan will be performed and the
           first available address will be used.
        2. Pass an already initialized pyb.I2C object.
           The instance passed in will be used to communicate with the sensor.
           The I2C instance should be initialized before any methods which
           require communication are called.

        """
        if len(args) < 1:
            raise TypeError('Please supply an I2C object or bus number.')
        if type(args[0]) is int:
            # assume first argument is bus number for I2C constructor
            self.i2c = I2C(*args, **kwargs)
            if self.bus_addr is None:
                try:
                    # assign address of first device found
                    self.bus_addr = self.i2c.scan()[0]
                except TypeError:
                    raise Exception('Sensor not found on I2C bus.')
        else:
            # first argument is initialized I2C bus object
            self.i2c = args[0]

    def deinit(self):
        """
        De-init sensor instance.
        Calls deinit() on I2C instance associated with sensor, and also resets
        sensor bus address.

        """
        try:
            self.i2c.deinit()
        except AttributeError:
            pass
        self.i2c = None
        self.bus_addr = None

    def bus_address(self, *args):
        """
        Sets the rangefinder I2C bus address if provided, otherwise returns the
        current rangefinder bus address.

        """
        if len(args) > 0:
            self.bus_addr = args[0]
        else:
            return self.bus_addr

    def scan_bus(self):
        """
        Scans I2C bus and returns a list of addresses found.

        """
        return self.i2c.scan()

    def sw_rev(self):
        """
        Returns the software revision of sensor.

        """
        rev = bytearray((256,))
        self.i2c.mem_read(rev, self.bus_addr, 0)
        if rev[0] > 255:
            raise Exception('Error reading from sensor.')
        return rev[0]

    def set_max_range(self, range_mm):
        """
        Sets the maximum range of the sensor.
        :param range_mm: Integer range in mm, min. 43mm max 11008mm.
        :return:

        """
        if range_mm < 43:
            raise ValueError('Minimum range is 43mm.')
        if range_mm > 11008:
            raise ValueError('Maximum range is 11008mm.')
        c = int(range_mm) // 43 - 1
        self.i2c.mem_write(c, self.bus_addr, 2)

    def set_analog_gain(self, gain):
        """
        Sets the analog gain of the sensor.
        :param gain: Sensor gain register value.
        :return:
        """
        if gain < 0:
            raise ValueError('Gain register must be greater than 0.')
        self.i2c.mem_write(int(gain), self.bus_addr, 1)

    def measure_range(self, units=SRF_RANGE_UNITS.CM):
        """
        Initiate rangefinder ranging.
        :param units: SRF_RANGE_UNITS, either IC, CM, or US for µ seconds.
        :return:

        """
        self.i2c.mem_write(units, self.bus_addr, 0)

    def read_range(self):
        """
        Read the range registers after ranging has completed.
        :param:
        :return: A list of integer range values in the units specified by
        measure_range(). In the case of sensors which report multiple echos,
        the first item in the list represents the first echo and the nth item
        represents the nth echo. If no echos were returned list will be empty.

        """
        self.i2c.mem_read(self.rxb, self.bus_addr, 0)
        values = []
        # skip first 2 bytes, then unpack high and low bytes from buffer data
        # data is pack in big-endian form
        for i in range(2, len(self.rxb), 2):
            range_val = (self.rxb[i] << 8) + self.rxb[i+1]
            if range_val > 0:
                values.append(range_val)
        return values


class SRF08(SRFBase):
    """
    A SRF08 Rangefinder.
    Supports up to 17 echo range values.
    Maximum analog gain of 31.
    TODO: Add ability to read light meter.

    """
    def __init__(self, *args, **kwargs):
        super(SRF08, self).__init__(*args, **kwargs)
        self.rxb = bytearray(36)

    def __unicode__(self):
        return '<SRF08 address {} on {}>'.format(self.bus_addr, self.i2c)

    def set_analog_gain(self, gain):
        if gain > 31:
            raise ValueError('Gain register must be less than or equal to 31.')
        super(SRF08, self).set_analog_gain(gain)


class SRF10(SRFBase):
    """
    A SRF10 rangefinder.
    Supports single echo range value.
    Maximum analog gain of 16.

    """
    def __unicode__(self):
        return '<SRF10 address {} on {}>'.format(self.bus_addr, self.i2c)

    def set_analog_gain(self, gain):
        if gain > 16:
            raise ValueError('Gain register must be less than or equal to 16.')
        super(SRF10, self).set_analog_gain(gain)
