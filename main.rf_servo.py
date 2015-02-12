"""
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
import json
import sys
from pyb import I2C, LED, Servo, Switch
from srf_rangefinder import SRF08


def setup():
    """Initialize hardware."""
    global led, servo, sw, rf
    led = LED(2)
    servo = Servo(1)
    sw = Switch()
    # SRF08 rangefinder on I2C bus 2
    rf = SRF08(2, I2C.MASTER, baudrate=50000)
    # print some sensor info
    sensor_info = {
        'address': rf.bus_address() << 1,
        'revision': rf.sw_rev(),
    }
    sys.stdout.write(json.dumps(sensor_info) + '\n')
    # set max range and gain
    rf.set_max_range(6000)
    rf.set_analog_gain(16)


def loop():
    """main program loop."""
    global led, servo, sw, rf
    pyb.wfi()
    if sw():
        led.on()
        sys.stdout.write(json.dumps({'reset': True}) + '\n')
        servo.angle(-90)
        # allow servo to reach starting angle
        pyb.delay(500)
        for a in range(-90, 100, 10):
            # set new angle, allow servo to settle
            servo.angle(a, 150)
            pyb.delay(10)
            # measure distance in cm
            rf.measure_range()
            pyb.delay(75)
            # read distance, send json to host
            sample = {
                'angle': a,
                'range': rf.read_range(),
            }
            sys.stdout.write(json.dumps(sample) + '\n')
        servo.angle(0)
        led.off()


if __name__ == "__main__":
    """
    Main program loop, does not return.

    """
    setup()
    while True:
        loop()
