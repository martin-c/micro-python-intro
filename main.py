# main.py -- put your code here!
import json
import sys
from pyb import I2C, LED, Servo, Switch

# initialize hardware
led = LED(2)
servo = Servo(1)
sw = Switch()
# SRF08 rangefinder on I2C bus 2
rf = I2C(2, I2C.MASTER, baudrate=50000)

# determine RF address, we assume it's the first device on the bus
try:
    rf_addr = rf.scan()[0]
except TypeError:
    raise Exception('SRF08 not found')
# log sensor address and HW revision
rxb = bytearray(36)
rf.mem_read(rxb, rf_addr, 0)
sensor_info = {
    'address': rf_addr << 1,
    'revision': rxb[0],
}
sys.stdout.write(json.dumps(sensor_info) + '\n')
# set max range to 138 (138 + 1) * 43mm = ~6m
rf.mem_write(138, rf_addr, 2)
# set analog gain
rf.mem_write(31, rf_addr, 1)

# loop forever
while True:
    pyb.wfi()
    if sw():
        led.on()
        sys.stdout.write(json.dumps({'reset': True}) + '\n')
        servo.angle(-90)
        # allow servo to reach starting angle
        pyb.delay(500)
        for a in range(-90, 100, 10):
            # set new angle, allow servo to settle
            servo.angle(a)
            pyb.delay(100)
            # read distance in cm
            rf.mem_write(0x51, rf_addr, 0)
            pyb.delay(100)
            rf.mem_read(rxb, rf_addr, 0)
            #	 if rxb[2] != 255 and rxb[3] != 255:
                # ranging complete
            #	     break
            sample = {
            'angle': a,
            'range': (rxb[2] << 8) + rxb[3],
            }
            sys.stdout.write(json.dumps(sample) + '\n')
        led.off()
