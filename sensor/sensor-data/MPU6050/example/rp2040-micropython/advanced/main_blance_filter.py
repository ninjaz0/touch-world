import machine
import utime
import imu 
import math
import blancefilter

i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
print("I2C addr: ", i2c.scan()[0])
sensor = imu.MPU6050(i2c)
lasttime = 0

while True:
    thistims = utime.ticks_ms()
    dt = (thistims-lasttime)/1000
    utime.sleep_ms(25)
    r=blancefilter.blance_filter(sensor.gyro.x,sensor.gyro.y,sensor.gyro.z,sensor.accel.x,sensor.accel.y,sensor.accel.z)
    print("{:.1f} {:.1f}".format(r[0],r[1]))
    lasttime = thistims