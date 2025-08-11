import machine
import utime
import imu 
import math
import q4

i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
print("I2C addr: ", i2c.scan()[0])
sensor = imu.MPU6050(i2c)
lasttime = 0

while True:
    thistims = utime.ticks_ms()
    dt = (thistims-lasttime)/1000
    utime.sleep_ms(25)
    q=q4.IMUupdate(sensor.gyro.x*0.0174533,sensor.gyro.y*0.0174533,sensor.gyro.z*0.0174533,sensor.accel.x*9.8,sensor.accel.y*9.8,sensor.accel.z*9.8)
    print("{:.1f},{:.1f},{:.1f}\n".format(q[0],q[1],q[2]))
    lasttime = thistims