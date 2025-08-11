# 学习MPU6050的姿态解算原理及降噪算法

### 只使用加速度计

这里我们不妨认为横滚角 **roll** 指重力加速度与“X-Z 平面”的夹角，
俯仰角 **pitch** 是重力加速度与“Y-Z 平面”的夹角。

```
import machine
import utime
import imu 
import math

i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
print("I2C addr: ", i2c.scan()[0])
sensor = imu.MPU6050(i2c)

while True:
    #angle calculate
    angle_arg_roll = -sensor.accel.y / math.sqrt(sensor.accel.x**2 + sensor.accel.z**2)
    angle_arg_pitch = -sensor.accel.x / math.sqrt(sensor.accel.y**2 + sensor.accel.z**2)
    roll = math.degrees(math.atan(angle_arg_pitch))
    pitch = math.degrees(math.atan(angle_arg_roll))
    #print(sensor.accel.xyz, sensor.gyro.xyz, sensor.temperature)
    print("加速度：{:7.2f}{:7.2f}{:7.2f} 角度：{:9.2f}{:9.2f}" \
          .format(sensor.accel.x, sensor.accel.y, sensor.accel.z,  \
                  roll,pitch))
    utime.sleep(0.5)
```
由于加速度计对振动敏感，易产生高频噪声使解算出的姿态产生抖动，所以需要探索其他的解算方法来解决这个问题。

### 只使用陀螺仪

我们把横滚角 **roll**和 俯仰角 **pitch**  分别对时间 t 求导，得到横滚角速度和俯仰角速度。于是我们得到了姿态角速度这个概念。陀螺仪的输出是“绕各个轴转动的角速度”，这两者有个共同点：都是角速度。**惯性传感器姿态的晃动会让陀螺仪检测到绕各个轴的转动，所以只要明确姿态角速度与各个轴角速度的对应关系就可以通过陀螺仪得到姿态角速度了**。

姿态角速度对dt积分就是姿态角。

但是陀螺仪也有他的缺点：虽然陀螺仪对振动不敏感，不产生高频噪声，但陀螺仪有**零偏**、积分出的姿态会**飘移**。所以最好的方法是把两者的数据结合使用。

## 互补滤波

新建文件 blancefilter.py

```
import math
i1=0
x=0 
y=0
angleAx=0
angleAy=0
blance_filter_angle=[0,0,0,0,0,0,0,0,0]
def blance_filter(gx,gy,gz,ax,ay,az):
    global i1
    global angleAy
    global angleAx
    global x
    global y
    K1 =0.3; # 对加速度计取值的权重
    global blance_filter_angle
    dt=0.03;#注意：dt的取值为滤波器采样时间
    if abs(gy)<0.07:
        gy=0
    if abs(gx)<0.07:
        gx=0
    if ay!=0 or az!=0:
        angleAx=math.atan(ax/math.sqrt(ay*ay+az*az))*180/math.pi;#计算与x轴夹角
    if ax!=0 or az!=0:
        angleAy=math.atan(ay/math.sqrt(ax*ax+az*az))*180/math.pi
    if i1==0:
        blance_filter_angle[0]=angleAx
        blance_filter_angle[1]=angleAy
        i1=i1+1
    blance_filter_angle[0] = K1 * angleAx+ (1-K1) * (blance_filter_angle[0] - gy* dt);
    blance_filter_angle[1] = K1 * angleAy+ (1-K1) * (blance_filter_angle[1] + gx* dt);

    blance_filter_angle[2]=angleAx
    blance_filter_angle[3]=angleAy
    # blance_filter_angle[4]=x
    # blance_filter_angle[5]=y
    # blance_filter_angle[6]=gyo
    # blance_filter_angle[7]=gxo
    #print(blance_filterr_angle)
    return blance_filter_angle;
```

主程序：
```
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
```

## 四元数解算

新建 q4.py

```
#四元数解算
import math
Kp=4 #比例增益支配率收敛到加速度计/磁强计
Ki=0.0005 #积分增益支配率的陀螺仪偏见的衔接
halfT=0.018 #采样周期的一半
q0=1
q1=0
q2=0
q3=0 #四元数的元素 ,代表估计方向
exInt=0
eyInt=0
ezInt=0 #按比例缩小积分误差
def IMUupdate(gx,gy,gz,ax,ay,az):
    K=0.7
    a=[0,0,0,0,0,0,0,0]
    global Kp,Ki,halfT,q0,q1,q2,q3,exInt,eyInt,ezInt
    if ax!=0 or ay!=0 or az!=0: 
        norm=math.sqrt(ax*ax+ay*ay+az*az);
    ax=ax/norm; #单位化
    ay=ay/norm;
    az=az/norm;
    #估计方向的重力
    vx=2* (q1*q3-q0*q2 );
    vy=2* (q0*q1+q2*q3 );
    vz=q0*q0-q1*q1-q2*q2+q3*q3;
    #错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
    ex= (ay*vz-az*vy );
    ey= (az*vx-ax*vz );
    ez= (ax*vy-ay*vx );
    #积分误差比例积分增益
    exInt=exInt+ex*Ki;
    eyInt=eyInt+ey*Ki;
    ezInt=ezInt+ez*Ki;
    #调整后的陀螺仪测量
    gx=gx+Kp*ex+exInt;
    gy=gy+Kp*ey+eyInt;
    gz=gz+Kp*ez+ezInt;
    #整合四元数率和正常化
    q0=q0+ (-q1*gx-q2*gy-q3*gz )*halfT;
    q1=q1+ (q0*gx+q2*gz-q3*gy )*halfT;
    q2=q2+ (q0*gy-q1*gz+q3*gx )*halfT;
    q3=q3+ (q0*gz+q1*gy-q2*gx )*halfT;
    #正常化四元
    norm=math.sqrt(q0*q0+q1*q1+q2*q2+q3*q3 );
    q0=q0/norm;
    q1=q1/norm;
    q2=q2/norm;
    q3=q3/norm;
    Pitch=math.asin(-2*q1*q3+2*q0*q2)*57.3; #pitch ,转换为度数
    if -2*q1*q1-2*q2*q2+1!=0:
        Roll=math.atan((2*q2*q3+2*q0*q1)/(-2*q1*q1-2*q2*q2+1))*57.3; #rollv
    if q0*q0+q1*q1-q2*q2-q3*q3!=0:
        Yaw=math.atan((2*q1*q2+2*q0*q3)/(q0*q0+q1*q1-q2*q2-q3*q3))*57.3;
    a[0]=Pitch
    a[1]=Roll
    a[2]=Yaw
    # if ay*ay+az*az!=0:
    #     a[2]=-math.atan(ax/math.sqrt(ay*ay+az*az))*57.2957795
    # if ax*ax+az*az!=0:
    #     a[3]=math.atan(ay/math.sqrt(ax*ax+az*az))*57.2957795
    # a[4]=gx
    # a[5]=gy
    # a[6]=gz
    # a[0]=-K*Pitch-(1-K)*a[2]
    # a[1]=K*Roll+(1-K)*a[3]
    return a
```

主程序修改为：

```
import machine
import utime
import imu 
import math
import blancefilter
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

```