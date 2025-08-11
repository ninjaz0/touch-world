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