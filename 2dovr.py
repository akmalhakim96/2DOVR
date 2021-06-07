#! /usr/bin/python3
#  Yasushi Honda 2021 5/27
#  2dovr_210513.py
#  2021-04-16
#  Masashi Yamada

#  各モジュールインポート
import csv
import time
import math
import sys
import cv2
import numpy as np
#  Pythonファイルインポート 
import ovm as OVM_py          # 2次元最適速度モデル関係
import picam as PICAM_py # picamera関係
import modules.motor5a as mt         #  (改良版)モーターを回転させるためのモジュール
#import pixy_210416 as PIXY_py       # Pixyカメラ関係
import modules.vl53_4a as lidar     #  赤外線レーザーレーダ 3つの場合
#import modules.tof2_3a as lidar      #  赤外線レーザーレーダ 2つの場合

#sokcet tuusinn kannkei
import socket



select_hsv = "n"
motor_run = "n"
imshow = "n"

SLEEP = 0.2
EX_TIME = 3    #  (min)
BUS = 1         # bus number
I2C_ADDR = 0x54 #I2Cアドレス
GPIO_L = 17     #  左モーターのgpio 17番
GPIO_R = 18     #  右モーターのgpio 18番
MAX_SPEED = 50  # パーセント
DT = 0.1
dt = DT

#  パラメータ記載のファイルの絶対パス
FILE = "parm.csv" 


#  実験パラメータ読み込み
def Parameter_read(file_path):
    tmp = []
    reader = csv.reader(file_path)
    header = next(reader)
    for row in reader:
        if len(row) == 0:
           pass 
        else:
            tmp.append(float(row[0]))
            tmp.append(float(row[1]))
            tmp.append(float(row[2]))
            tmp.append(float(row[3]))
            tmp.append(float(row[4]))
            tmp.append(float(row[5]))
    return tmp
    
#  物体未認識時のhyperbolic-tan
def tanh1(x):
    alpha=30.0
    alpha2=30.0
    beta=0.004 # 0.004
    beta2=10.00
    b=360  # 280
    c=0.0
    f=math.tanh(beta*(x-b)) + math.tanh(beta2*(x-b))+c
    return f

def tanh2(x):
    alpha=30.0
    alpha2=30.0
    beta=0.004 # 0.004
    beta2=10.00
    b=360  # 360
    c=0.0
    f=math.tanh(beta*(x-b)) + math.tanh(beta2*(x-b))+c
    return f
"""
def tanh(x):
    alpha=30.0
    alpha2=30.0
    beta=0.004 #  0.004
    beta2=10.00
    b=160  #  280
    c=0
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    delta = 0.1 #  beta
    p = 250     #  b
    q = 0.0     #  c
    f = (math.tanh(delta * (x - p) ) + q )
    return f
"""
#  各変数定義
parm = []

#  パラメータ読み込み
file_pointer = open(FILE,'r')
parm = Parameter_read(file_pointer)

#  インスタンス生成
ovm = OVM_py.Optimal_Velocity_class(parm)         #  2次元最適速度モデル関係
tofR,tofL,tofC=lidar.start() #  赤外線レーザ(3)
#tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
time.sleep(2)
picam =PICAM_py.PI_CAMERA_CLASS() 
print("picamera 接続完了\n")
time.sleep(2)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

"""
print("===============================")
print("実験開始から3分10秒で自動停止します．")
print("===============================")

print("===============================")
print("=  実験開始  =")
print("===============================")
print()
"""
count = 0
data = []
gamma=0.33 # Center weight
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")

if select_hsv=='y':
    lower_light,upper_light=picam.calc_hsv()
else:
    #Red Cup H:S:V=3:140:129
    # h,s,v = 171,106,138
    # 177  139  141 2021/06/01
    H = 177; S = 139; V = 141
    h_range = 20; s_range = 80; v_range = 80 # 明度の許容範囲
    lower_light = np.array([H-h_range, S-s_range, V-v_range])
    upper_light = np.array([H+h_range, S+s_range, V+v_range])
start = time.time()
now = start


key=cv2.waitKey(1)
vl=0;vr=0
while key!=ord('q'):
    #  実験中
    dist,theta,frame = picam.calc_dist_theta(lower_light, upper_light)
    count = count + 1
    try :
        if dist != None:
            mode = "picam"
            dist = float(dist)
            # pixyカメラで物体を認識している時
            vl, vr, omega = ovm.calc(dist,theta,dt)
            
            

        else:
            dist = float(2000)
            theta = 0.0
            vl, vr, omega = ovm.calc(dist,theta,dt)
            
        mode = "VL53L0X"
        lidar_distanceL=tofL.get_distance()
        if lidar_distanceL>2000:
            lidar_distanceL=2000

        lidar_distanceC=tofC.get_distance()
        if lidar_distanceC>2000:
            lidar_distanceC=2000
           
        lidar_distanceR=tofR.get_distance()
        if lidar_distanceR>2000:
            lidar_distanceR=2000

        if lidar_distanceL>0 and lidar_distanceC>0:
            areaL=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceL))
        if lidar_distanceR>0 and lidar_distanceC>0:
            areaR=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceR))


        tof_r = tanh1(areaL) / 2
        tof_l = tanh2(areaR) / 2
        print("\r %6.2f " % (now-start),end="")
        print(" dist=%6.2f " % dist, end="")
        print(" theta=%6.2f " % theta, end="")
        #print(" v_L=%6.2f " % vl, end="")
        #print(" v_R=%6.2f " % vr, end="")
        print(" dL=%6.2f " % lidar_distanceL, end="")
        print(" dC=%6.2f " % lidar_distanceC, end="")
        print(" dR=%6.2f " % lidar_distanceR, end="")

        vl = vl * tof_l * MAX_SPEED 
        vr = vr * tof_r * MAX_SPEED
        if vl > 100:  # 左モータに対する
            vl =100   # 閾値処理
        if vl < -100: # -1 < v_l < 1
            vl = -100 #
        
        if vr > 100:  # 右モータに対する
            vr =100   # 閾値処理
        if vr < -100: # -1 < v_r < 1
            vr = -100 #
        if motor_run == 'y':
            mL.run(vl)
            mR.run(vr)
        if imshow == 'y':    
            cv2.imshow("frame",frame)
            key=cv2.waitKey(1)
        time.sleep(DT)
        last = now
        now = time.time()
        dt = now-last
    except KeyboardInterrupt:
        mR.stop()
        mL.stop()
        print("Ctrl + C button pressed")
        sys.exit("\nsystem exit ! \n")
mR.stop()
mL.stop()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
print()
print("===============================")
print("=  実験時間 {:.1f} (sec)".format(now-start))
print("=  q_s--->")
print("===============================")
print()
print("おつかれさまでした  ^-^/")

#print("測定回数--->",count)
#print("測定レート {:.3f} (回数/sec)".format(count/15))
