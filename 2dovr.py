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
import datetime
import platform
import numpy as np

# Pythonファイルインポート 
import ovm as OVM_py             # 2次元最適速度モデル関係
import picam as PICAM_py         # picamera関係
import modules.motor5a as mt     # モーターを回転させるためのモジュール
import modules.vl53_4a as lidar  # 赤外線レーザーレーダ 3つの場合
#import modules.tof2_3a as lidar # 赤外線レーザーレーダ 2つの場合
import file_read as fr

select_hsv = "n" # 画面上で対象物を選択する場合は"y"
show_res = 'y'   # モータ出力や距離センサの値を表示する場合は "y"
motor_run = "y"  # モータを回転させる場合は"y"
imshow = "y"     # カメラが捉えた映像を表示する場合は"y"

# 弾性散乱のための変数
TURN_TIME=0.3
TURN_POWER=100

SLEEP = 0.05
BUS = 1         # bus number
I2C_ADDR = 0x54 # I2Cアドレス
GPIO_L = 17     # 左モーターのgpio 17番
GPIO_R = 18     # 右モーターのgpio 18番
MAX_SPEED = 62  # パーセント
DT = 0.015
dt = DT
THRESHOLD = 0.2 # OVMをon/offするための閾値
EX_TIME = 5 

#  パラメータ記載のファイルの絶対パス
PARM_OVM = "/home/pi/2DOVR/parm_ovm.csv" 
FRAME_SIZE = "/home/pi/2DOVR/framesize.csv"

def motor_out_adjust(vl,vr):
    if vl > 100:
        vl = 100
    if vl < -100:
        vl = -100

    if vr > 100:
        vr = 100
    if vr < -100:
        vr = -100

    return vl,vr

def tof_get_dist(tofL,tofC,tofR):
    
    disL=tofL.get_distance()/1000
    disC=tofC.get_distance()/1000
    disR=tofR.get_distance()/1000

    if distL > 2:
        distL = 2

    if distC > 2:
        distC = 2

    if distR > 2:
        distR = 2

    return distL,distC,distR

def synergistic_dist(distL,distC,distR,gamma):
    areaL=math.exp(gamma*math.log(distC))*math.exp((1-gamma)*math.log(distL))
    areaR=math.exp(gamma*math.log(distC))*math.exp((1-gamma)*math.log(distR))
    return areaL, areaR

#  各変数定義
parm_ovm = []

#  パラメータ読み込み
parm_ovm = fr.read_parm(PARM_OVM)
upper,lower = fr.read_framesize(FRAME_SIZE)

#  インスタンス生成
ovm = OVM_py.Optimal_Velocity_class(parm_ovm)         #  2次元最適速度モデル関係
tofL,tofR,tofC=lidar.start() #  赤外線レーザ(3)
#tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
time.sleep(1)
picam =PICAM_py.PI_CAMERA_CLASS(upper,lower) 
print("picamera 接続完了\n")
time.sleep(1)
print("\nparm-OV")
print(" vs,   a, alpha,beta, b,  c,  None")
print(parm_ovm)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

data = []
gamma=0.33 # Center weight

print("#-- #-- #-- #-- #-- #-- #-- #-- #--")

if select_hsv=='y':
    lower_light,upper_light=picam.calc_hsv()
else:
    #Red Cup H:S:V=3:140:129
    # h,s,v = 171,106,138
    #H = 171; S = 110; V =215
    # H,S,V = 173,110,215 21/10/26 VRシアター
    H = 173; S = 99; V =200
    h_range = 10; s_range = 50; v_range = 50 # 明度の許容範囲
    hL = H - h_range
    hU = H + h_range
    sL = S - s_range
    sU = S + s_range
    vL = V - v_range
    vU = V + v_range
    if sL < 0:
        sL = 0
    if sU > 255:
        sU = 255
    if vL < 0:
        vL = 0
    if vU > 255:
        vU = 255
    lower_light = np.array([hL, sL, vL])
    upper_light = np.array([hU, sU, vU])

start = time.time()
now = start

key=cv2.waitKey(1)
vl=0;vr=0
while key!=ord('q'):
    dist,theta,frame = picam.calc_dist_theta(lower_light, upper_light)
    if dist==None:
        dist=2.0
        theta=0.0
    try :
        distanceL,distanceC,distanceR = tof_get_dist(tofL,tofC,tofR)

        if distanceL > 0 and distanceC > 0 and distanceR > 0:
           areaL, areaR = synergistic_dist(distanceL, distanceC, distanceR)

        if areaL <= THRESHOLD or areaR <= THRESHOLD:
            if areaL<areaR:
                mL.run(TURN_POWER)
                mR.run(-TURN_POWER)
                time.sleep(TURN_TIME)
            else:
                mL.run(-TURN_POWER)
                mR.run(TURN_POWER)
                time.sleep(TURN_TIME)
        else:
            vl, vr, omega = ovm.calc(dist,theta,dt)

        vl = vl * MAX_SPEED 
        vr = vr * MAX_SPEED

        vl,vr = motor_out_adjust(vl,vr)

        if show_res == 'y':
            print("\r %6.2f " % (now-start),end="")
            #print(" dist=%6.2f " % dist, end="")
            #print(" theta=%6.2f " % theta, end="")
            #print(" v_L=%6.2f " % vl, end="")
            #print(" v_R=%6.2f " % vr, end="")
            print(" dL=%6.2f " % distanceL, end="")
            print(" dC=%6.2f " % distanceC, end="")
            print(" dR=%6.2f " % distanceR, end="")
            #print(" areaL=%6.2f " % areaL, end="")
            #print(" areaR=%6.2f " % areaR, end="")

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
        sys.exit("\nsystem exit ! \n")
mR.stop()
mL.stop()
print()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
