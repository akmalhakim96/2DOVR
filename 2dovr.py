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
THRESHOLD = 0.3 # OVMをon/offするための閾値

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

def tof_adjust(distL,distC,distR):
    if distL > 2:
        distL = 2

    if distC > 2:
        distC = 2

    if distR > 2:
        distR = 2

    return distL,distC,distR

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
time.sleep(2)
picam =PICAM_py.PI_CAMERA_CLASS(upper,lower) 
print("picamera 接続完了\n")
time.sleep(2)
print("\nparm-OV")
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
    H = 172; S = 107; V =148
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
    if dist==None:
        dist=2.0
        theta=0.0
    try :
        distanceL=tofL.get_distance()/1000

        distanceC=tofC.get_distance()/1000
           
        distanceR=tofR.get_distance()/1000

        distanceL,distanceC,distanceR = tof_adjust(distanceL,distanceC,distanceR)

        if distanceL>0 and distanceC>0:
            areaL=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceL))
        if distanceR>0 and distanceC>0:
            areaR=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceR))

        # vl,vrは2次元最適速度モデルで決定される速度

        if areaL > THRESHOLD and areaR > THRESHOLD:
           vl, vr, omega = ovm.calc(dist,theta,dt)
        else:
            if areaL<areaR:
                mL.run(TURN_POWER)
                mR.run(-TURN_POWER)
                time.sleep(TURN_TIME)
            else:
                mL.run(-TURN_POWER)
                mR.run(TURN_POWER)
                time.sleep(TURN_TIME)
            
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
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
