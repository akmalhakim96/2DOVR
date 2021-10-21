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
import datetime
import platform
import numpy as np

# Pythonファイルインポート 
import picam as PICAM_py         # picamera関係
import modules.motor5a as mt     # モーターを回転させるためのモジュール
import modules.vl53_4a as lidar  # 赤外線レーザーレーダ 3つの場合
#import modules.tof2_3a as lidar # 赤外線レーザーレーダ 2つの場合
import file_read as fr

show_res = 'y'   # モータ出力や距離センサの値を表示する場合は "y"
motor_run = "y"  # モータを回転させる場合は"y"

SLEEP = 0.05
BUS = 1         # bus number
I2C_ADDR = 0x54 # I2Cアドレス
GPIO_L = 17     # 左モーターのgpio 17番
GPIO_R = 18     # 右モーターのgpio 18番
MAX_SPEED = 62  # パーセント
DT = 0.015
dt = DT
THRESHOLD = 0.3 # OVMをon/offするための閾値

def tanh_single(x):
    gamma = 75
    delta = 0.1
    p = 400
    q = 0
    return gamma * (math.tanh(delta*(x-p) + q))


def max_min_adjust(vl,vr):
    if vl > 100:
        vl = 100
    if vl < -100:
        vl = -100

    if vr > 100:
        vr = 100
    if vr < -100:
        vr = -100

    return vl,vr

#  各変数定義
parm_ovm = []
parm_smm = []


#  インスタンス生成
tofL,tofR,tofC=lidar.start() #  赤外線レーザ(3)
#tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
time.sleep(2)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

count = 0
data = []
gamma=0.33 # Center weight

print("#-- #-- #-- #-- #-- #-- #-- #-- #--")

start = time.time()
now = start

vl=0;vr=0
while 1:
    #  実験中
    count = count + 1
    try :
        lidar_distanceL=tofL.get_distance()
        lidar_distanceR=tofR.get_distance()

        vr = tanh_single(lidar_distanceL)
        vl = tanh_single(lidar_distanceR)

        flag = 0
        
        # vl,vrは2次元最適速度モデルで決定される速度
        # tof_l,tof_rは感覚運動写像で決定される速度 

        vl,vr = max_min_adjust(vl,vr)

        if show_res == 'y':
            print("\r %6.2f " % (now-start),end="")
            print(" v_L=%6.2f " % vl, end="")
            print(" v_R=%6.2f " % vr, end="")
            print(" distL=%6.2f " % lidar_distanceL, end="")
            print(" distR=%6.2f " % lidar_distanceR, end="")

        if motor_run == 'y':
            mL.run(vl)
            mR.run(vr)

        time.sleep(DT)
        last = now
        now = time.time()
        dt = now-last
    except KeyboardInterrupt:
        mR.stop()
        mL.stop()
        write_fp.close()
        sys.exit("\nsystem exit ! \n")
mR.stop()
mL.stop()
write_fp.close()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
