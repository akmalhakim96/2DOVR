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
#  Pythonファイルインポート 
import ovm as OVM_py          # 2次元最適速度モデル関係
import picam as PICAM_py # picamera関係
import modules.motor5a as mt         #  (改良版)モーターを回転させるためのモジュール
import modules.vl53_4a as lidar     #  赤外線レーザーレーダ 3つの場合
#import modules.tof2_3a as lidar      #  赤外線レーザーレーダ 2つの場合

#sokcet 通信関係 
import file_read as fr

select_hsv = "n"
show_res = 'n'
motor_run = "y"
imshow = "n"

SLEEP = 0.01
EX_TIME = 3     # (min)
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
PARM_SMM = "/home/pi/2DOVR/parm_smm.csv" 
FRAME_SIZE = "/home/pi/2DOVR/framesize.csv"

#  物体未認識時のhyperbolic-tan
def tanh1(x,list):
    alpha=float(list[0])  # 0.0
    alpha2=float(list[1]) # 1.0
    beta=float(list[2])   # 0.004
    beta2=float(list[3])  # 1000
    b=float(list[4])      # 0.4
    c=float(list[6])      # 0.0
    f=(alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c) / (alpha + alpha2 + c)
    return f

def tanh2(x,list):
    alpha=float(list[0])  # 0.0
    alpha2=float(list[1]) # 1.0
    beta=float(list[2])   # 0.004
    beta2=float(list[3])  # 1000
    b=float(list[5])      # 0.6
    c=float(list[6])      # 0.0
    f=(alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c) / (alpha + alpha2 + c)
    return f

#  各変数定義
parm_ovm = []
parm_smm = []

ex_start_time = datetime.datetime.now()
ex_start_time = str(ex_start_time.strftime('%Y%m%d%H%M%S'))
ex_start_time = ex_start_time.replace("'",'')
ex_start_time = ex_start_time.replace(" ",'')

hostname = '[%s]' % platform.uname()[1]
hostname = hostname.replace("[",'')
hostname = hostname.replace("]",'')

write_file = str(hostname) + "-" +str(ex_start_time) + ".txt"

write_fp = open("/home/pi/2DOVR/result/"+write_file,"w")
write_fp.write("#"+hostname+"\n")

write_fp.write("time, ")
write_fp.write("rate , ")
write_fp.write("sleep"+str(DT)+",")
write_fp.write("\n")

#  パラメータ読み込み
parm_ovm = fr.read_parm(PARM_OVM)
parm_smm = fr.read_parm(PARM_SMM)
upper,lower = fr.read_framesize(FRAME_SIZE)

print("smm-parm")
print("# alpha",end="")
print("  alpha2",end="")
print("  beta",end="")
print("  beta2",end="")
print("      b1",end="")
print("      b2",end="")
print("     c",end="")
print("    THRESHOLD")

print("%7.3f" % parm_smm[0],end="")
print("%7.3f" % parm_smm[1],end="")
print("%7.3f" % parm_smm[2],end="")
print("  %7.3f" % parm_smm[3],end="")
print("%7.3f" % parm_smm[4],end="")
print("%7.3f" % parm_smm[5],end="")
print("%7.3f" % parm_smm[6],end="")
print("%7.3f" % THRESHOLD)
print(len(parm_smm))
print("\nparm-OV")


#  インスタンス生成
ovm = OVM_py.Optimal_Velocity_class(parm_ovm)         #  2次元最適速度モデル関係
print(parm_ovm)
tofL,tofR,tofC=lidar.start() #  赤外線レーザ(3)
#tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
time.sleep(2)
picam =PICAM_py.PI_CAMERA_CLASS(upper,lower) 
print("picamera 接続完了\n")
time.sleep(2)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

count = 0
data = []
gamma=0.33 # Center weight

print("#-- #-- #-- #-- #-- #-- #-- #-- #--")

if select_hsv=='y':
    lower_light,upper_light=picam.calc_hsv()
else:
    #Red Cup H:S:V=3:140:129
    # h,s,v = 171,106,138
    # 177  139  141 2021/06/01  電気OFF
    # 172  160  148 2021/06/15  電気ON
    # 179  116  101 2021/06/15  電気ON
    # 172  164  152 2021/06/22  電気ON
    # 175  153  152 2021/06/24  電気ON
    # 174   97  126 2021/08/06  電気ON,ピンクのテープ貼付
    # 174  124  136 2021/08/13  電気ON,ピンクのテープ貼付

    H = 174; S = 124; V =136 
    h_range = 20; s_range = 80; v_range = 80 # 明度の許容範囲
    lower_light = np.array([H-h_range, S-s_range, V-v_range])
    upper_light = np.array([H+h_range, S+s_range, V+v_range])
start = time.time()
now = start
rate_start = start
count = 0

key=cv2.waitKey(1)
vl=0;vr=0
#while key!=ord('q'):
while now - start < 15:
    #  実験中
    dist,theta,frame = picam.calc_dist_theta(lower_light, upper_light)
    count = count + 1
    try :
        lidar_distanceL=tofL.get_distance()/1000
        if lidar_distanceL>2:
            lidar_distanceL=2

        lidar_distanceC=tofC.get_distance()/1000
        if lidar_distanceC>2:
            lidar_distanceC=2
           
        lidar_distanceR=tofR.get_distance()/1000
        if lidar_distanceR>2:
            lidar_distanceR=2

        if lidar_distanceL>0 and lidar_distanceC>0:
            areaL=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceL))
        if lidar_distanceR>0 and lidar_distanceC>0:
            areaR=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceR))

        tof_r = tanh1(areaL,parm_smm)
        tof_l = tanh2(areaR,parm_smm)
        #print(tof_r,tof_l)
        flag = 0
        
        if areaL > THRESHOLD and areaR > THRESHOLD:
            if dist == None:
                dist = float(2)
                theta = 0.0
                vl, vr, omega = ovm.calc(dist,theta,dt)
            else:
                dist = float(dist)
                # pixyカメラで物体を認識している時
                vl, vr, omega = ovm.calc(dist,theta,dt)
        else:
            if dist == None:
                dist=float(2)
                theta=0.0
                vl = 1.0
                vr = 1.0
            else:
                dist = float(dist)
                vl = 1.0
                vr = 1.0
        vl = vl * tof_l * MAX_SPEED 
        vr = vr * tof_r * MAX_SPEED

        if vl < -100:
            vl = -100
        if vl > 100:
            vl = 100

        if vr < -100:
            vr = -100
        if vr > 100:
            vr = 100



        if show_res == 'y':
            print("\r %6.2f " % (now-start),end="")
            print(" dist=%6.2f " % dist, end="")
            #print(" theta=%6.2f " % theta, end="")
            #print(" v_L=%6.2f " % vl, end="")
            #print(" v_R=%6.2f " % vr, end="")
            #print(" dL=%6.2f " % lidar_distanceL, end="")
            #print(" dC=%6.2f " % lidar_distanceC, end="")
            #print(" dR=%6.2f " % lidar_distanceR, end="")
            print(" areaL=%6.2f " % areaL, end="")
            print(" areaR=%6.2f " % areaR, end="")

        if motor_run == 'y':
            mL.run(vl)
            mR.run(vr)

        if imshow == 'y':    
            cv2.imshow("frame",frame)
            key=cv2.waitKey(1)

        #time.sleep(DT)
        last = now
        now = time.time()
        if now - rate_start > 1.0000000000:
            write_fp.write(str('{:.6g}'.format(now-start))+", ")
            write_fp.write(str(int(count)))
            write_fp.write("\n")
            count = 0
            rate_start = now
        count = count + 1
        dt = now-last
    except KeyboardInterrupt:
        mR.stop()
        mL.stop()
        write_fp.close()
        sys.exit("\nsystem exit ! \n")
mR.stop()
mL.stop()
#write_fp.close()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
