#!/usr/bin/python3
# CC BY-SA Yasushi Honda 2020 5/16

import modules.motor as mt # モーターを回転させるためのモジュール
import modules.vl53_3a as lidar
import time
import math

mL=mt.Lmotor(17) # 左モーター(gpio17番)
mR=mt.Rmotor(18) # 右モーター(gpio18番)

tofR,tofL=lidar.start()

a=5.0
beta=10.0
b=300
c=0

while True:
    try:
        distanceR=tofR.get_distance()
        time.sleep(0.02)
        distanceL=tofL.get_distance()
    
        if distanceL>2000:
            distanceL=2000
        if distanceR>2000:
            distanceR=2000
            
        # 交差抑制性結合による感覚運動写像
        #sensorL=a/distanceL
        #sensorR=a/distanceR
        #powerL=c-g*sensorR
        #powerR=c-g*sensorL

        powerL=a*(math.tanh(beta*(distanceR-b))+c)
        powerR=a*(math.tanh(beta*(distanceL-b))+c)

        #mL.run(powerL)
        #mR.run(powerR)
        print("\r %5d %5d %5.2f %5.2f" % (distanceL,distanceR,powerL,powerR),end="")
        time.sleep(0.05)
        
    except KeyboardInterrupt: #例外処理でプログラムを止める
        break

mL.run(0)
mR.run(0)
