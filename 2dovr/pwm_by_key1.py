#!/usr/bin/python
# -*- coding: utf8 -*-

# pwm_by_key1.py
# 2017-10-18
# Yasushi Honda

# How to execute
# sudo pigpiod
# pwm_by_key1.py 

import sys
import termios
import time
import random
import pigpio

GAIN=0.2

SLEEP=0.1

NUM_GPIO=32

MIN_WIDTH=1000
MID_WIDTH=1500
MAX_WIDTH=2000

STEP=10

width = [0]*NUM_GPIO

#標準入力のファイルディスクリプタを取得
fd = sys.stdin.fileno()

#fdの端末属性をゲットする
#oldとnewには同じものが入る。
#newに変更を加えて、適応する
#oldは、後で元に戻すため
old = termios.tcgetattr(fd)
new = termios.tcgetattr(fd)

#new[3]はlflags
#ICANON(カノニカルモードのフラグ)を外す
new[3] &= ~termios.ICANON
#ECHO(入力された文字を表示するか否かのフラグ)を外す
new[3] &= ~termios.ECHO

pi = pigpio.pi()

if not pi.connected:
   exit()

G=[17,18] # list of gpio number that servo are connected to
   
ch="c"
print "Input q to stop."
width[17]=MID_WIDTH
width[18]=MID_WIDTH
while(ch!="q"):
   try:
       # 書き換えたnewをfdに適応する
       termios.tcsetattr(fd, termios.TCSANOW, new)
       # キーボードから入力を受ける。
       # lfalgsが書き換えられているので、エンターを押さなくても次に進む。echoもしない
       ch = sys.stdin.read(1)
       print ch

   finally:
       # fdの属性を元に戻す
       # 具体的にはICANONとECHOが元に戻る
       termios.tcsetattr(fd, termios.TCSANOW, old)

   try:

      if ch == "l" :
         width[17]+= STEP
         width[18]+= STEP

      if ch == "h" :
         width[17]-= STEP
         width[18]-= STEP

      if ch == "f" :
         width[17]-= STEP
         width[18]+= STEP

      if ch == "b" :
         width[17]+= STEP
         width[18]-= STEP

      for g in G:
         if width[g]<MIN_WIDTH or width[g]>MAX_WIDTH:
            width[g] = MIN_WIDTH
         pi.set_servo_pulsewidth(g, width[g])

      time.sleep(SLEEP)

   except KeyboardInterrupt:
      for g in G:
         pi.set_servo_pulsewidth(g, MID_WIDTH)
      break


print("\nTidying up")
for g in G:
   pi.set_servo_pulsewidth(g, MID_WIDTH)

pi.stop()
