#!/usr/bin/python3

# motor_by_key1a.py
# 2019-10-30
# Yasushi Honda

import pigpio
import time

MIN_WIDTH=1000
MID_WIDTH=1500
MAX_WIDTH=2000
SLEEP=0.00

class Motor:

   def __init__(self,gpio):
      self.gpio=gpio
      self.pi = pigpio.pi()
      if not self.pi.connected:
         exit()
      self.pi.set_servo_pulsewidth(gpio, MID_WIDTH)
      #time.sleep(0.1)

   def move(self,power):
      self.pi.set_servo_pulsewidth(self.gpio, MID_WIDTH+power)
      time.sleep(SLEEP)    

   def stop(self):
      self.pi.stop()

class Lmotor(Motor):
   def run(self,power):
      self.move(power)

class Rmotor(Motor):
   def run(self,power):
      self.move(-power)
       
if __name__=='__main__':
   import sys
   import termios
   import random

   STEP=10
   MIN=-100
   MAX=100

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

   motorL = Lmotor(17)
   motorR = Rmotor(18)
   
   ch="c"
   print("Input q to stop.")
   motorL.run(0)
   motorR.run(0)
   time.sleep(1)

   left=0
   right=0
   while(ch!="q"):
      try:
         # 書き換えたnewをfdに適応する
         termios.tcsetattr(fd, termios.TCSANOW, new)
         # キーボードから入力を受ける。
         # lfalgsが書き換えられているので、エンターを押さなくても次に進む。echoもしない
         ch = sys.stdin.read(1)
         print(ch)

      finally:
         # fdの属性を元に戻す
         # 具体的にはICANONとECHOが元に戻る
         termios.tcsetattr(fd, termios.TCSANOW, old)

      try:
         if ch == "l" :
            left+= STEP
            right-= STEP

         if ch == "h" :
            left-= STEP
            right+= STEP

         if ch == "f" :
            left+= STEP
            right+= STEP

         if ch == "b" :
            left-= STEP
            right-= STEP

         if left<MIN or left>MAX:
               left = MIN
         motorL.run(left)

         if right<MIN or right>MAX:
               right = MIN
         motorR.run(right)

         time.sleep(SLEEP)

      except KeyboardInterrupt:
         motorL.run(0)
         motorR.run(0)
         break


   print("\nTidying up")
   motorL.run(0)
   motorR.run(0)

   motorL.stop()
   motorR.stop()
