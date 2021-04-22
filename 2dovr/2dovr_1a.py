#!/usr/bin/python3
import motor_by_key1b as mt
import sock4robot_mltpicam1a as sk
import socket
import keyin
import time

if __name__=='__main__':

   gain=0.04
   STEP=10
   SHOW_PERIOD=0.2

   mL=mt.Lmotor(17)
   mR=mt.Rmotor(18)
   mL.run(0)
   mR.run(0)
   print('mL & mR ok')

   cam=sk.UDP_Recv(sk.ROBOT_ADDR,sk.PICAM_PORT)
   data=[]

   now=time.time()
   start=now
   init=now

   key=keyin.Keyboard()
   ch='c'
   left=0
   right=0
   rate=0
   while ch!='q':

      if ch=='l':
         left+=STEP 
         right-=STEP 
      if ch=='h':
         left-=STEP 
         right+=STEP 

      try:
         data=cam.recv()
         #print(data)
         rate+=1
         obj_x=float(data[0])+float(data[2])/2
         obj_y=float(data[1])+float(data[3])/2
         obj_w=float(data[2])
         obj_h=float(data[3])
         left+=gain*(obj_x-160)
         right-=gain*(obj_x-160)
      except (BlockingIOError, socket.error):
         pass

      now=time.time()
      if now-init>SHOW_PERIOD:
         rate=rate/SHOW_PERIOD
         try:
            print("\r %5.2f %5.2f %5.2f %5.2f" % (now-start,rate,obj_x,obj_w),end='') 
         except:
            pass
         rate=0
         init=now

      mL.run(left)
      mR.run(right)

      #time.sleep(0.01)
      ch=key.read()
     
   mL.run(0)
   mR.run(0)
   print("\n Finished \n")
   
