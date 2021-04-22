#!/usr/bin/python3
import motor_by_key1b as mt
import sock4robot_mltpicam1a as sk
import socket
import keyin
import time
import math

if __name__=='__main__':

   a=1.00
   beta=0.10
   alpha=8.0
   b=1
   c=1
   d=10

   gain=0.5
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
   last=now

   vx=0.0
   vy=1.0

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

      now=time.time()
      dt=now-last
      last=now
      try:
         data=cam.recv()
         rate+=1
         #print(data)
         obj_x=float(data[0])+float(data[2])/2
         obj_y=float(data[1])+float(data[3])/2
         obj_w=float(data[2])
         obj_h=float(data[3])
         obj_n=float(data[4])

         # Test
         #left=gain*(obj_x-160)
         #right=-gain*(obj_x-160)
         #left-=(obj_w-100)
         #right-=(obj_w-100)
         pass

         # 2D OV model
         r=100/obj_w
         theta=(160-obj_x)/35
         f = alpha*(math.tanh(beta*(r-b))+c)

         v=math.sqrt(vx*vx+vy*vy)
         nx=math.sin(theta)
         ny=math.cos(theta)

         ovx=(1+math.cos(theta))*f*nx
         ovy=(1+math.cos(theta))*f*ny
         ax=a*(ovx-vx)
         ay=a*(ovy-vy)
         vx_next=vx+dt*ax
         vy_next=vy+dt*ay
         sign=(vx*vy_next-vx_next*vy)/math.fabs(vx*vy_next-vx_next*vy)
         v_next=math.sqrt(vx_next*vx_next+vy_next*vy_next)
         d_theta=sign*math.acos((vx*vx_next+vy*vy_next)/(v*v_next))

         left=v_next + d*d_theta/dt
         right=v_next - d*d_theta/dt
         vx=vx_next
         vy=vy_next

      except (BlockingIOError, socket.error):
         pass


      if now-init>SHOW_PERIOD:
         rate=rate/SHOW_PERIOD
         try:
            print("\r %5.2f %5.2f %8.5f %8.5f %8.5f" % (now-start,rate,r,left,right),end='') 
         except:
            pass
         rate=0
         init=now


      mL.run(left)
      mR.run(right)

      time.sleep(0.01)
      ch=key.read()
     
   mL.run(0)
   mR.run(0)
   print("\n Finished \n")
   
