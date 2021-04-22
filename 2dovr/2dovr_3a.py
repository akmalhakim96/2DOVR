#!/usr/bin/python3
import motor_by_key1b as mt
import sock4robot_7a as sk
import socket
import keyin
import time
import math

if __name__=='__main__':
   run='n'

   a=4.00
   beta=5.00
   alpha=70.0
   b=0.13  # (m)
   c=0.0
   d=0.10  # (m)

   motor_factor=1.0

   # Camera Property
   max_angl=100
   px_width=320
   theta_factor=max_angl/2*math.pi/(px_width/2*180)
   r_factor=10.0

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

   vx=0.1
   vy=0.1

   key=keyin.Keyboard()
   ch='c'
   left=0
   right=0
   rate=0
   print("  time  rate     r  thta      f       dt    vx    vy")
   while ch!='q':

      if ch=='l':
         left+=STEP 
         right-=STEP 
      if ch=='h':
         left-=STEP 
         right+=STEP 
      if ch=='k':
         left+=STEP 
         right+=STEP 
      if ch=='j':
         left-=STEP 
         right-=STEP 

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

         # 2D OV model
         r=r_factor/obj_w
         f = alpha*(math.tanh(beta*(r-b))+c)

         theta=(px_width/2-obj_x)*theta_factor

         v=math.sqrt(vx*vx+vy*vy)
         theta_v=math.acos(vx/v)
         nx=math.cos(theta+theta_v)
         ny=math.sin(theta+theta_v)

         ovx=(1+math.cos(theta))*f*nx
         ovy=(1+math.cos(theta))*f*ny
         ax=a*(ovx-vx)
         ay=a*(ovy-vy)
         vx_next=vx+dt*ax
         vy_next=vy+dt*ay
         v_next=math.sqrt(vx_next*vx_next+vy_next*vy_next)
         try:
            #sign=(vx*vy_next-vx_next*vy)/math.fabs(vx*vy_next-vx_next*vy)
            ext=vx*vy_next-vx_next*vy
            d_theta=math.asin(ext/(v*v_next))

            left=(v_next - d*d_theta/dt)*motor_factor
            right=(v_next + d*d_theta/dt)*motor_factor
            vx=vx_next
            vy=vy_next
         except:
            pass

      except (BlockingIOError, socket.error):
         pass


      if now-init>SHOW_PERIOD:
         rate=rate/SHOW_PERIOD
         try:
            print("\r %5.2f %5.2f %5.2f %5.2f %6.2f %8.5f %5.1f %5.1f" 
                      % (now-start,rate,r,theta,f,dt,vx,vy),end='') 
         except:
            pass
         rate=0
         init=now

      if run=='y':
         mL.run(left)
         mR.run(right)

      time.sleep(0.02)
      ch=key.read()
     
   mL.run(0)
   mR.run(0)
   print("\n Finished \n")
   
