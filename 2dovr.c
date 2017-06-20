// このプログラムをコンパイルして(2dovr)ロボットの/tmpに置く．
// ベースLinux(BASSE)のプログラムbasseからspawnされる．

#include "pvm_4frobo.c" // pvm_sendなどの関数。HAUTE,BASSEなどマシン名もここに定義してある。
#include "mpu9150.c" // 加速度、ジャイロセンセーのための関数など

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <BBB.h>  // BBBでpwm出力するためのライブラリ

#define gain_objx 2.500 // 2.0 for throttle 300
#define gain_objy 3.700 // 3.0 for 300
#define gain_objdy 0 // 0
#define gain_objz 0 //0

#define OBJ_XOS 170 // 170
#define OBJ_WOS 30  //  40
#define OBJ_YOS 80  //  80

#define PULS_ZERO_LEFT 1480000  // 1480000 ns for servo zero
#define PULS_ZERO_RIGHT 1460000  // 1420000 ns for servo zero
#define PULS_UNIT 1000     // usec.  -500 < throttle < +500

#define SRF02_MAX 600
#define PULS4OBJ_Y_MUG 4.20 // 4.2

// calibration factor
#define ML_CALIB 0.8000 // 
#define MR_CALIB 0.9999 // 
#define ML_OFFSET -0*PULS_UNIT // 0--500
#define MR_OFFSET -0*PULS_UNIT //

// For frb18.0 anisotorpy factor
#define anstpy 1.00 // 2400mAh 0.75

// Reverse avoid factor (2016 3.5)
#define MMRY_N 32 // to keep memmory of height etc
#define REV_AVO_FAC 0.00 // ex 0.70 for GWS8040mod


#define N_INT 256 // the number of integrating values of gyro sensor (2016.2.18)

#define INT_FACTOR 5.50 // ジャイロ値を積分して加速度値に合わせるための定数(2016.2.18)
#define TANH_FAC 10.0

#define PI acos(-1.0)

#define SENSORS_OUTPUT "/tmp/sensors.xy"
#define MOTORS_OUTPUT "/tmp/motors.xy"

#define INIT_RECV_SRF02 2.0

#define DATASEND_PERIOD 0.20 // ex. 0.2sec

#define HART_L "/sys/class/gpio/gpio45/value"
#define HART_R "/sys/class/gpio/gpio44/value"

// Prototype of functions
// integrates values of gyro sensor to angle
struct threed_type IntegGyro(struct threed_type value, struct threed_type *memory_array);
// integrates values of accel sensor to velocity
struct threed_type IntegTrans(struct threed_type value, struct threed_type *memory_array);


struct threed_type norm_angl(struct threed_type, struct threed_type);
struct threed_type norm_gyro(struct threed_type, struct threed_type);

void pwm_init(BBB_pwm *pwm);
void pwm_final(BBB_pwm *pwm); 

int Blink(char *file);
int Blink_H();

// Function prototype
// memmory a value to memory_array
void mmry(double value, double *memory_array);

double average(double *memory_array);
 
void DriveLeftMotor(BBB_pwm *m, int power); // power : 0--500
void DriveRightMotor(BBB_pwm *m, int power); // power : 0--500


int main(void) {
   int    i;
   int    throttle,rL,rR;
   int    rL_calib,rR_calib;
   int    keyval=0;
   int    flag_height, flag_srf02, flag_pixy, flag_feeler;
   int    round_distance[10];
   int    indc_dist; // Indicator of distance
   int    mpu6050_fd, ak8975_fd;   
   int    hmc6352_fd;   
   int    puls4obj_x, puls4obj_y, puls4obj_dy, puls4obj_z;
   int    basse_stat;
   int    nhost,narch,infos;
   int    rate_counter;
   int    feeler_LO, feeler_LI, feeler_RO, feeler_RI;
   struct pvmhostinfo *hostp;
   struct obj_type obj1, obj2;

   unsigned char rbuf[6];
   unsigned char wbuf[2];

   double dw;
   double data[256];
   double init_sec, start_sec, now_sec, fstart_sec, rate_start_sec;
   double normal_throttle;
   double sensitiv;
   double accel_mmry[MMRY_N],accel2_mmry[MMRY_N];
   double gyro_mmry[MMRY_N];
   double react_L_sec,react_R_sec,react_F_sec;
   double react_L,react_R,react_F;

   struct threed_type accel,accel2;    
   struct threed_type accel_last,accel2_last;    
   struct threed_type gyro,gyro2;    
   struct threed_type gyro_last,gyro2_last;    
   struct threed_type accel_ave,accel2_ave;    
   struct threed_type gyro_ave,gyro2_ave;    
   struct threed_type offset_accel;
   struct threed_type angl;    
   struct threed_type angl_ave;
   struct threed_type last_angl;
   struct threed_type diff_angl;

   float offset_direction;

   // For making result filename
   time_t timer;
   char str[128];
   char gain_s[128];
   char *file_name;
   FILE *fp,*mfp;

   fp=fopen(SENSORS_OUTPUT,"a");
   mfp=fopen(MOTORS_OUTPUT,"a");

   setlinebuf(stdout);   

   printf("\n"); 
   printf("XX=====================XX\n"); 
   printf("X    2dOVR B1 start!   X\n"); 
   printf("XX==============(^.^)/=XX\n"); 

   pvm_config(&nhost,&narch,&hostp);
   for(i=0;i<nhost;i++){
      printf("host name=%s\n",hostp[i].hi_name);
   }

  
   struct threed_type compass; 
   long tempData[1];       

   float direction;


   // PWM prepare
   BBB_pwm *mL = BBB_open_pwm("P9_14");
   pwm_init(mL);
   printf("mL initialized \n");

   BBB_pwm *mR = BBB_open_pwm("P8_19");
   pwm_init(mR);
   printf("mR initialized \n");

   //throttle=ZERO;
   DriveLeftMotor(mL,0);
   DriveRightMotor(mR,0);

/*
   // MPU6050 （加速度　＆　ジャイロセンサ）
   if ((mpu6050_fd = open(ACCEL_I2C, O_RDWR)) < 0){
      printf("Faild to open MPU6050 port.\n");
      exit(1);
   }
   MPU6050_init(mpu6050_fd,MPU6050_ADDR);
*/

   /*
   // HMC6352 （コンパス）
   if ((hmc6352_fd = open(I2C_HMC6352, O_RDWR)) < 0){
      printf("Faild to open HMC6352 port.\n");
      exit(1);
   }
   HMC6352_init(hmc6352_fd);
   */

   pvm_init_4haute();

   printf("Data recieve from basse starts\n");
   start_sec=gettimeofday_sec();
   init_sec=gettimeofday_sec();

   data[0]=2.78; // Sending 2.78 as signal of starting main loop
                 // The value has NO meaning.
   pvm_send_from_haute_to_basse(data,1);
   printf("Start signal(%lf) is sent to basse.\n",data[0]);


   // Blocking receive of Keyboard value.
   while(pvm_recv_from_basse_to_haute(data,1)==0){}
   printf("%lf\n",data[0]);
   keyval=(int)data[0];
   while(keyval!=-999){

      // Gains must be recieved from basse
      while(pvm_recv_from_basse_to_haute(data,2)<=0){}
      sensitiv=data[0]; 
      normal_throttle=data[1]; 

      printf("======================\n");
      printf("sensitivity=%lf\n",sensitiv);
      printf("normal_throttle=%lf\n",normal_throttle);
      printf("======================\n");
      
      //MPU6050_init(mpu6050_fd,MPU6050_ADDR);

      pvm_spawn_pixy();
      data[0]=(double)OBJ_XOS;
      data[1]=(double)OBJ_WOS;
      data[2]=(double)OBJ_YOS;
      pvm_send_from_haute_to_pixy(data,3);

      pvm_spawn_feeler();

      throttle=0;
      rate_counter=0;

      rate_start_sec=gettimeofday_sec();
      start_sec=gettimeofday_sec();
      init_sec=gettimeofday_sec();

      time(&timer);
      fprintf(fp,"\n# %s",ctime(&timer));

      fprintf(mfp,"\n# %s",ctime(&timer));
      
      react_L_sec=0.0;
      react_R_sec=0.0;
      react_F_sec=0.0;
      while(throttle>=0){

         // pvm_recv(diff)
         // data[0]:Increment of throttle value send from basse
         if(pvm_recv_from_basse_to_haute(data,1)>0){
            throttle+=(int)data[0];
            //printf("%d\n",throttle);
            rL=throttle;
            rR=throttle;
         }
      
         //Blink_H();

         // Receive position data from PIXY camera via PVM
         flag_pixy=pvm_recv_from_pixy_to_haute(data,12);
         if(flag_pixy>0){
            obj1.signature=(int)data[0];
            obj1.x  =(int)data[1];
            obj1.y  =(int)data[2];
            dw      =(int)data[3];
            obj1.width =(int)data[4];
            obj1.height =(int)data[5];
            puls4obj_x=gain_objx*(obj1.x-OBJ_XOS); 
            puls4obj_y=gain_objy*(obj1.width-OBJ_WOS);  
            if(obj1.width-OBJ_WOS<0){
               puls4obj_y=PULS4OBJ_Y_MUG*puls4obj_y;
            }
            puls4obj_dy=gain_objdy*dw;  
            puls4obj_z=-gain_objz*(obj1.y-OBJ_YOS); 
            //printf("%3d %5d %5d %5d %5d \n",obj_sig,obj_x,obj_y,obj_w,obj_h);
         }
         // To follow object
         rL=throttle+puls4obj_x;
         rR=throttle-puls4obj_x;
         rL-=puls4obj_y;
         rR-=puls4obj_y;


         // Receive feeler data from FEELER via PVM
         flag_feeler=pvm_recv_from_feeler_to_haute(data,4);
         if(flag_feeler>0){
            feeler_LO=(int)data[0];
            feeler_LI=(int)data[1];
            feeler_RO=(int)data[2];
            feeler_RI=(int)data[3];
            //printf("%d %d %d %d\n",feeler_LO,feeler_LI,feeler_RO,feeler_RI);
         }
         // To react wall (Left)
         now_sec=gettimeofday_sec(); // Mesure second for now
         if(now_sec-react_L_sec<0.7){
            // turn right back
            rL=-throttle*0.8;
            rR=-throttle*0.8;
            HartOn(HART_L);
         }else if(now_sec-react_L_sec<2.0){
            rL=+throttle*0.8;
            rR=-throttle*0.8;
            //Blink(HART_L);
            HartOn(HART_L);
         }else{   
            if(feeler_LO>0||feeler_LI>0){ 
               react_L_sec=gettimeofday_sec(); 
               react_R_sec=0.0;
            }
            //Blink_H();
            //HartOff(HART_L);
         }

         // To react wall (Right)
         now_sec=gettimeofday_sec(); // Mesure second for now
         if(now_sec-react_R_sec<0.7){
            // turn left back
            rL=-throttle*0.8;
            rR=-throttle*0.8;
            //Blink(HART_R);
            HartOn(HART_R);
         }else if(now_sec-react_R_sec<2.0){
            rL=-throttle*0.8;
            rR=+throttle*0.8;
            //Blink(HART_R);
            HartOn(HART_R);
         }else{   
            if(feeler_RO>0||feeler_RI>0){ 
               react_R_sec=gettimeofday_sec(); 
               react_L_sec=0.0;
            }
            //Blink_H();
            //HartOff(HART_R);
         }

         // Print RATE
         rate_counter++;
         if(now_sec-rate_start_sec>0.2){
            rate_start_sec=gettimeofday_sec();
            printf("%5.2f %5d %3d %3d %3d\n",now_sec-init_sec,rate_counter,rL,rR,puls4obj_y);
            rate_counter=0;
         }

         if(now_sec-start_sec>DATASEND_PERIOD){
            // Compus read
            //direction=HMC6352_read(hmc6352_fd)/10.0-180.0;

            start_sec=gettimeofday_sec();

            data[0]=accel2_ave.z;
            data[1]=react_R;
            data[2]=gyro.x;
            data[3]=gyro.x-gyro_last.x;
            data[4]=(double)obj1.x;
            data[5]=(double)obj1.y;
            data[6]=(double)obj1.width;
            data[7]=(double)obj2.width;
            pvm_send_from_haute_to_basse(data,8);
            // it had better to commen out this pvm_send in order to decrease time delay in wireless LAN

            // Output to SENSORS_OUTPUT
            fprintf(fp,"%5.2f ",now_sec-init_sec); // #1
            fprintf(fp,"%7.3f %7.3f %7.3f %7.3f\n",obj1.x,obj1.y,obj1.width,obj1.height);

            // Output to MOTORS_OUTPUT
            fprintf(mfp,"%5.2f ",now_sec-init_sec);
            fprintf(mfp,"%6.4f %6.4f ", (float)rL_calib/PULS_UNIT, (float)rR_calib/PULS_UNIT); 
                

            // 8 USsensors 0--7 are selected by this indicator
            if(indc_dist==7)indc_dist=0;
            else indc_dist++;
         
         }  // end of DATASEND_PRIOD if


         // pwm_output(motors)
         if(throttle>0){
            DriveLeftMotor(mL,rL);
            DriveRightMotor(mR,rR);
         }else{
            DriveLeftMotor(mL,0);
            DriveRightMotor(mR,0);
         }


      }// end of throttle loop
      //pvm_kill(srf02_tid);
      pvm_kill(pixy_tid);
      pvm_kill(feeler_tid);

      if(flag_height==1){
         now_sec=gettimeofday_sec();
         printf("# flight sec=%5.1f\n",now_sec-fstart_sec);
         fprintf(fp,"# flight sec=%5.1f\n",now_sec-fstart_sec);
         data[0]=now_sec-fstart_sec;
         pvm_send_from_haute_to_basse(data,1);
      }else{
         data[0]=0.0;
         pvm_send_from_haute_to_basse(data,1);
      }


      while(pvm_recv_from_basse_to_haute(data,1)==0){}
      keyval=(int)data[0];
      //printf("keyval=%d\n",keyval);
   }// end of keyval loop
 

   // fprintf(gains)
  
   pwm_final(mL);
   pwm_final(mR);

   //MPU9150_final(mpu6050_fd);
   //close(mpu6050_fd);

   fclose(fp);
   fclose(mfp);

   return 0;
} // end of main

void DriveLeftMotor(BBB_pwm *m, int power){
   int pwm_puls;

   pwm_puls=PULS_ZERO_LEFT+(int)(power*ML_CALIB*PULS_UNIT)+ML_OFFSET;

   // pwm_output(motors)
   m->set_duty(m,pwm_puls);
} // end of DriveLeftMotor

void DriveRightMotor(BBB_pwm *m, int power){
   int pwm_puls;

   pwm_puls=PULS_ZERO_RIGHT-(int)(power*MR_CALIB*PULS_UNIT)-MR_OFFSET;

   // pwm_output(motors)
   m->set_duty(m,pwm_puls);
} // end of DriveLeftMotor

struct threed_type IntegGyro(struct threed_type value, struct threed_type *memory_array){ 
// integrates values of gyro sensor

   int i;
   struct threed_type integ;

   // Update short range memory of gyro values
   i=0;
   while(i<N_INT-1){
      memory_array[i].x=memory_array[i+1].x;
      memory_array[i].y=memory_array[i+1].y;
      memory_array[i].z=memory_array[i+1].z;
      i++;
   }
   memory_array[N_INT-1].x=value.x;
   memory_array[N_INT-1].y=value.y;
   memory_array[N_INT-1].z=value.z;

   // Calulate integration to get angle
   integ.x=0.0; integ.y=0.0; integ.z=0.0; 
   i=0;
   while(i<N_INT-1){
      integ.x += memory_array[i].x;
      integ.y += memory_array[i].y;
      integ.z += memory_array[i].z;
      i++;
   }
   integ.x=-INT_FACTOR*integ.x/N_INT;
   integ.y=-INT_FACTOR*integ.y/N_INT;
   integ.z=-INT_FACTOR*integ.z/N_INT;

   return integ;

}


struct threed_type norm_angl(struct threed_type data, 
                             struct threed_type offset){

   struct threed_type angl;

   // normalize and translate sensor axis to frobo axis

   angl.x= data.x*ACCEL_YNORM-offset.x;
   angl.x=angl.x;
   if(angl.x>1.0){
      printf("angl.x=%f\n",angl.x);
      angl.x=1.0;
   }
   if(angl.x<-1.0){
      printf("angl.x=%f\n",angl.x);
      angl.x=-1.0;
   }
   angl.x=asin(angl.x);

   angl.y= data.y*ACCEL_XNORM-offset.y;
   angl.y=angl.y;
   if(angl.y>1.0){
      printf("angl.y=%f\n",angl.y);
      angl.y=1.0;
   }
   if(angl.y<-1.0){
      printf("angl.y=%f\n",angl.y);
      angl.y=-1.0;
   }
   angl.y=-asin(angl.y);

   angl.z= data.z*ACCEL_ZNORM;
   if(angl.z>1.0){angl.z=1.0;}
   if(angl.z<-1.0){angl.z=-1.0;}
   angl.z=asin(angl.z);

   return angl;

}
 
struct threed_type norm_gyro(struct threed_type data, struct threed_type offset){

   struct threed_type gyro;
   gyro.x= data.x*GYRO_NORM-offset.x;
   gyro.y= data.y*GYRO_NORM-offset.y;
   gyro.z= data.z*GYRO_NORM-offset.z;


   return gyro;

}

void pwm_init(BBB_pwm *pwm) {
  if(pwm == NULL){
    printf("open pwm error.\n");
    exit(0);
  }
  pwm->set_polarity(pwm,0);
  pwm->set_period(pwm,20000000);
  pwm->set_duty(pwm,1000000);

  pwm->start(pwm);
}

void pwm_final(BBB_pwm *pwm) {
  pwm->stop(pwm);
  BBB_close_pwm(pwm);
}

double average(double *memory_array){
    int i;
    double val;
 
    val=0.0;
    i=0;
    while(i<MMRY_N){
       val+=memory_array[i]; 
       i++;
    }
    val=val/MMRY_N; 
    
    return val;
}

void mmry(double value, double *memory_array){

   int i;

   i=0;
   while(i<MMRY_N-1){
      memory_array[i]=memory_array[i+1];
      i++;
   }
   memory_array[MMRY_N-1]=value;

}

int Blink(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"1"); 
   fclose(fp);
   usleep(100000);

   fp=fopen(file,"w");
   fprintf(fp,"0"); 
   fclose(fp);
   usleep(100000);

}

int Blink_H(){

   FILE *fpL,*fpR;

   fpL=fopen(HART_L,"w");
   fpR=fopen(HART_R,"w");
   fprintf(fpL,"1"); 
   fprintf(fpR,"1"); 
   fclose(fpL);
   fclose(fpR);
   usleep(100000);

   fpL=fopen(HART_L,"w");
   fpR=fopen(HART_R,"w");
   fprintf(fpL,"0"); 
   fprintf(fpR,"0"); 
   fclose(fpL);
   fclose(fpR);
   usleep(100000);

}

int HartOn(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"1"); 
   fclose(fp);

}
int HartOff(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"0"); 
   fclose(fp);

}
