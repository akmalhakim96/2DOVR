#include <time.h>
#include <sys/ioctl.h>
#include "pvm_4frobo.c"
#include "keyin.c"

// =========================================================

#define BASSE_PERIOD 50000 // 20Hz , 1000000: 1Hz
#define THROTTLE_MAX 300

#define DATA_OUTPUT "/tmp/basse.dat" 
// # mount -t tmpfs -o size=128m tmpfs /tmp を実行しておくと，/tmpがramdiskになって高速
#define GYRO_FACTOR 0.3 // is multiplied to gyro value to calculate angl2

struct threed_type norm_accel(struct threed_type, struct threed_type);
struct threed_type norm_gyro(struct threed_type, struct threed_type);

int main(int argc, char **argv){

   char line[256];
   double now,start;
   int avoid;
   int keep_height;
   int flag_height;
   int normal_throttle;
   int i;
   int q;
   int diff=0;
   int keyval=0;
   int tids[3];
   int info;
   int throttle;
   int mesure_count;
   FILE *fp,*fp_gain;

   double angl2;
   double sensitiv;
   double data[256];
   struct gain_t gain;

   time_t timer;

   // 
   srand(783173);
   fp=fopen(DATA_OUTPUT,"a");
   if((fp_gain=fopen(GAINS,"r"))==NULL){
      printf("GAINS open error!!\n");
      exit(EXIT_FAILURE);
   }

   while(fgets(line,sizeof(line),fp_gain)!=NULL){
      printf("%s",line);
   }
   sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf %d %lf %d %d",
      &gain.ax,&gain.gx,&gain.gz,&gain.z,&gain.dz,&gain.us,&gain.dus,&gain.direc,&avoid,&sensitiv,&keep_height,&normal_throttle);
   printf("avoid.dist =%d\n",avoid);
   gain.ay=gain.ax;
   gain.gy=gain.gx;
   fclose(fp_gain);
   fp_gain=fopen(GAINS,"a");

   setlinebuf(stdout);
   signal(SIGINT,sigcatch);

   printf("XX===========================================XX\n");
   printf("X       Basse #2dOVR_B1 since 2016 12/9      X\n");        
   printf("XX=============================== (^.^)/ ====XX\n");
   
   pvm_init_4basse();  // haute is spawned in this init. function
   printf("PVM Init finish.\n");


   // Waiting for haute recv preparation
   while(pvm_recv_from_haute_to_basse(data,1)==0){} 
   printf("Start signal(%lf) is recieved from haute.\n",data[0]);

   keyval=keyin();
   data[0]=(double)keyval;
   pvm_send_from_basse_to_haute(data,1);
   while(keyval!=-999){

      gain=new_gain(gain);

      printf("======================\n");
      printf("sensitiv =%lf\n",sensitiv);
      printf("normal.throttle =%d\n",normal_throttle);
      printf("======================\n");

      data[0]=sensitiv;
      data[1]=normal_throttle;

      pvm_send_from_basse_to_haute(data,2);
      printf("[[ Please waite seconds to initialize SRF02. Thank you. ]]\n\n");
      init_keyboard();

      angl2=0.0;
      flag_height=0;
      start=gettimeofday_sec();
      mesure_count=0;
      throttle=0;
      diff=0;

      //usleep(2000000); // haute@bbbXXw でのタイムスタンプと秒を同期するため必要
      time(&timer);
      fprintf(fp,"\n# %s",ctime(&timer));
      fprintf(fp,"#    sec ");
      fprintf(fp,"      ax ");
      fprintf(fp,"      ay ");
      fprintf(fp,"      gx ");
      fprintf(fp,"      gy ");
      fprintf(fp,"  direct ");
      fprintf(fp," round_d ");
      fprintf(fp," under_d ");
      fprintf(fp,"d_under_d\n");

      printf("[[ throttle : 0 -- 500 ]]\n\n");
      while(throttle>=0){

         diff=mpwr_from_kybd();
         if(diff==-999){
            throttle=-999; // to break this loop
            data[0]=(double)diff;
            pvm_send_from_basse_to_haute(data,1);
         }
         else if((throttle==0)&&(diff<0)){
            data[0]=0.0;
            pvm_send_from_basse_to_haute(data,1);
         }
         else if((throttle>=THROTTLE_MAX)&&(diff>0)){
            data[0]=0.0;
            pvm_send_from_basse_to_haute(data,1);
         }
         else{
            throttle+=diff;
            data[0]=(double)diff;
            pvm_send_from_basse_to_haute(data,1);
         }

         printf("\033[%dA" ,1); // move one line upward
         printf("\033[41m"); // red background
         printf("throttle=%3d; diff=%3d; Quit:q \n",throttle,diff);


         //Recieve from haute data
         if(pvm_recv_from_haute_to_basse(data,8)>0){
            now=gettimeofday_sec();
            
            fprintf(fp,"%8.3f ",now-start);
            fprintf(fp,"%8.3f ",data[0]);
            fprintf(fp,"%8.3f ",data[1]);
            fprintf(fp,"%8.3f ",data[2]);
            fprintf(fp,"%8.3f ",data[3]);
            fprintf(fp,"%8.3f ",data[4]);
            fprintf(fp,"%9.3f ",data[5]);
            fprintf(fp,"%9.3f ",data[6]);
            fprintf(fp,"%9.3f ",data[7]);
            fprintf(fp,"\n");
            fflush(fp);
            
            
            if((throttle>normal_throttle)&&(flag_height==1)){
               mesure_count++;
               angl2+=data[0]*data[0]+data[1]*data[1];
               angl2+=GYRO_FACTOR*GYRO_FACTOR*data[2]*data[2];
               angl2+=GYRO_FACTOR*GYRO_FACTOR*data[3]*data[3];
            }
         }


         usleep(BASSE_PERIOD); // this length of sleep determines period of reflesh diff
      } // end of throttle loop
      now=gettimeofday_sec();


      final_keyboard();
      printf("\033[49m"); // normal background
    
      while(pvm_recv_from_haute_to_basse(data,1)==0){}
   
//      if(mesure_count>0){
         angl2=angl2/((int)mesure_count);
         fprintf(fp_gain,"%8.1f ",gain.ax);
         fprintf(fp_gain,"%7.1f ",gain.gx);
         fprintf(fp_gain,"%7.1f ",gain.gz);
         fprintf(fp_gain,"%6.1f ",gain.z);
         fprintf(fp_gain,"%5.1f ",gain.dz);
         fprintf(fp_gain,"%6.1f ",gain.us);
         fprintf(fp_gain,"%4.1f ",gain.dus);
         fprintf(fp_gain,"%5.1f ",gain.direc);
         fprintf(fp_gain,"%4d ",avoid);
         fprintf(fp_gain,"%6.4f ",sensitiv);
         fprintf(fp_gain,"%3d ",keep_height);
         fprintf(fp_gain,"%3d ",normal_throttle);
         printf("angl2=%lf\n",angl2);
         fprintf(fp_gain,"%5.1f ",data[0]);
         fprintf(fp_gain,"%s",ctime(&timer));
         fflush(fp_gain);
 //     }

      keyval=keyin();
      data[0]=(double)keyval;
      pvm_send_from_basse_to_haute(data,1);
      
   } // end of keyval loop

   pvm_kill(haute_tid);

   fclose(fp);
   fclose(fp_gain);

   return;
 } // END of MAIN
