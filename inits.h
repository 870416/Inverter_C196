#ifndef DA502F1_HH
#define DA502F1_HH
void init196();
void init8255();
void init485();
void init1820();
void initsys();
void Err_handler();
void treatmess();
void mc_off();
void mc_on();    /* 接触器控制之闭合 */  
   void time_cal();   /* 用外部RAM */
void  delay45m();
void disp();
void Motor_petection();   /* 电机过流保护、定时、恢复(20min);  */
void chk_data();
void measure1();
#endif
