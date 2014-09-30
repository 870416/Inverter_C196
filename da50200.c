#include _SFR_H_
#include <KB_FUNCS.H>
#include <KB_SFRS.H>
#include "da502d1.h"  /* 参量定义，系统说明 */

#include "inits.h"
/******************************************************
   The current code is based on CSMA502_001E
   dr502f1.c中函数原型申明 
   天津滨海线5KVA应急电源用
******************************************************/   

 

BW  bw;
UNA   tempbw0,tempbw1;
MESSAGE1 *msg1;
MESSAGE2 *msg2;
 BYTE    rsb0,rsb1,rsb2,rsp_c;
/* 通讯用，0/共用，1/发送计数，2/接收计数，rsp_c/sp_con映射 */   
 BYTE    *ptrb0,*ptrb1,*ptrb2,*ptrb3,*ptrb4;   
    /*  0/公用,1/开关模式,2/故障口，3/发送区首指针，4/接收区首指针  */
 UI      *ptrw0,*ptrw1,*ptrw2,*ptrw3,*ptrw4; 
    /* 0/公用，1/正弦表头，2/正弦表尾，3/时间计算表，4/时间发送表 */
 BYTE    io1,io2;         /*  端口影射      */
 BYTE    c_nb2; /*  逆变控制变量  */
 BYTE p_erro,s_erro,s_set;
 BYTE tempb0,tempb1,tempb2;
UI   tempw0,tempw1,tempw2,tempw3,tempw4,tempw5,tempw6,tempw7; 
BYTE     num2[2];    /* 分段数，0-当前，1-预备 */
BYTE     s_x1,s_ii1,s_ii2,s_ii3;
UI       s_x2,s_time0;
BYTE     s_control,s_ok,s_send; 



//------------------------------------------------------------------------
#pragma interrupt(hso_int=3)   /* 定时用         */
#pragma interrupt(soft_int=5)  /* 逆变器控制用   */
#pragma interrupt(rs485_int=6) /* 串口中断用     */
#pragma interrupt(ext_int=7)   /* 故障处理用     */

void hso_int() /* 120S */
{
    hso_command=0x13;    /* 0001,0101,HSO.3=0   */
    hso_time=timer1+9997; /* 10mS,10000-3      */
      enable();
      int_mask=0x20;    
	s_time0++;
	if(s_time0>=12000)  s_time0=0;
	return;
}

void soft_int()
{
    s_x2=*ptrw4;
      s_x2-=AVG_ANSWER;
      ptrw4++;
    s_x1=*ptrb1;
    s_x1|=c_nb2;
    io1|=s_x1;  
      ioport1=io1; 
    if(s_ii3!=(BYTE)0x02) { s_ii3+=2; s_ii3-- ; } 
      else                { s_ii3=0 ; s_ii2++ ; ptrb1-=3;}
    hso_command=0x38;
    hso_time=timer1+s_x2;
    ptrb1++;

    io1=s_x1;
      ioport1=io1;
    if(s_ii2==num2[0])
    {
   	    s_ii2=0;
   	    s_ii1++;
        ptrb1+=3;
        if(s_ok==(BYTE)0xff)  /* 数据有效则更换发送区，并置无效标志 */
          {  s_send^=0xff;s_ok=0x00;num2[0]=num2[1];  }	
      	ptrw4=(UI *)ADDR_CA0;
   	      if(s_send==(BYTE)0xff)  ptrw4=(UI *)ADDR_CA1;
  	}  
    if(s_ii1==(BYTE)0x06) { s_ii1=0; ptrb1-=18; }
    while((s_x2=*ptrw4)==0)
    { 
       ptrb1++;
       ptrw4++;
       if(s_ii3!=(BYTE)0x02) { s_ii3++; } 
         else                { s_ii3=0; s_ii2++  ; ptrb1-=3;}
       if(s_ii2==num2[0])
       {
	      s_ii2=0;
      	  s_ii1++;
          ptrb1+=3;
          if(s_ok==(BYTE)0xff)  /* 数据有效则更换发送区，并置无效标志 */
            {  s_send^=0xff;s_ok=0x00;num2[0]=num2[1]; }	
      	  ptrw4=(UI *)ADDR_CA0;
   	      if(s_send==(BYTE)0xff)  ptrw4=(UI *)ADDR_CA1;
  	   }  
       if(s_ii1==(BYTE)0x06) { s_ii1=0; ptrb1-=18; }
    }
	return;
}    

void rs485_int()
{   
    enable();
      int_mask=0x20;    
	rsb0=sp_stat;
    if(rsb0 & 0x20) /* 发送 */
    {
       rsb1++;
       if(rsb1<14)  sbuf=*(ptrb3+rsb1);
         else       { E_485r; }
    }	
    if(rsb0 & 0x40) /* 接收 */
    {
       if(rsb2<100)	{ *(ptrb4+rsb2)=sbuf; rsb2++;}
    }	
	return;	
}

void ext_int()  /* 故障处理 */
{
    p_erro=*ptrb2;/* 读故障状态 */ 
    p_erro|=0x20;
    if((p_erro & 0x07)!=0x07)  /* F1_V，F1_U，F2_A */
    {   
    	io1=0xff;
    	  ioport1=io1;   /* 即封后级 */
    	*(ptrb2+2)=0x00; /* 即封前级 */
    	c_nb2=0xff;      /* 屏蔽后级 */
    	s_erro &=p_erro;
        return;
    }
    if((p_erro & 0x08)==0) /* I2H  */
    {   /* 简单处理，主程序中后级降频 */
    	io1=0xff;
    	  ioport1=io1;
    }
    if((p_erro & 0x10)==0) /* I1H  */
    {   /* 简单处理，主程序中后级降频 */
    	*(ptrb2+2)=0x00; 
    }
    /* 如何解除前级封锁       */
    /* 如何解除后级封锁       */ 
    /* 如何与显示、通讯相配   */
	return;	
}



 
void main()   /* 主程序 */
{
	init196();    	
  /*  80C196KC初始化，芯片配置  */
  	init8255();    	
  /*  8255初始化  */
/*    delay1s();  */
  /*  启动延时1S，影响3525软启动，取消  */
/*    checkram(); */
  /* RAM检查 */  
	init1820();    	
  /*  1820初始化  */
	init485();    	
  /*  串行通讯初始化  */
	initsys(); 
  /*  系统初始化  */
  for(tempb0=0;tempb0<8;tempb0++) measure1(tempb0); 
  while(1)
  {
  	/*  看门狗  */
  	    E_watchdog;
  	/*  系统定时45分  */
/*  	    delay45m();   */
  	/*  信息搜集   */
    	/*  八路参数采样（分散），查询方式   */
    	{
          if(s_time0 != (*msg1).bt10)
          {
          	(*msg1).bt10=s_time0;
          	tempb0=(*msg1).bt10%5;
          	switch(tempb0)
          	{
          		case 0 :measure1(0);break;
          		case 1 :measure1(1);break;
          		case 2 :measure1(2);break;
          		case 3 :measure1(3);break;
                default:motorp()   ;break;
      		}
	      }  
    	  measure1(0x04)  ;  
    	  measure1(0x05)  ;  
    	  measure1(0x06)  ;  
    	  measure1(0x07)  ;
	    }   
 	/*  系统对策表 */
       /* 两级逆变控制 */
          treaterro(); 
       /*  当前工作频率确定  */
        { tempw0=s_time0;
          tempw0>>=1;
          if(tempw0 != (*msg1).bt20)
          {
          	(*msg1).bt20=tempw0;
          	
          	tempw1=(*msg1).fnowc;
        	if(s_control & 0x04) tempw1++;
            	else             tempw1-=5;  /* 故障时降频加速 */
            tempw1-=(*msg1).addl;	     /* 工况转换降频 */
            if((*msg1).snow & 0x03);  
              else  tempw1=F_STARTC;   /* 停机状态频率回到起点 */
            if(tempw1 >= F_ENDC)   tempw1=F_ENDC; /* 运行频率限制 */
            if(tempw1 <= F_STARTC) tempw1=F_STARTC;
            (*msg1).fnowc=tempw1;  /* 过程用中间变量，结果一言而决 */
            tempw1/=10;
            (*msg1).fnow=tempw1;
            
            tempb0=(*msg1).bt20 %5;
            switch(tempb0)
            {
            	case 0 :{ E_485t;rsb1=0;sbuf=*(ptrb3+rsb1);rsb2=10;break;}
            	case 1 :{ treatmess()     ;break;} /* 信息处理、工况控制 */
            	case 2 :{ mc_off();disp() ;break;} /* 电机分断、数码显示 */
            	case 3 :{ mc_off();mc_on();break;} /* 先分后合，防止意外 */
            	case 4 :{ chk_data()      ;break;} /* 通讯数据处理 */
            	default:break;
        	}
          }
        }    

    /*  数据处理   */
        /*  当前时间模式计算  */ 
        if(s_ok==0x00)  /* 数据无效则重新计算 */
        {  
       	  if((*msg1).fnowc!=(*msg1).foldc)  /* 不变不算 */
          { 
            (*msg1).foldc=(*msg1).fnowc;
    	    time_cal(); 
    	  }	
        }
        if(s_control & 0x80)
        {  
        	s_control&=0x3f;   /* 清首次计算标志，置数据无效 */
        	  s_send^=0xff;    /* 转换当前发送区 */
        	  s_ok=0x00;
        	  
       	    num2[0]=num2[1];
            s_ii1=0;  /* 区数 */
            s_ii2=0;  /* 段数 */
            s_ii3=0;  /* 节数 */
        	/* 时间模式指针 */ 
        	tempw0=ADDR_CA0;
        	  if(s_send==(BYTE)0xff) tempw0=ADDR_CA1;
        	  ptrw4=(UI *)tempw0;
        	  
            while((s_x2=*ptrw4)==0)
            { 
              ptrb1++;
              ptrw4++;
              if(s_ii3!=(BYTE)0x02) { s_ii3++; } 
                else                { s_ii3=0; s_ii2++  ; ptrb1-=3;}
              if(s_ii2==num2[0])
              {
      	        s_ii2=0;
      	        s_ii1++;
                ptrb1+=3;
                if(s_ok==(BYTE)0xff)  /* 数据有效则更换发送区，并置无效标志 */
                  {  s_send^=0xff;s_ok=0x00;num2[0]=num2[1];  }	
        	    tempw0=ADDR_CA0;
        	      if(s_send==(BYTE)0xff) tempw0=ADDR_CA1;
        	      ptrw4=(UI *)tempw0;
  	          }  
              if(s_ii1==(BYTE)0x06) { s_ii1=0; ptrb1-=18; }
            }
        	disable();
          	  hso_command=0x38;
        	  hso_time=timer1+0x04;
        	enable();
        }
  }
}