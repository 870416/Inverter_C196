#include _SFR_H_
#include <KB_FUNCS.H>
#include <KB_SFRS.H>
#include "da502d1.h"  /* �������壬ϵͳ˵�� */

#include "inits.h"
/******************************************************
   The current code is based on CSMA502_001E
   dr502f1.c�к���ԭ������ 
   ��������5KVAӦ����Դ��
******************************************************/   

 

BW  bw;
UNA   tempbw0,tempbw1;
MESSAGE1 *msg1;
MESSAGE2 *msg2;
 BYTE    rsb0,rsb1,rsb2,rsp_c;
/* ͨѶ�ã�0/���ã�1/���ͼ�����2/���ռ�����rsp_c/sp_conӳ�� */   
 BYTE    *ptrb0,*ptrb1,*ptrb2,*ptrb3,*ptrb4;   
    /*  0/����,1/����ģʽ,2/���Ͽڣ�3/��������ָ�룬4/��������ָ��  */
 UI      *ptrw0,*ptrw1,*ptrw2,*ptrw3,*ptrw4; 
    /* 0/���ã�1/���ұ�ͷ��2/���ұ�β��3/ʱ������4/ʱ�䷢�ͱ� */
 BYTE    io1,io2;         /*  �˿�Ӱ��      */
 BYTE    c_nb2; /*  �����Ʊ���  */
 BYTE p_erro,s_erro,s_set;
 BYTE tempb0,tempb1,tempb2;
UI   tempw0,tempw1,tempw2,tempw3,tempw4,tempw5,tempw6,tempw7; 
BYTE     num2[2];    /* �ֶ�����0-��ǰ��1-Ԥ�� */
BYTE     s_x1,s_ii1,s_ii2,s_ii3;
UI       s_x2,s_time0;
BYTE     s_control,s_ok,s_send; 



//------------------------------------------------------------------------
#pragma interrupt(hso_int=3)   /* ��ʱ��         */
#pragma interrupt(soft_int=5)  /* �����������   */
#pragma interrupt(rs485_int=6) /* �����ж���     */
#pragma interrupt(ext_int=7)   /* ���ϴ�����     */

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
        if(s_ok==(BYTE)0xff)  /* ������Ч�������������������Ч��־ */
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
          if(s_ok==(BYTE)0xff)  /* ������Ч�������������������Ч��־ */
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
    if(rsb0 & 0x20) /* ���� */
    {
       rsb1++;
       if(rsb1<14)  sbuf=*(ptrb3+rsb1);
         else       { E_485r; }
    }	
    if(rsb0 & 0x40) /* ���� */
    {
       if(rsb2<100)	{ *(ptrb4+rsb2)=sbuf; rsb2++;}
    }	
	return;	
}

void ext_int()  /* ���ϴ��� */
{
    p_erro=*ptrb2;/* ������״̬ */ 
    p_erro|=0x20;
    if((p_erro & 0x07)!=0x07)  /* F1_V��F1_U��F2_A */
    {   
    	io1=0xff;
    	  ioport1=io1;   /* ����� */
    	*(ptrb2+2)=0x00; /* ����ǰ�� */
    	c_nb2=0xff;      /* ���κ� */
    	s_erro &=p_erro;
        return;
    }
    if((p_erro & 0x08)==0) /* I2H  */
    {   /* �򵥴����������к󼶽�Ƶ */
    	io1=0xff;
    	  ioport1=io1;
    }
    if((p_erro & 0x10)==0) /* I1H  */
    {   /* �򵥴����������к󼶽�Ƶ */
    	*(ptrb2+2)=0x00; 
    }
    /* ��ν��ǰ������       */
    /* ��ν���󼶷���       */ 
    /* �������ʾ��ͨѶ����   */
	return;	
}



 
void main()   /* ������ */
{
	init196();    	
  /*  80C196KC��ʼ����оƬ����  */
  	init8255();    	
  /*  8255��ʼ��  */
/*    delay1s();  */
  /*  ������ʱ1S��Ӱ��3525��������ȡ��  */
/*    checkram(); */
  /* RAM��� */  
	init1820();    	
  /*  1820��ʼ��  */
	init485();    	
  /*  ����ͨѶ��ʼ��  */
	initsys(); 
  /*  ϵͳ��ʼ��  */
  for(tempb0=0;tempb0<8;tempb0++) measure1(tempb0); 
  while(1)
  {
  	/*  ���Ź�  */
  	    E_watchdog;
  	/*  ϵͳ��ʱ45��  */
/*  	    delay45m();   */
  	/*  ��Ϣ�Ѽ�   */
    	/*  ��·������������ɢ������ѯ��ʽ   */
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
 	/*  ϵͳ�Բ߱� */
       /* ���������� */
          treaterro(); 
       /*  ��ǰ����Ƶ��ȷ��  */
        { tempw0=s_time0;
          tempw0>>=1;
          if(tempw0 != (*msg1).bt20)
          {
          	(*msg1).bt20=tempw0;
          	
          	tempw1=(*msg1).fnowc;
        	if(s_control & 0x04) tempw1++;
            	else             tempw1-=5;  /* ����ʱ��Ƶ���� */
            tempw1-=(*msg1).addl;	     /* ����ת����Ƶ */
            if((*msg1).snow & 0x03);  
              else  tempw1=F_STARTC;   /* ͣ��״̬Ƶ�ʻص���� */
            if(tempw1 >= F_ENDC)   tempw1=F_ENDC; /* ����Ƶ������ */
            if(tempw1 <= F_STARTC) tempw1=F_STARTC;
            (*msg1).fnowc=tempw1;  /* �������м���������һ�Զ��� */
            tempw1/=10;
            (*msg1).fnow=tempw1;
            
            tempb0=(*msg1).bt20 %5;
            switch(tempb0)
            {
            	case 0 :{ E_485t;rsb1=0;sbuf=*(ptrb3+rsb1);rsb2=10;break;}
            	case 1 :{ treatmess()     ;break;} /* ��Ϣ������������ */
            	case 2 :{ mc_off();disp() ;break;} /* ����ֶϡ�������ʾ */
            	case 3 :{ mc_off();mc_on();break;} /* �ȷֺ�ϣ���ֹ���� */
            	case 4 :{ chk_data()      ;break;} /* ͨѶ���ݴ��� */
            	default:break;
        	}
          }
        }    

    /*  ���ݴ���   */
        /*  ��ǰʱ��ģʽ����  */ 
        if(s_ok==0x00)  /* ������Ч�����¼��� */
        {  
       	  if((*msg1).fnowc!=(*msg1).foldc)  /* ���䲻�� */
          { 
            (*msg1).foldc=(*msg1).fnowc;
    	    time_cal(); 
    	  }	
        }
        if(s_control & 0x80)
        {  
        	s_control&=0x3f;   /* ���״μ����־����������Ч */
        	  s_send^=0xff;    /* ת����ǰ������ */
        	  s_ok=0x00;
        	  
       	    num2[0]=num2[1];
            s_ii1=0;  /* ���� */
            s_ii2=0;  /* ���� */
            s_ii3=0;  /* ���� */
        	/* ʱ��ģʽָ�� */ 
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
                if(s_ok==(BYTE)0xff)  /* ������Ч�������������������Ч��־ */
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