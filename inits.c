/*
  da502f1.c�к���ԭ������
  ����  WJDA502_002
  ��������5KVAӦ����Դ�� 
*/
#include _SFR_H_
#include <KB_SFRS.H>
#include <KB_FUNCS.H>
#include "da502d1.h"
#include "inits.h"
/*
  da502f1.c�к���ԭ������ 
  ����  CSMA502_001E��2003/04/10 
  �Ķ�֮����
  1��Ƿѹʱֻ��Ƶ����ǰ����
  2��ÿ�μ���ǰ��ͨ��50S��
*/

/**/
/* 80C196KC��ʼ���� */
void init196()
{   /* �����ջ */
/*
  	ptrw0=(UI *)0x18;
    	*ptrw0=0x100; 
*/
	io1=0xff;
      ioport1=io1;
	io2=0xff;
      ioport2=io2;
	disable();
	int_mask=0x00;
	imask1=0x00;
	int_pending=0x00;  /*  ��ֹ�ж�   */
	ioc1=0x70;  /* һ����λ������ֲ�����ʱ�Ķ�-��-д */
	
    wsr=0x01;
    ad_result_hi=0xec;/* 1110,1100 ʵ��ΪAD_TIME,20��S */
    wsr=0x00;     /*  ˮƽ����0     */
    ioc2=0x08;
	;
	return;
}

/*  8255��ʼ���� */
void init8255()
{
	ptrb2=(BYTE *)ADDR_8255;
	*(ptrb2+3)=0x90;  /* (A�����룬B��C���) */
	*(ptrb2+1)=0xff;  /* �����ȫ��          */
	*(ptrb2+3)=0x00;  /* ��ֹǰ�����,PC.0=0 */	
	return;
}

/* ����ͨѶ��ʼ���� */
void init485()
{
    disable();
/* ����дioport0������ͨѶ����ʵ����д�Ĳ����ʣ�����*/
      baud_rate=0x6b;  /* 16MHz/(bt(bps)*16)-1,cfH-4800;67H-9600 */
      /* Ϊ��ϲ��ݰ壬������ʵ��Ϊ9259 */
      baud_rate=0x80;  /* ��λΪ1ѡ�ڲ�ʱ�ӣ�6M-0x26    */
      rsp_c=0x09;      /* ����ֱ�Ӷ�sp_con���в���������ͨѶ��*/
      sp_con=rsp_c;    /* MODE=1               */
      int_mask|=0x40;  /* ������ͨѶ�ж�     */
      rsb0=0;
      rsb1=0;
      rsb2=0;
      ptrb3=(BYTE *) ADDR_MEM1;
      ptrb4=(BYTE *) ADDR_MEM2;
      E_485t; 
    enable();
	return;
}

/* 1820��ʼ����*/
void init1820()
{
	return;
}

/* ϵͳ��ʼ����*/
void initsys()
{
    ptrb1=(BYTE *)ADDR_MOD;  /* ����ģʽָ�� */
    ptrb2=(BYTE *)ADDR_8255; /* ���Ͽ�ָ��   */

    /* ͣ������״̬ */
    msg1=(MESSAGE1 *)ADDR_MEM1;
    (*msg1).head1=0x55;
    (*msg1).u1   =0x00;
    (*msg1).u2_l =0x00;
    (*msg1).u2_h =0x00;
    (*msg1).i1   =0x00;
    (*msg1).i2   =0x00;
    (*msg1).fend =50;
    (*msg1).fnow=(*msg1).fnowc/10;
    (*msg1).send =0x00;
    (*msg1).snow =0x00;
    (*msg1).ec1  =0x00;
    (*msg1).ec2  =0xff;
    (*msg1).sendm=0x00;
    
    (*msg1).end1 =0xfd;
    (*msg1).fendc=(*msg1).fend*10;
    (*msg1).fnowc=F_STARTC;
    (*msg1).foldc=(*msg1).fnowc-10;
    (*msg1).addl=0x02;  
    (*msg1).t0a=0x00;
    
    (*msg1).t0d=0x00;
    (*msg1).t1d=0x00;
    (*msg1).t2d=0x00;
    (*msg1).t3d=0x00;
    
    (*msg1).im0=0x00;
    (*msg1).im1=0x00;
    (*msg1).im2=0x00;
    (*msg1).im3=0x00;
    
    (*msg1).m0a=0x00;
    (*msg1).m1a=0x00;
    (*msg1).m2a=0x00;
    (*msg1).m3a=0x00;
    
    (*msg1).bt10=0x00;
    (*msg1).bt20=0x00;
    (*msg1).bt1m=0x00;
    (*msg1).dt45m=0x00;
    
    msg2=(MESSAGE2 *)ADDR_MEM2;
    for(tempb1=0;tempb1<100;tempb1++)  *(ptrb4+tempb1)=0x00;
    
    /* ��ʱ�жϣ��޷�����������  */
    s_time0=0;
    int_mask|=0x08;
    disable();
      hso_command=0x13;      /* 0001,0101,HSO.3=0   */
      hso_time=timer1+10000; /* 10mS                */
    enable();

    /*  ���뿪��״̬��ȡ       */
    tempb0=ioport2;  /* p2.4-p2.3 */
    tempb0&=0x18; 
    s_set=0xff;   
    switch(tempb0)  /* 01/����״̬������/����״̬ */
    {
    	case  0x00: s_set&=0xf5;break;              
		case  0x08: s_set&=0xfd;break;
		case  0x10: s_set&=0xf7;break;
       	default   :             break;
    }	

    s_erro=0xff;
    s_control=0xb4;   /* 1011,0100 ������д��0xc4������������ */
      s_ok=0x00;
      s_send=0xff;
    c_nb2=0xff;
      
    int_mask|=0xa0;   /* 1010,0000,      */  
    int_pending=0x00;
    
    E_watchdog;
	return;
}

/* ���жϹ��ϴ�����������������*/
void treaterro()
{
    /*  ���Ͽ����ݶ�ȡ         */
    p_erro=*(ptrb2+0);  
      p_erro|=0x20;     /* PA.5λ����δ��  */
	tempb0 =s_control;
	tempb0|=0x0f;
	if((*msg1).dt45m>=45) tempb0&=0xf1; /* ��ʱ��,��ֹ���� */
   	tempb1=p_erro;
      tempb1&=s_erro;	
    (*msg1).ec2=tempb1;   /* �����ź��⴫ */
    if((s_erro & 0x07)!=0x07) (*msg1).snow&=0x0c; /* IGBT����ͣ�� */  
	if(tempb1 & 0x80);
	  else { tempb0&=0xfb;} /* .7��   ǰ��Ƿѹʱ������ѹ   */
/*	
    if(tempb1 & 0x40);
	  else { tempb0&=0xf7;} 
*/
	                        /* .6��   �󼶹�ѹ   */
/* ��CPU�Բ��������ص�������ǰ��Ƶ���ضϣ������ڱ�ѹ���͵�� */
	if(tempb1 & 0x10);
	  else { tempb0&=0xf3;} /* .4��   ǰ������   */
	if(tempb1 & 0x08); 
	  else { tempb0&=0xfb;} /* .3��   �󼶹���   */
	if(tempb1 & 0x04);
	  else { tempb0&=0xf1;} /* .2��   IGBT����   */
	if(tempb1 & 0x02);
	  else { tempb0&=0xf1;} /* .1��   IGBT����   */
	if(tempb1 & 0x01);
	  else { tempb0&=0xf1;} /* .0��   IGBT����   */
    tempw0=(*msg1).u2_h;
      tempw0<<=8;
      tempw0+=(*msg1).u2_l;		
/*	if((*msg1).u1<= 40) { tempb0&=0xf3;}  */ /* ǰ����ѹ���㣡   */
    if(s_set!=0xfd)  /* ����ʱ���� */
    {
	  if((*msg1).u1>=145) { tempb0&=0xf3;}   /* ǰ����ѹ���ߣ�   */
	  if(tempw0<=300)     { tempb0&=0xfb;}   /* �м��ѹ���㣡   */
    }
	if(tempw0>=580) { tempb0&=0xf7;}   /* �м��ѹ���ߣ�   */
	if(tempw0>=600) { tempb0&=0xf1;}   /* �м��ѹΣ�գ�   */
	if((*msg1).i1>= 75) { tempb0&=0xf3;}   /* ǰ������Σ�գ�   */
	if((*msg1).i2>= 20) { tempb0&=0xf1;}   /* �м����Σ�գ�   */
	tempb1=0x00;  /* ǰ������ */
 	  if(tempb0 & 0x08) tempb1=0x01; 
      *(ptrb2+3)=tempb1; 
      
/* �����c_nb2���������ʹ���Ʋ�����Ӧ�е�ë�̣��ù��ɱ������������
   �����жϵ�ԭ�򣬿��Ʊ���Ӧһ�Զ��� */ 
	if(tempb0 & 0x02) tempb1=0x00;
	  else               tempb1=0xff;
	if((*msg1).fnowc==F_STARTC) tempb1=0xff;
	c_nb2=tempb1;  
	s_control=tempb0;
    return;
}   

void treatmess()  /* ��Ϣ�����������ơ�������� */
{   
    /* ȷ��Ŀ��Ƶ�ʡ�����Ŀ��Ƶ�� */
	  tempb1=(*msg2).wend;   
 	  tempb1&=0xf0;
 	  tempb1>>=4;
      tempb2=50;
      tempb2-=tempb1;
      (*msg1).fend=tempb2;
      (*msg1).fendc=(*msg1).fend*10;
	/* ȷ�������Ŀ�깤�� */
	  tempb1=(*msg2).wend;  
 	  tempb1&=0x0f;
      (*msg1).send=tempb1;
    /* ȷ������������� */
	  tempb2=(*msg1).send;  
	    tempb2&=0x03;  
	  tempb1=(*msg1).ec1;
	  if(tempb1 & 0x01)  tempb2=0x00;/* ͨ�����ͣ�� */ 
	  if(tempb1 & 0x06)              /* �����������תͨ�� */
	    if(tempb2 == 0x02) 	tempb2=0x01; 
	  if(tempb1 & 0x08)              /* ���ȹ�������תͨ�� */
	    if(tempb2 == 0x03)  tempb2=0x01;
	    
      tempw0=(*msg1).t0a;         /* ��ֹ��ʱ�������� */   
	    if(((*msg1).snow & 0x03)==0x01)  tempw0++;
	      else  tempw0=0;            /* ͨ�繤����ʱ */
	    if(tempw0>500) tempw0=500;   /* 50S */
        if((tempb2 & 0x02)>((*msg1).snow & 0x02)) 
          if(tempw0<500)  tempb2=0x01;/* ͨ�粻�㲻����� */
	    (*msg1).t0a=tempw0;
	      
      tempw0=(*msg1).fnow;           /* Ƶ��>6Hz������� */
        if((tempb2 & 0x02)>((*msg1).snow & 0x02)) 
        { if(tempw0>=6)  
          { 
	        tempb2=0x01;
	        (*msg1).addl=0x02;    /* ��Ƶ */	
          } 
        }
        else  (*msg1).addl=0x00;  /* ��Ƶ */
	  tempb1=(*msg1).send;  /* ȡ�˿���Ϣ */
	    tempb1&=0x0c;   
        tempb2|=tempb1;     /* ȡ������Ϣ */
	  (*msg1).sendm=tempb2;   
    /* ȷ����ǰ���� */
      tempb1=(*msg1).sendm;   /* ����ת�� */
      tempb2=(*msg1).snow;  
        tempb1&=0x03;   /* ������ */
        tempb2&=0x03;   /* ��ǰ���� */
        (*msg1).snow&=0xfc; /* ֻ������ǰ�˿� */
        switch(tempb2)
        {  case 0x00: /* ͣ��-ֻתͨ��      */
        	 if(tempb1>0x00)  (*msg1).snow|=0x01;
        	 break;
           case 0x01: /* ͨ��ת���⹤�� */	 
        	 (*msg1).snow|=tempb1;
        	 break;
           case 0x02: /* ���䲻ת���� */	 
        	 if(tempb1==0x03) (*msg1).snow|=0x01;
        	   else  (*msg1).snow|=tempb1;
        	 break;  
           case 0x03: /* ���Ȳ�ת���� */	 
        	 if(tempb1==0x02) (*msg1).snow|=0x01;
        	   else  (*msg1).snow|=tempb1;
        	 break;
           default  :break;	 
    	}
      tempb1=(*msg1).sendm; /* ����ͣ��������������ݸ�λ */  
      tempb2=(*msg1).snow;  
        if((tempb1 & 0x0c)!=(tempb2 & 0x0c)) 
        {
          tempb1&=0x0c;
          (*msg1).snow=tempb1; /* ����ͣ����ͣ��-�����仯��Ȼ */
          
          (*msg1).ec1&=0xf0;  /* ����������ݸ�λ */
          (*msg1).t0d=0x00;
          (*msg1).t1d=0x00;
          (*msg1).t2d=0x00;
          (*msg1).t3d=0x00;
          (*msg1).m0a=0x00;
          (*msg1).m1a=0x00;
          (*msg1).m2a=0x00;
          (*msg1).m3a=0x00;
  	    }
  	/* ͣ����λ:Ϊ������׼�� */
  	  tempb1=(*msg1).snow;
  	  tempb1&=0x03;
  	  if(tempb1==0)   {  (*msg1).t0a=0x00;    (*msg1).addl=0x02;  }
    /* ȷ��������� */
      tempb1=(*msg1).snow;
      tempb1&=0x03;
      switch(tempb1)
      {
      	case  0:tempb2=0x00;break;
      	case  1:tempb2=0x10;break;
      	case  2:tempb2=0x70;break;
      	case  3:tempb2=0x90;break;
      	default:break;
  	  }
  	  tempb1=(*msg1).snow;
  	  tempb1&=0x0f;
  	  tempb1|=tempb2;
  	  (*msg1).snow=tempb1;
  	  return;
}

void mc_off()    /* �Ӵ�������֮�ֶ� */  
{  	tempb1=(*msg1).snow;
  	if(tempb1 & 0x10);  /* ͨ����ֶ� */
      else
        { disable();hso_command=0x00; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x20);  /* �������ֶ� */
      else
        { disable();hso_command=0x01; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x40);  /* ѹ�����ֶ� */
      else
        { disable();hso_command=0x02; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x80);  /* ����ȷֶ� */
      else
        { disable();hso_command=0x03; hso_time=timer1+4;enable(); }
    tempb1&=0x0c;
    switch(tempb1)  
    {
      case 0x04:  /* һ�� */
          { disable();hso_command=0x05; hso_time=timer1+4;enable();
            break;
          }
      case 0x08:  /* ���� */
          { disable();hso_command=0x04; hso_time=timer1+4;enable();
            break;
          }
      default:    /* ��Чͣ�� */
          { disable();hso_command=0x04; hso_time=timer1+4;enable();
        	  tempb2+=2;tempb2-=2;
        	disable();hso_command=0x05; hso_time=timer1+4;enable();
            break;
          }
    }
	return;
}

void mc_on()    /* �Ӵ�������֮�պ� */  
{  	tempb1=(*msg1).snow;
  	if(tempb1 & 0x10)  /* ͨ��� */
      { disable();hso_command=0x20; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x20)  /* ������ */
      { disable();hso_command=0x21; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x40)  /* ѹ���� */
      { disable();hso_command=0x22; hso_time=timer1+4;enable(); }
  	if(tempb1 & 0x80)  /* ����� */
      { disable();hso_command=0x23; hso_time=timer1+4;enable(); }
    tempb1&=0x0c;
    switch(tempb1)  /* �ȶϺ�� */
    {
      case 0x04:  /* һ�� */
          { 
        	disable();hso_command=0x24; hso_time=timer1+4;enable();
            break;
          }
      case 0x08:  /* ���� */
          { 
        	disable();hso_command=0x25; hso_time=timer1+4;enable();
            break;
          }
      default:    /* ��Чͣ�� */
          { 
            break;
          }
    }
	return;
}

/* ����������*/
   /* tempb0/����   ��               */  
   /* tempw0/���Ʊ� , tempw1/Ts , tempw2 , tempw3 /2^10*M*TS ��/����   */
   /* Ƶ��ȷ��   */
   /* �ֶ���ȷ�� ,tempb1,tempw1 */
   
   /* ��������   */
   void time_cal()   /* ���ⲿRAM */
   {
      tempw0=(*msg1).fnowc;
        if(tempw0>500) tempw0=500;
/* tempw0=300;        */
      /* �ֶ���ȷ��,ת��Ƶ��<8K */
           if  (tempw0<111)  num2[1]=120;
      else if  (tempw0<167)  num2[1]= 80;
      else if  (tempw0<222)  num2[1]= 60;
      else if  (tempw0<278)  num2[1]= 48;
      else if  (tempw0<333)  num2[1]= 40;
      else if  (tempw0<444)  num2[1]= 30;
      else if  (tempw0<556)  num2[1]= 24;
      else if  (tempw0<667)  num2[1]= 20;
      else                   num2[1]= 15; 

      /* �ֶ���ȷ��,ת��Ƶ��<5K */
/*
           if  (tempw0< 69)  num2[1]=120;
      else if  (tempw0<104)  num2[1]= 80;
      else if  (tempw0<139)  num2[1]= 60;
      else if  (tempw0<208)  num2[1]= 40;
      else if  (tempw0<277)  num2[1]= 30;
      else if  (tempw0<347)  num2[1]= 24;
      else if  (tempw0<417)  num2[1]= 20;
      else if  (tempw0<556)  num2[1]= 15;
      else if  (tempw0<694)  num2[1]= 12;      
      else                   num2[1]=  8; 
*/
      /* �ֶ���ȷ��,ת��Ƶ��<4K */
/*
           if  (tempw0< 56)  num2[1]=120;
      else if  (tempw0< 83)  num2[1]= 80;
      else if  (tempw0<111)  num2[1]= 60;
      else if  (tempw0<167)  num2[1]= 40;
      else if  (tempw0<222)  num2[1]= 30;
      else if  (tempw0<278)  num2[1]= 24;
      else if  (tempw0<333)  num2[1]= 20;
      else if  (tempw0<444)  num2[1]= 15;
      else if  (tempw0<555)  num2[1]= 12;      
      else                   num2[1]=  8; 
*/
      /* �ֶ���ȷ��,ת��Ƶ��<3K */
/*
           if  (tempw0< 42)  num2[1]=120;
      else if  (tempw0< 63)  num2[1]= 80;
      else if  (tempw0< 83)  num2[1]= 60;
      else if  (tempw0<125)  num2[1]= 40;
      else if  (tempw0<167)  num2[1]= 30;
      else if  (tempw0<208)  num2[1]= 24;
      else if  (tempw0<250)  num2[1]= 20;
      else if  (tempw0<333)  num2[1]= 15;
      else if  (tempw0<417)  num2[1]= 12;      
      else                   num2[1]=  8; 
*/
      ptrw1=(UI *)ADDR_WID;
      tempbw0.ll=*(ptrw1+tempw0); /* ����ֱ���������ֳ� */
      tempbw0.ll*=1060;  /* ��ѹ���� 530V*2    */      	 
/* ȡ����������Ƶ */
         tempw7=530; 
      tempbw0.ll/=tempw7;
      tempw0=tempbw0.bw.lo; 
      /* ���ָ�� */ 
      tempw4=240;
      tempw4/=num2[1];
      tempb0= tempw4;  /* ˫�ֽ� */
      tempw4>>=1;  
      tempw5= 240;     /* ��Ҫ�ø�����ʽ */
      tempw5-=tempb0;
      tempw5+=tempw4;
      
      ptrw1=(UI *)ADDR_SIN;
      ptrw2=(UI *)ADDR_SIN;
      ptrw1+=tempw4;       /*  SIN��A��*/
      ptrw2+=tempw5;       /*  SIN��60-A��*/
      
      ptrw3=(UI *)ADDR_CA1;
        if(s_send==(BYTE)0xff) ptrw3=(UI *)ADDR_CA0;
      
      tempbw0.ll=(*msg1).fnowc;
      tempbw0.ll*=num2[1]; /* 10F*N */
      tempw7=tempbw0.bw.lo; 
      tempbw0.ll=0x196e6b;       /* 10*fosc/6/16 */ 
      tempbw0.ll/=tempw7;          
      tempw1=tempbw0.bw.lo;      /* Ts */
      
      tempbw0.ll=tempw0;
      tempbw0.ll*=tempw1;
      tempw2=tempbw0.bw.lo;  /* TS*2^10*M ���� */
      tempw3=tempbw0.bw.hi;  /* TS*2^10*M ���� */  
      
      /* ʱ����� */
      for(tempb1=0;tempb1<num2[1];tempb1++)
      {  
         tempw4=*ptrw1;
           tempbw0.ll=tempw4;
           tempbw0.ll*=tempw2;
           if(tempw3==0)  { tempbw1.ll=0; }
             else { tempbw1.ll=tempw4;tempbw1.ll*=tempw3; }
           tempbw1.ll+=tempbw0.bw.hi;
           tempbw1.ll>>=4;    /* /2^20 */
           tempw4=tempbw1.bw.lo;       	     	     	 
      	   ptrw1+=tempb0;
         tempw5=*ptrw2;
           tempbw0.ll=tempw5;
           tempbw0.ll*=tempw2;
           if(tempw3==0)  { tempbw1.ll=0; }
             else { tempbw1.ll=tempw5;tempbw1.ll*=tempw3; }
           tempbw1.ll+=tempbw0.bw.hi;
           tempbw1.ll>>=4;    /* /2^20 */
           tempw5=tempbw1.bw.lo;       	     	     	 
      	   ptrw2-=tempb0;
      	 tempw7=tempw4;
      	   tempw7+=tempw5;
      	   if(tempw1>=tempw7) {tempw6=tempw1;tempw6-=tempw7; }
             else
             { tempw6=0;
               tempw7-=tempw1; 
               tempw7>>=1; 
               if(tempw4<tempw7) 
               {  tempw5-=tempw7; 	
                  tempw5-=tempw7;
               } 
                 else
                 { if(tempw5<tempw7) 
                   {  tempw4-=tempw7; 	
                      tempw4-=tempw7;
                   } 
                     else
                     { tempw4-=tempw7; 	
                       tempw5-=tempw7;
                     } 
                 }    
             } 
           if(tempw6<=TIMER0_MODIFY)  tempw6=0;
             else  tempw6-=TIMER0_MODIFY;
         
         /* ��С�������� */
         tempw7=MIN_PULSE;
         tempw7>>=1;
         if(tempw4<=tempw7) tempw4=0;
           else 
             if(tempw4<=MIN_PULSE) tempw4=MIN_PULSE; 
         if(tempw5<=tempw7) tempw5=0;
           else 
             if(tempw5<=MIN_PULSE) tempw5=MIN_PULSE; 
         if(tempw6<=tempw7) tempw6=0;
           else 
             if(tempw6<=MIN_PULSE) tempw6=MIN_PULSE; 
              
         *ptrw3=tempw5;/* !!,˳��!! */
            ptrw3++;
         *ptrw3=tempw4;
            ptrw3++;
         *ptrw3=tempw6;
            ptrw3++;
      }
      s_ok=0xff; /* ��������Ч��־ */
      return; 	  
   }
   

/* �жϺ����� */

/*  ������ʱ1S  */
/*
void  delay1s()
{
	for(tempw0=0;tempw0<60000;tempw0++) 
	{ 
		tempb0+=16;
	    tempb0-=16;
		tempb0+=16;
	    tempb0-=16;
	}
	return;
}
*/

/* RAM��� */  
/*
void  checkram()
{
	ptrw0=(UI *)ADDR_MEM1;
	for(tempw0=0;tempw0<4096;)
	{  *ptrw0=tempw0;
	   while((tempw1=*ptrw0) != tempw0)	*ptrw0=tempw0;
	   tempw0++;
	   ptrw0++;
    }
    return;
}   
*/
/*
void disv(xx1)
BYTE   xx1;
{
	BYTE xx2;
	xx1%=16;
    ptrb0=(BYTE *)ADDR_DIS;
	xx2=*(ptrb0+xx1);
	*(ptrb2+1)=xx2;
}
*/

/*  ϵͳ��ʱ45��  */
void  delay45m()
{
    tempbw0.ll=s_time0;
    tempbw0.ll/=6000;
    if(tempbw0.bw.lo != (*msg1).bt1m) 
    {  (*msg1).bt1m=tempbw0.bw.lo;
       (*msg1).dt45m+=1;   /* ����ʱ�趨Ϊ5min */
    }	
	if((*msg1).dt45m>45)  (*msg1).dt45m=45;
	return;
	
}

/* �������ʾ����������*/
void disp()
{  /* ������ʾƵ�ʣ�������ʾ���� */
   tempw0=s_time0;
   tempw0/=40;  /*  400mS  */
   tempw0%=4;
   tempb0    = p_erro;
     tempb0 &= s_erro;
   if(tempw0==0)
   {  if(tempb0==0xff)
   	  { 
   	  	if((*msg1).fnowc==F_STARTC)
   	  	{
  		  (*msg1).dis0=18;  
   	  	  (*msg1).dis1=20;
   	  	  (*msg1).dis2=18;

  		}
  		  else
  		  {
  		  	(*msg1).dis0=15;  
   	  	    (*msg1).dis1=(*msg1).fnow/10;
   	  	    (*msg1).dis2=(*msg1).fnow%10;
  	      }
  	  }
  	    else
  	    {
  	      (*msg1).dis0=14;  
 	  	  (*msg1).dis1=tempb0/16;
   	  	  (*msg1).dis2=tempb0%16;
    	}
   	  (*msg1).dis3=20;
   }
   switch(tempw0)
   {
      case 0 :tempb1=(*msg1).dis0; break;
      case 1 :tempb1=(*msg1).dis1; break;
      case 2 :tempb1=(*msg1).dis2; break;
      default:tempb1=(*msg1).dis3; break;
   }
   ptrb0=(BYTE *)ADDR_DIS;
   tempb0=*(ptrb0+tempb1);
   *(ptrb2+1)=tempb0;
   ;
   return;
}

void measure1(port0_num)  /* ��ѯ��ʽ����,6�� */
BYTE port0_num;
{
	tempw0=0xffff;    /* ��Сֵ */
	tempw1=0x0000;    /* ���ֵ */
	tempw2=0x0000;    /* �ۼӺ� */
    port0_num+=0x08;  /* 10λ���������������ж�*/
	for(tempb1=0;tempb1<0x06;tempb1++)
	{
	   ad_command=port0_num;
	   while(ad_result_lo & 0x08)  E_watchdog;
	   tempw3=ad_result_hi;
	   tempw3<<=8;
	   tempw3+=ad_result_lo;
	   tempw3>>=6;
	   if(tempw3<tempw0) tempw0=tempw3;
	   if(tempw3>tempw1) tempw1=tempw3;
	   tempw2+=tempw3;
	   E_watchdog;

    }  
    tempw2-=tempw0;
    tempw2-=tempw1;
    tempw2>>=2;
    port0_num-=0x08;
    switch(port0_num)   /* 5*k/2^10 */
    {
      case 0 :  /* k=20��0.25 */
        {  tempw2*= 5; tempw2>>=10; tempw2*= 5; (*msg1).im0 =tempw2;
           tempw0=(*msg1).m0a;tempw0>>=10;(*msg1).m0a-=tempw0;/* -1/1024 */
           if(tempw2>I013)
           {
              tempw2-=I013;
              tempbw0.ll=tempw2;
              tempbw0.ll*=tempw2;
              if(tempbw0.ll>I017) { (*msg1).m0a=I017;(*msg1).m0a++; }
                else  (*msg1).m0a+=tempbw0.bw.lo;
       	   }
           break;
    	}
      case 1 :  /* k=20��0.5 */
        {  tempw2*=10; tempw2>>=10; tempw2*= 5; (*msg1).im1 =tempw2;
           tempw0=(*msg1).m1a;tempw0>>=10;(*msg1).m1a-=tempw0;/* -1/1024 */
           if(tempw2>I113)
           {
              tempw2-=I113;
              tempbw0.ll=tempw2;
              tempbw0.ll*=tempw2;
              if(tempbw0.ll>I117) { (*msg1).m1a=I117;(*msg1).m1a++; }
                else  (*msg1).m1a+=tempbw0.bw.lo;
       	   }
           break;
    	}
      case 2 :  /* k=20��2.5 */ 
        {  tempw2*=50; tempw2>>=10; tempw2*= 5; (*msg1).im2 =tempw2;
           tempw0=(*msg1).m2a;tempw0>>=10;(*msg1).m2a-=tempw0;/* -1/1024 */
           if(tempw2>I213)
           {
              tempw2-=I213;
              tempbw0.ll=tempw2;
              tempbw0.ll*=tempw2;
              if(tempbw0.ll>I217) { (*msg1).m2a=I217;(*msg1).m2a++; }
                else  (*msg1).m2a+=tempbw0.bw.lo;
       	   }
           break;
    	}
      case 3 :  /* k=20��2.5 */ 
        {  tempw2*=50; tempw2>>=10; tempw2*= 5; (*msg1).im3 =tempw2;
           tempw0=(*msg1).m3a;tempw0>>=10;(*msg1).m3a-=tempw0;/* -1/1024 */
           if(tempw2>I313)
           {
              tempw2-=I313;
              tempbw0.ll=tempw2;
              tempbw0.ll*=tempw2;
              if(tempbw0.ll>I317) { (*msg1).m3a=I317;(*msg1).m3a++; }
                else  (*msg1).m3a+=tempbw0.bw.lo;
       	   }
           break;
	   	}
      case 4 : tempw2*=25; tempw2>>=8; (*msg1).i1 =tempw2; break;
      case 5 : tempw2*=25; tempw2>>=7; (*msg1).u1 =tempw2; break;
      case 6 : tempw2*= 5; tempw2>>=8; (*msg1).i2 =tempw2; break;
      case 7 : tempw2*=31; tempw2>>=5; (*msg1).u2_l=tempw2;
                           tempw2>>=8; (*msg1).u2_h=tempw2;break;
   	  default:                                             break;
    }
	return;
}  

void motorp()   /* ���������������ʱ���ָ�(20min)  */
{
	 /* ͨ�����������ʱ�� */
/*	(*msg1).t0a++;      
      if((*msg1).t0a>=1000)  (*msg1).t0a=1000;
*/
	if((*msg1).m0a>=I017) { (*msg1).m0a=0;(*msg1).ec1|=0x01;}
	if((*msg1).m1a>=I117) { (*msg1).m1a=0;(*msg1).ec1|=0x02;}
	if((*msg1).m2a>=I217) { (*msg1).m2a=0;(*msg1).ec1|=0x04;}
	if((*msg1).m3a>=I317) { (*msg1).m3a=0;(*msg1).ec1|=0x08;}
	
	tempb1=(*msg1).ec1;
	if(tempb1 & 0x01) (*msg1).t0d++;
	if(tempb1 & 0x02) (*msg1).t1d++;
	if(tempb1 & 0x04) (*msg1).t2d++;
	if(tempb1 & 0x08) (*msg1).t3d++;
	
	if((*msg1).t0d >= (UI)24000) { (*msg1).t0d=0;(*msg1).ec1&=0xfe;}
	if((*msg1).t1d >= (UI)24000) { (*msg1).t1d=0;(*msg1).ec1&=0xfd;}
	if((*msg1).t2d >= (UI)24000) { (*msg1).t2d=0;(*msg1).ec1&=0xfb;}
	if((*msg1).t3d >= (UI)24000) { (*msg1).t3d=0;(*msg1).ec1&=0xf7;}
	
	return;
}	

void chk_data()
{
	(*msg2).wenda=0x00;
	(*msg2).wendb=0x00;
	(*msg2).wendc=0x00;

	for(tempb1=10;tempb1<100;)
	{
	    tempb2=*(ptrb4+tempb1);
        if((tempb2==0xaa) && ((*msg2).wendc==0x00))
	    {  /*  C�������б�  */
	    	tempb1++;
   	    	  tempb2=*(ptrb4+tempb1);
	          (*msg2).wendc=tempb2;
	    	tempb1++;
   	    	  tempb2+=*(ptrb4+tempb1);
              if(tempb2==0xff)
              {
              	tempb1++;
   	    	    tempb2=*(ptrb4+tempb1);
                if(tempb2!=0xfd) (*msg2).wendc=0x00;
              }
              else (*msg2).wendc=0x00;
    	}

	    tempb2=*(ptrb4+tempb1);
        if((tempb2==0x5a) && ((*msg2).wendb==0x00))
	    {  /*  B�������б�  */
	    	tempb1++;
   	    	  tempb2=*(ptrb4+tempb1);
	          (*msg2).wendb=tempb2;
	    	tempb1++;
   	    	  tempb2+=*(ptrb4+tempb1);
              if(tempb2==0xff)
              {
              	tempb1++;
   	    	    tempb2=*(ptrb4+tempb1);
                if(tempb2!=0xfd) (*msg2).wendb=0x00;
              }
              else (*msg2).wendb=0x00;
    	}

	    tempb2=*(ptrb4+tempb1);
        if((tempb2==0xa5) && ((*msg2).wenda==0x00))
	    {  /*  A�������б�  */
	    	tempb1++;
   	    	  tempb2=*(ptrb4+tempb1);
	          (*msg2).wenda=tempb2;
	    	tempb1++;
   	    	  tempb2+=*(ptrb4+tempb1);
              if(tempb2==0xff)
              {
              	tempb1++;
   	    	    tempb2=*(ptrb4+tempb1);
                if(tempb2!=0xfd) (*msg2).wenda=0x00;
              }
              else (*msg2).wenda=0x00;
    	}
		tempb1++;
	}
	 
    tempb1=0x00;   /* ~Ϊ��λȡ����!Ϊ�߼��ǣ������ */
     /* Ϊ1��Ч,6/C����Ч��5/B��?��4/A��?��2/C���,1/B��?,0/A��? */
    tempb2=(*msg2).wenda;
      if(tempb2==0x00)  tempb1|=0x01;  
    tempb2=(*msg2).wendb;
      if(tempb2==0x00)  tempb1|=0x02;  
    tempb2=(*msg2).wendc;
      if(tempb2==0x00)  tempb1|=0x04;  

    if(tempb1 & 0x01) /* A��ͨѶ�����������ʶ */
    { (*msg2).rs_ea++; tempb1|=0x10; /* ͨѶ����������Ч */
    	if((*msg2).rs_ea>=100) (*msg2).rs_ea=100;
    }
      else   
      {  (*msg2).rs_ea=0; /* �˿���Ϊ0x00��������Ч��������Ч */
      	 if((*msg2).wenda & 0x0c) tempb1&=0xef;
      	   else tempb1|=0x10;  
  	  }
    if(tempb1 & 0x02) /* B��ͨѶ�����������ʶ */
    { (*msg2).rs_eb++; tempb1|=0x20;
    	if((*msg2).rs_eb>=100) (*msg2).rs_eb=100;
    }
      else            
      {  (*msg2).rs_eb=0;
      	 if((*msg2).wendb & 0x0c) tempb1&=0xdf;
      	   else tempb1|=0x20;  
  	  }
    if(tempb1 & 0x04) /* C��ͨѶ�����������ʶ */
    { (*msg2).rs_ec++; tempb1|=0x40;
    	if((*msg2).rs_ec>=100) (*msg2).rs_ec=100;
    }
      else     
      {  (*msg2).rs_ec=0;
      	 if((*msg2).wendc & 0x0c) tempb1&=0xbf;
      	   else tempb1|=0x40;  
  	  }
  	  
    (*msg2).wend=0x00;  
    if(tempb1 & 0x40)  /* ����ȷ�������ȼ���C�� > B�� > A�� */
    {
      if(tempb1 & 0x20)
      {
        if(tempb1 & 0x10);
        else (*msg2).wend=(*msg2).wenda;  
  	  } 	
      else (*msg2).wend=(*msg2).wendb;  
	}
	else (*msg2).wend=(*msg2).wendc; 
	
	if((*msg2).wend==0x00) /* ��ʱ����Ч�����򲻸ı乤�� */
	  (*msg2).wend=(*msg1).snow;
	if((tempb1 & 0x70)!=0x70) 
	  ( *msg2).rs_er=0;      /* �оͲ�Ϊ�� */
    else 
	  { (*msg2).rs_er++; /* ��������Ч�����ʱ */
	  	if((*msg2).rs_er>=100)
	  	{ (*msg2).rs_er=100;(*msg2).wend=0x00;}
	  }

    if((*msg2).rs_er>=100) (*msg1).ec1|=0x80; /* ͨѶ�����ʶ */
      else                 (*msg1).ec1&=0x7f;
    if((*msg2).rs_ec>=100) (*msg1).ec1|=0x40;
      else                 (*msg1).ec1&=0xbf;
    if((*msg2).rs_eb>=100) (*msg1).ec1|=0x20;
      else                 (*msg1).ec1&=0xdf;
    if((*msg2).rs_ea>=100) (*msg1).ec1|=0x10;
      else                 (*msg1).ec1&=0xef;
      
	return;
}
/*   
   ������Ч�Լ�麯����
RS485ͨѶ������
   ��ʼ������ 
   ���պ�����
   ���ݺ�������
   ���ͺ�����
   ����׼��������
   ����У�麯����
   ��ʱ������
�¶Ȳ���������
   ��ʼ������ 
   ��λ������
   ��λ������
   ���ֽں�����
   дλ����
   д�ֽں�����
   ��ʱ������
*/   
/**/

