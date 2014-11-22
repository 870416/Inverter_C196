/******************************************************
The current code is based on CSMA502_001E
dr502f1.c�к���ԭ������
��������5KVAӦ����Դ��
******************************************************/

#include _SFR_H_
#include <KC_FUNCS.H>
#include <KC_SFRS.H>
#include "data.h"  /* �������壬ϵͳ˵�� */
#include "inits.h"
#include "proc.h"
#include "errs.h"
#include "contctr.h"


BW  bw;
UNA   tempbw0, tempbw1;
MESSAGE1 *msg1;
MESSAGE2 *msg2;
/* ͨѶ�ã�0/���ã�1/���ͼ�����2/���ռ�����rsp_c/sp_conӳ�� */
BYTE    rsb0, rsb1, rsb2, rsp_c;

/*  0/����,1/����ģʽ,2/���Ͽڣ�3/��������ָ�룬4/��������ָ��  */
BYTE    *ptrb0, *PWM_out_ptr, *Ptr_8255, *send_buf_ptr, *rec_buf_ptr;

/* 0/���ã�1/���ұ�ͷ��2/���ұ�β��3/ʱ������4/ʱ�䷢�ͱ� */
UI      *ptrw0, *ptrw1, *ptrw2, *ptrw4;

BYTE    io1, io2;         /*  �˿�Ӱ��      */
BYTE    PWM_shutdown; /*  �����Ʊ���  */
BYTE p_erro, s_erro/*0xff: No err*/, s_set;
BYTE tempb0, tempb1, tempb2;
UI   tempw0, tempw1, tempw2, tempw3, tempw4, tempw5, tempw6, tempw7;
BYTE     num2[2];    /* �ֶ�����0-��ǰ��1-Ԥ�� */
BYTE     s_x1, s_ii1, s_ii2, s_ii3;
UI       Sys_tic_10ms;
BYTE     s_control, s_ok, Cur_addr_ca;
/* 7/�״μ��㣬6/������Ч��5/��ǰ��������4/δ�ã�*/
/* 3/��ѹ����2/��Ƶ    ��1/�������  ��0/�����̣�*/
/* Ϊ��߱���Ч�ʣ���λ6��5����Ϊs_ok��Cur_addr_ca */
#define OUTPUT_DEBUG 1


//------------------------------------------------------------------------
#pragma interrupt(hso_int=3)   /* ��ʱ��         */
#pragma interrupt(soft_int=5)  /* �����������   */
#pragma interrupt(rs485_int=6) /* �����ж���     */
#pragma interrupt(ext_int=7)   /* ���ϴ�����     */

void hso_int() /* 120S */
{
	hso_command = 0x13;    //bit6:0 timer1. bit5:0 pin= 0. bit4:1 enable interrupt; 0011: HSO.3  
	hso_time = timer1 + 9997; /* 10mS,10000-3      */
	enable();
	int_mask = 0x20;
	Sys_tic_10ms++;
	if (Sys_tic_10ms >= 12000)
		Sys_tic_10ms = 0;
	return;
}

/********************************************
//	PWM output
********************************************/
void soft_int()
{
	UI timer_interval = *ptrw4;
	timer_interval -= AVG_ANSWER;
	ptrw4++;

	// output
	s_x1 = *PWM_out_ptr;
	s_x1 |= PWM_shutdown;
	io1 |= s_x1;
	ioport1 = io1;

	if (s_ii3 != (BYTE)0x02) 
	{ 
		s_ii3 ++;  
	}
	else                
	{ 
		s_ii3 = 0; 
		s_ii2++; 
		PWM_out_ptr -= 3; 
	}
	hso_command = 0x38;  //bit6:0 timer1. bit5:1 set the pin. bit4:1 enable intterupt. 1000: software timer0
	hso_time = timer1 + timer_interval;
	PWM_out_ptr++;

	io1 = s_x1;
	ioport1 = io1;

	if (s_ii2 == num2[0])  //2 * num2[0]
	{
		s_ii2 = 0;
		s_ii1++;
		PWM_out_ptr += 3;
		if (s_ok == (BYTE)0xff)  /* ������Ч�������������������Ч��־ */
		{
			Cur_addr_ca ^= 0xff; 
			s_ok = 0x00; 
			num2[0] = num2[1];
		}
		if (Cur_addr_ca == (BYTE)0xff)  
			ptrw4 = (UI *)ADDR_CA1;
		else
			ptrw4 = (UI *)ADDR_CA0;
	}

	if (s_ii1 == (BYTE)0x06) // 6*2*num2[0]
	{ 
		s_ii1 = 0; 
		PWM_out_ptr -= 18; //finish one cycle
	}

//Meaningless
	while ( *ptrw4 == 0)
	{
		PWM_out_ptr++;
		ptrw4++;
		if (s_ii3 != (BYTE)0x02) 
		{ 
			s_ii3++; 
		}
		else                
		{ 
			s_ii3 = 0; s_ii2++; PWM_out_ptr -= 3; 
		}

		if (s_ii2 == num2[0])  //2 * num2[0]
		{
			s_ii2 = 0;
			s_ii1++;
			PWM_out_ptr += 3;
			if (s_ok == (BYTE)0xff)  /* ������Ч�������������������Ч��־ */
			{
				Cur_addr_ca ^= 0xff;
				s_ok = 0x00;
				num2[0] = num2[1];
			}
			if (Cur_addr_ca == (BYTE)0xff)
				ptrw4 = (UI *)ADDR_CA1;
			else
				ptrw4 = (UI *)ADDR_CA0;
		}

		if (s_ii1 == (BYTE)0x06) // 6*2*num2[0]
		{
			s_ii1 = 0;
			PWM_out_ptr -= 18; //finish one cycle
		}
	}

	return;
}

void rs485_int()
{
	enable();
	int_mask = 0x20;
	rsb0 = sp_stat;
	if (rsb0 & 0x20) /* ���� */
	{
		rsb1++;
		if (rsb1 < 14)
			sbuf = *(send_buf_ptr + rsb1);
		else
		{
			E_485r;
		}
	}
	if (rsb0 & 0x40) /* ���� */
	{
		if (rsb2 < 100)
		{
			*(rec_buf_ptr + rsb2) = sbuf;
			rsb2++;
		}
	}
	return;
}

void ext_int()  /* ���ϴ��� */
{
	p_erro = *Ptr_8255;/* ������״̬ */
	p_erro |= 0x20;
	if ((p_erro & 0x07) != 0x07)  /* F1_V��F1_U��F2_A */
	{
		io1 = 0xff;
		ioport1 = io1;   /* ����� */
		*(Ptr_8255 + 2) = 0x00; /* ����ǰ�� */
		PWM_shutdown = 0xff;      /* ���κ� */
		s_erro &= p_erro;
		return;
	}
	if ((p_erro & 0x08) == 0) /* I2H  */
	{   /* �򵥴����������к󼶽�Ƶ */
		io1 = 0xff;
		ioport1 = io1;
	}
	if ((p_erro & 0x10) == 0) /* I1H  */
	{   /* �򵥴����������к󼶽�Ƶ */
		*(Ptr_8255 + 2) = 0x00;
	}
	/* ��ν��ǰ������       */
	/* ��ν���󼶷���       */
	/* �������ʾ��ͨѶ����   */
	return;
}




void main()   /* ������ */
{
	unsigned char port = 0;
	/*  80C196KC��ʼ����оƬ����  */
	init196();
	/*  8255��ʼ��  */
	init8255();
	/*  ������ʱ1S��Ӱ��3525��������ȡ��  */
	/*    delay1s();  */
	/* RAM��� */
	/*    checkram(); */
	/*  1820��ʼ��  */
	init1820();
	/*  ����ͨѶ��ʼ��  */
	init485();
	/*  ϵͳ��ʼ��  */
	initsys();

	for (port = 0; port < 8; port++)
		read_adport(port);

	while (1)
	{
		/*  ���Ź�  */
		E_watchdog;
		/*  ϵͳ��ʱ45��  */
		/*  	    delay45m();   */
		/*  ��Ϣ�Ѽ�   */
		/*  ��·������������ɢ������ѯ��ʽ   */
		if (Sys_tic_10ms != (*msg1).bt10)
		{
			(*msg1).bt10 = Sys_tic_10ms;
			tempb0 = (*msg1).bt10 % 5;
			switch (tempb0)
			{
			case 0:read_adport(0); break;	//P0.0:IM1, ͨ�������;          1V - 0.25A
			case 1:read_adport(1); break;	//P0.1:IM2, ����������;          1V - 0.5A
			case 2:read_adport(2); break;	//P0.2:IM3, ѹ��������;          1V - 2.5A
			case 3:read_adport(3); break;	//P0.3:IM4, ����������;          1V - 2.5A
			default:Motor_petection(); break;
			}
		}

		//P0.7:U2C, �м�ֱ����ѹ;        1V - 200V
		//P0.6:I2C, �м�ֱ������;        1V - 4A
		//P0.5:U1B, ����ֱ����ѹ;        1V - 40V
		//P0.4:I1C, ����ֱ������;        1V - 20A
		read_adport(0x04);
		read_adport(0x05);
		read_adport(0x06);
		read_adport(0x07);

		//----------------------------------------------------------------


		/*  ϵͳ�Բ߱� */
		/* ���������� */
		Exception_handler();


		/*  ��ǰ����Ƶ��ȷ��  */
		tempw0 = Sys_tic_10ms;
		tempw0 >>= 1;
		if (tempw0 != (*msg1).bt20)
		{
			(*msg1).bt20 = tempw0;

#if OUTPUT_DEBUG
			(*msg1).cur_freqc = 500;
#else
			
			if (s_control & 0x04) // 0x04: Freq Raising enable
				(*msg1).cur_freqc++;
			else
				(*msg1).cur_freqc -= 5;  /* ����ʱ��Ƶ���� */

			(*msg1).cur_freqc -= (*msg1).addl;	     /* ����ת����Ƶ */

			if (((*msg1).cur_mode & 0x03) == 0)
				(*msg1).cur_freqc = F_STARTC;   /* ͣ��״̬Ƶ�ʻص���� */
#endif

			if ((*msg1).cur_freqc >= FREQ_MAX_LIMIT)
				(*msg1).cur_freqc = FREQ_MAX_LIMIT; /* ����Ƶ������ */


			if ((*msg1).cur_freqc <= F_STARTC) //F_startc = minimum freq
				(*msg1).cur_freqc = F_STARTC;

			(*msg1).cur_freq = (*msg1).cur_freqc / 10;

			if ((*msg1).cur_mode & 0x01) // fanning
			{
				disable();
				hso_command = 0x14;    /* 0001,0100,HSO.4=0   */
				//hso_time = timer1 + 9997; /* 10mS,10000-3      */
				enable();
			}
			else
			{
				disable();
				hso_command = 0x34;    /* 0001,0100,HSO.4=0   */
				//hso_time = timer1 + 9997; /* 10mS,10000-3      */
				enable();
			}


			tempb0 = (*msg1).bt20 % 5;
			switch (tempb0)
			{
			case 0:
			{
				E_485t; rsb1 = 0; sbuf = *(send_buf_ptr + rsb1); rsb2 = 10; break;
			}
			case 1:
			{ /* ��Ϣ������������ */
				treatmess();
				break;
			}
			case 2:
			{/* ����ֶϡ�������ʾ */
				mc_off((*msg1).cur_mode);
				disp();
				break;
			}
			case 3:
			{ /* �ȷֺ�ϣ���ֹ���� */
				mc_off((*msg1).cur_mode);
				mc_on((*msg1).cur_mode);
				break;
			}
			case 4:
			{ /* ͨѶ���ݴ��� */
				proc_RS485_buff();
				break;
			}
			default:break;
			}
		}


		/*  ���ݴ���   */
		/*  ��ǰʱ��ģʽ����  */
		if (s_ok == 0x00)  /* ������Ч�����¼��� */
		{
			if ((*msg1).cur_freqc != (*msg1).pre_freqc)  /* ���䲻�� */
			{
				(*msg1).pre_freqc = (*msg1).cur_freqc;
				time_cal(); //change ADDR_CA1; set up s_ok = 0xff
			}
		}

		if (s_control & 0x80) //First time calculation
		{
			s_control &= 0x3f;	/* ���״μ����־����������Ч */
			
			// This is the first time to initialize software timer0 PWM output.
			// So when the interrupt's triggered, everything is ready to output.
			disable();
			hso_command = 0x38; //bit6:0 timer1. bit5:1 set the pin. bit4:1 enable intterupt. 1000: software timer0
			hso_time = timer1 + 0x04;
			enable();
		}
	}
}
