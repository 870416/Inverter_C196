/******************************************************
The current code is based on CSMA502_001E
dr502f1.c中函数原型申明
天津滨海线5KVA应急电源用
******************************************************/

#include _SFR_H_
#include <KC_FUNCS.H>
#include <KC_SFRS.H>
#include "data.h"  /* 参量定义，系统说明 */
#include "inits.h"
#include "proc.h"
#include "errs.h"
#include "contctr.h"


BW  bw;
UNA   tempbw0, tempbw1;
MESSAGE1 *msg1;
MESSAGE2 *msg2;
/* 通讯用，0/共用，1/发送计数，2/接收计数，rsp_c/sp_con映射 */
BYTE    rsb0, rsb1, rsb2, rsp_c;

/*  0/公用,1/开关模式,2/故障口，3/发送区首指针，4/接收区首指针  */
BYTE    *ptrb0, *PWM_out_ptr, *Ptr_8255, *send_buf_ptr, *rec_buf_ptr;

/* 0/公用，1/正弦表头，2/正弦表尾，3/时间计算表，4/时间发送表 */
UI      *ptrw0, *ptrw1, *ptrw2, *ptrw4;

BYTE    io1, io2;         /*  端口影射      */
BYTE    PWM_shutdown; /*  逆变控制变量  */
BYTE p_erro, s_erro/*0xff: No err*/, s_set;
BYTE tempb0, tempb1, tempb2;
UI   tempw0, tempw1, tempw2, tempw3, tempw4, tempw5, tempw6, tempw7;
BYTE     num2[2];    /* 分段数，0-当前，1-预备 */
BYTE     s_x1, s_ii1, s_ii2, s_ii3;
UI       Sys_tic_10ms;
BYTE     s_control, s_ok, Cur_addr_ca;
/* 7/首次计算，6/数据有效，5/当前发送区，4/未用，*/
/* 3/升压允许，2/升频    ，1/逆变允许  ，0/逆变过程，*/
/* 为提高编译效率，将位6，5单列为s_ok，Cur_addr_ca */
#define OUTPUT_DEBUG 1


//------------------------------------------------------------------------
#pragma interrupt(hso_int=3)   /* 定时用         */
#pragma interrupt(soft_int=5)  /* 逆变器控制用   */
#pragma interrupt(rs485_int=6) /* 串口中断用     */
#pragma interrupt(ext_int=7)   /* 故障处理用     */

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
		if (s_ok == (BYTE)0xff)  /* 数据有效则更换发送区，并置无效标志 */
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
			if (s_ok == (BYTE)0xff)  /* 数据有效则更换发送区，并置无效标志 */
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
	if (rsb0 & 0x20) /* 发送 */
	{
		rsb1++;
		if (rsb1 < 14)
			sbuf = *(send_buf_ptr + rsb1);
		else
		{
			E_485r;
		}
	}
	if (rsb0 & 0x40) /* 接收 */
	{
		if (rsb2 < 100)
		{
			*(rec_buf_ptr + rsb2) = sbuf;
			rsb2++;
		}
	}
	return;
}

void ext_int()  /* 故障处理 */
{
	p_erro = *Ptr_8255;/* 读故障状态 */
	p_erro |= 0x20;
	if ((p_erro & 0x07) != 0x07)  /* F1_V，F1_U，F2_A */
	{
		io1 = 0xff;
		ioport1 = io1;   /* 即封后级 */
		*(Ptr_8255 + 2) = 0x00; /* 即封前级 */
		PWM_shutdown = 0xff;      /* 屏蔽后级 */
		s_erro &= p_erro;
		return;
	}
	if ((p_erro & 0x08) == 0) /* I2H  */
	{   /* 简单处理，主程序中后级降频 */
		io1 = 0xff;
		ioport1 = io1;
	}
	if ((p_erro & 0x10) == 0) /* I1H  */
	{   /* 简单处理，主程序中后级降频 */
		*(Ptr_8255 + 2) = 0x00;
	}
	/* 如何解除前级封锁       */
	/* 如何解除后级封锁       */
	/* 如何与显示、通讯相配   */
	return;
}




void main()   /* 主程序 */
{
	unsigned char port = 0;
	/*  80C196KC初始化，芯片配置  */
	init196();
	/*  8255初始化  */
	init8255();
	/*  启动延时1S，影响3525软启动，取消  */
	/*    delay1s();  */
	/* RAM检查 */
	/*    checkram(); */
	/*  1820初始化  */
	init1820();
	/*  串行通讯初始化  */
	init485();
	/*  系统初始化  */
	initsys();

	for (port = 0; port < 8; port++)
		read_adport(port);

	while (1)
	{
		/*  看门狗  */
		E_watchdog;
		/*  系统定时45分  */
		/*  	    delay45m();   */
		/*  信息搜集   */
		/*  八路参数采样（分散），查询方式   */
		if (Sys_tic_10ms != (*msg1).bt10)
		{
			(*msg1).bt10 = Sys_tic_10ms;
			tempb0 = (*msg1).bt10 % 5;
			switch (tempb0)
			{
			case 0:read_adport(0); break;	//P0.0:IM1, 通风机电流;          1V - 0.25A
			case 1:read_adport(1); break;	//P0.1:IM2, 冷凝机电流;          1V - 0.5A
			case 2:read_adport(2); break;	//P0.2:IM3, 压缩机电流;          1V - 2.5A
			case 3:read_adport(3); break;	//P0.3:IM4, 电热器电流;          1V - 2.5A
			default:Motor_petection(); break;
			}
		}

		//P0.7:U2C, 中间直流电压;        1V - 200V
		//P0.6:I2C, 中间直流电流;        1V - 4A
		//P0.5:U1B, 输入直流电压;        1V - 40V
		//P0.4:I1C, 输入直流电流;        1V - 20A
		read_adport(0x04);
		read_adport(0x05);
		read_adport(0x06);
		read_adport(0x07);

		//----------------------------------------------------------------


		/*  系统对策表 */
		/* 两级逆变控制 */
		Exception_handler();


		/*  当前工作频率确定  */
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
				(*msg1).cur_freqc -= 5;  /* 故障时降频加速 */

			(*msg1).cur_freqc -= (*msg1).addl;	     /* 工况转换降频 */

			if (((*msg1).cur_mode & 0x03) == 0)
				(*msg1).cur_freqc = F_STARTC;   /* 停机状态频率回到起点 */
#endif

			if ((*msg1).cur_freqc >= FREQ_MAX_LIMIT)
				(*msg1).cur_freqc = FREQ_MAX_LIMIT; /* 运行频率限制 */


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
			{ /* 信息处理、工况控制 */
				treatmess();
				break;
			}
			case 2:
			{/* 电机分断、数码显示 */
				mc_off((*msg1).cur_mode);
				disp();
				break;
			}
			case 3:
			{ /* 先分后合，防止意外 */
				mc_off((*msg1).cur_mode);
				mc_on((*msg1).cur_mode);
				break;
			}
			case 4:
			{ /* 通讯数据处理 */
				proc_RS485_buff();
				break;
			}
			default:break;
			}
		}


		/*  数据处理   */
		/*  当前时间模式计算  */
		if (s_ok == 0x00)  /* 数据无效则重新计算 */
		{
			if ((*msg1).cur_freqc != (*msg1).pre_freqc)  /* 不变不算 */
			{
				(*msg1).pre_freqc = (*msg1).cur_freqc;
				time_cal(); //change ADDR_CA1; set up s_ok = 0xff
			}
		}

		if (s_control & 0x80) //First time calculation
		{
			s_control &= 0x3f;	/* 清首次计算标志，置数据无效 */
			
			// This is the first time to initialize software timer0 PWM output.
			// So when the interrupt's triggered, everything is ready to output.
			disable();
			hso_command = 0x38; //bit6:0 timer1. bit5:1 set the pin. bit4:1 enable intterupt. 1000: software timer0
			hso_time = timer1 + 0x04;
			enable();
		}
	}
}
