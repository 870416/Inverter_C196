/*
da502f1.c中函数原型申明
配用  WJDA502_002
天津滨海线5KVA应急电源用
*/

#include "data.h"
#include "inits.h"
/*
da502f1.c中函数原型申明
配用  CSMA502_001E，2003/04/10
改动之处：
1、欠压时只降频不封前级；
2、每次加载前需通风50S。
*/

/**/
/* 80C196KC初始化； */
void init196()
{   /* 重设堆栈 */
	/*
	ptrw0=(UI *)0x18;
	*ptrw0=0x100;
	*/
	io1 = 0xff;
	ioport1 = io1;
	io2 = 0xff;
	ioport2 = io2;
	disable();
	int_mask = 0x00;
	imask1 = 0x00;
	int_pending = 0x00;  /*  禁止中断   */
	ioc1 = 0x70;  /* 一步到位，避免分步进行时的读-改-写 */

	wsr = 0x01;
	ad_result_hi = 0xec;/* 1110,1100 实际为AD_TIME,20μS */
	wsr = 0x00;     /*  水平窗口0     */
	ioc2 = 0x08;
	;
	return;
}

/*  8255初始化； */
void init8255()
{
	Ptr_8255 = (BYTE *)ADDR_8255;
	*(Ptr_8255 + 3) = 0x90;  /* (A：输入，B、C输出) */
	*(Ptr_8255 + 1) = 0xff;  /* 数码管全亮          */
	*(Ptr_8255 + 3) = 0x00;  /* 禁止前级逆变,PC.0=0 */
	return;
}

/* 串行通讯初始化； */
void init485()
{
	disable();
	/* 曾因写ioport0而引起通讯错误，实际是写的波特率！！！*/
	baud_rate = 0x6b;  /* 16MHz/(bt(bps)*16)-1,cfH-4800;67H-9600 */
	/* 为配合操纵板，波特率实际为9259 */
	baud_rate = 0x80;  /* 高位为1选内部时钟，6M-0x26    */
	rsp_c = 0x09;      /* 曾因直接对sp_con进行操作而引起通讯错*/
	sp_con = rsp_c;    /* MODE=1               */
	int_mask |= 0x40;  /* 允许串行通讯中断     */
	rsb0 = 0;
	rsb1 = 0;
	rsb2 = 0;
	send_buf_ptr = (BYTE *)ADDR_MEM1;
	rec_buf_ptr = (BYTE *)ADDR_MEM2;
	E_485t;
	enable();
	return;
}

/* 1820初始化；*/
void init1820()
{
	return;
}

/* 系统初始化；*/
void initsys()
{
	PWM_out_ptr = (BYTE *)ADDR_MOD;  /* 开关模式指针 */
	Ptr_8255 = (BYTE *)ADDR_8255; /* 故障口指针   */

	/* 停机备用状态 */
	msg1 = (MESSAGE1 *)ADDR_MEM1;
	(*msg1).head1 = 0x55;
	(*msg1).u1 = 0x00;
	(*msg1).u2_l = 0x00;
	(*msg1).u2_h = 0x00;
	(*msg1).i1 = 0x00;
	(*msg1).i2 = 0x00;
	(*msg1).target_freq = 50;
	(*msg1).cur_freq = (*msg1).target_freq / 10;
	(*msg1).target_mode = 0x00;
	(*msg1).cur_mode = 0x00;
	(*msg1).ec1 = 0x00;
	(*msg1).ec2 = 0xff;
	(*msg1).target_modem = 0x00;

	(*msg1).end1 = 0xfd;
	(*msg1).target_freqc = (*msg1).target_freq * 10;
	(*msg1).cur_freqc = F_STARTC;
	(*msg1).pre_freqc = (*msg1).cur_freqc - 10;
	(*msg1).addl = 0x02;
	(*msg1).t0a = 0x00;

	(*msg1).t0d = 0x00;
	(*msg1).t1d = 0x00;
	(*msg1).t2d = 0x00;
	(*msg1).t3d = 0x00;

	(*msg1).im0 = 0x00;
	(*msg1).im1 = 0x00;
	(*msg1).im2 = 0x00;
	(*msg1).im3 = 0x00;

	(*msg1).m0a = 0x00;
	(*msg1).m1a = 0x00;
	(*msg1).m2a = 0x00;
	(*msg1).m3a = 0x00;

	(*msg1).bt10 = 0x00;
	(*msg1).bt20 = 0x00;
	(*msg1).bt1m = 0x00;
	(*msg1).dt45m = 0x00;

	msg2 = (MESSAGE2 *)ADDR_MEM2;
	for (tempb1 = 0; tempb1 < 100; tempb1++)  
		*(rec_buf_ptr + tempb1) = 0x00;

	/* 定时中断，无法用锁定功能  */
	Sys_tic_10ms = 0;
	int_mask |= 0x08;
	disable();
	hso_command = 0x13;      /* 0001,0101,HSO.3=0   */
	hso_time = timer1 + 10000; /* 10mS                */
	enable();

	/*  拨码开关状态读取       */
	tempb0 = ioport2;  /* p2.4-p2.3 */
	tempb0 &= 0x18;
	s_set = 0xff;
	switch (tempb0)  /* 01/调试状态，其余/正常状态 */
	{
	case  0x00: s_set &= 0xf5; break;
	case  0x08: s_set &= 0xfd; break;
	case  0x10: s_set &= 0xf7; break;
	default:             break;
	}

	s_erro = 0xff;
	s_control = 0xb4;   /* 1011,0100 ，曾因写成0xc4而出错！！！！ */
	s_ok = 0x00;
	Cur_addr_ca = 0xff;
	PWM_shutdown = 0xff;

	int_mask |= 0xa0;   /* 1010,0000,      */
	int_pending = 0x00;

	E_watchdog;
	return;
}









/* 中断函数， */

/*  启动延时1S  */

void  delay1s()
{
	unsigned int temp;
	for (temp = 0; temp < 60000; temp++)
	{
		tempb0 += 16;
		tempb0 -= 16;
		tempb0 += 16;
		tempb0 -= 16;
	}
	return;
}


/* RAM检查 */
/*
void  checkram()
{
ptrw0=(UI *)ADDR_MEM1;
for(tempw0=0;tempw0<4096;)
{  *ptrw0=tempw0;
while((tempw1=*ptrw0) != tempw0) *ptrw0=tempw0;
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
*(Ptr_8255+1)=xx2;
}
*/

/*  系统定时45分  */
void  delay45m()
{
	tempbw0.ll = Sys_tic_10ms;
	tempbw0.ll /= 6000;
	if (tempbw0.bw.lo != (*msg1).bt1m)
	{
		(*msg1).bt1m = tempbw0.bw.lo;
		(*msg1).dt45m += 1;   /* 调试时设定为5min */
	}
	if ((*msg1).dt45m > 45)  (*msg1).dt45m = 45;
	return;

}

/* 数码管显示驱动函数；*/
void disp()
{  /* 正常显示频率，否则显示故障 */
	BYTE err_num = p_erro & s_erro;
	tempw0 = Sys_tic_10ms;
	tempw0 /= 40;  /*  400mS  */
	tempw0 %= 4;
	
	if (tempw0 == 0)
	{
		if (err_num == 0xff) //NO error
		{
			if ((*msg1).cur_freqc == F_STARTC)
			{
				(*msg1).dis0 = 18;
				(*msg1).dis1 = 20;
				(*msg1).dis2 = 18;

			}
			else
			{
				(*msg1).dis0 = 15;
				(*msg1).dis1 = (*msg1).cur_freq / 10;
				(*msg1).dis2 = (*msg1).cur_freq % 10;
			}
		}
		else
		{
			(*msg1).dis0 = 14; //E error
			(*msg1).dis1 = err_num / 16;
			(*msg1).dis2 = err_num % 16;
		}
		(*msg1).dis3 = 20;
	}

	switch (tempw0)
	{
	case 0:tempb1 = (*msg1).dis0; break;
	case 1:tempb1 = (*msg1).dis1; break;
	case 2:tempb1 = (*msg1).dis2; break;
	default:tempb1 = (*msg1).dis3; break;
	}
	ptrb0 = (BYTE *)ADDR_DIS;
	err_num = *(ptrb0 + tempb1);
	*(Ptr_8255 + 1) = err_num;
	;
	return;
}


/*
数据有效性检查函数。
RS485通讯函数：
初始化函数
接收函数，
数据后处理函数，
发送函数，
数据准备函数，
数据校验函数。
定时函数。
温度采样函数：
初始化函数
复位函数，
读位函数，
读字节函数，
写位函数
写字节函数，
定时函数。
*/
/**/

