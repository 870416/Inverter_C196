#include "data.h"
#include "errs.h"


/* 非中断故障处理函数；两级逆变控制*/
void Exception_handler()
{
	/*  故障口数据读取         */
	p_erro = *(Ptr_8255 + 0);
	p_erro |= 0x20;     /* PA.5位悬而未用  */
	tempb0 = s_control;
	tempb0 |= 0x0f;
	if ((*msg1).dt45m >= 45)
		tempb0 &= 0xf1; /* 定时到,禁止工作 */
	tempb1 = p_erro;
	tempb1 &= s_erro;
	(*msg1).ec2 = tempb1;   /* 故障信号外传 */
	if ((s_erro & 0x07) != 0x07)
		(*msg1).cur_mode &= 0x0c; /* IGBT故障停机 */
	if (tempb1 & 0x80)
		;
	else
	{
		tempb0 &= 0xfb;
	} /* .7，   前级欠压时允许升压   */
	/*
	if(tempb1 & 0x40);
	else { tempb0&=0xf7;}
	*/
	/* .6，   后级过压   */
	/* 与CPU自采数据相重叠，引起前级频繁关断，不利于变压器和电感 */
	if (tempb1 & 0x10);
	else { tempb0 &= 0xf3; } /* .4，   前级过流   */
	if (tempb1 & 0x08);
	else { tempb0 &= 0xfb; } /* .3，   后级过流   */
	if (tempb1 & 0x04);
	else { tempb0 &= 0xf1; } /* .2，   IGBT故障   */
	if (tempb1 & 0x02);
	else { tempb0 &= 0xf1; } /* .1，   IGBT故障   */
	if (tempb1 & 0x01);
	else { tempb0 &= 0xf1; } /* .0，   IGBT故障   */
	tempw0 = (*msg1).u2_h;
	tempw0 <<= 8;
	tempw0 += (*msg1).u2_l;
	/*  if((*msg1).u1<= 40) { tempb0&=0xf3;}  */ /* 前级电压不足！   */
	if (s_set != 0xfd)  /* 调试时不控 */
	{
		if ((*msg1).u1 >= 145) { tempb0 &= 0xf3; }   /* 前级电压过高！   */
		if (tempw0 <= 300)     { tempb0 &= 0xfb; }   /* 中间电压不足！   */
	}
	if (tempw0 >= 580) { tempb0 &= 0xf7; }   /* 中间电压过高！   */
	if (tempw0 >= 600) { tempb0 &= 0xf1; }   /* 中间电压危险！   */
	if ((*msg1).i1 >= 75) { tempb0 &= 0xf3; }   /* 前级电流危险！   */
	if ((*msg1).i2 >= 20) { tempb0 &= 0xf1; }   /* 中间电流危险！   */
	tempb1 = 0x00;  /* 前级控制 */
	if (tempb0 & 0x08)
		tempb1 = 0x01;
	*(Ptr_8255 + 3) = tempb1;

	/* 曾因对PWM_shutdown多次置数，使控制产生不应有的毛刺，用过渡变量解决！！！
	因有中断的原因，控制变量应一言而决 */
	if (tempb0 & 0x02) 
		tempb1 = 0x00;
	else 
		tempb1 = 0xff;
	if ((*msg1).cur_freqc == F_STARTC) tempb1 = 0xff;
	PWM_shutdown = tempb1;
	s_control = tempb0;
	return;
}




void treatmess()  /* 信息处理、工况控制、电机控制 */
{
	/* 确定目标频率、控制目标频率 */
	tempb1 = (*msg2).ext_setting_mode;
	tempb1 &= 0xf0;
	tempb1 >>= 4;

	(*msg1).target_freq = 50 - tempb1;
	(*msg1).target_freqc = (*msg1).target_freq * 10;
	/* 确定各电机目标工况 */
	tempb1 = (*msg2).ext_setting_mode;
	(*msg1).target_mode = tempb1 & 0x0f;

	/* 确定各电机允许工况 */
	tempb2 = (*msg1).target_mode;
	tempb2 &= 0x03;

	tempb1 = (*msg1).ec1;
	/* 通风故障停机 */
	if (tempb1 & 0x01)
		tempb2 = 0x00;

	/* 制冷故障制冷转通风 */
	if (tempb1 & 0x06)
		if (tempb2 == 0x02)
			tempb2 = 0x01;

	/* 制热故障制热转通风 */
	if (tempb1 & 0x08)
		if (tempb2 == 0x03)
			tempb2 = 0x01;

	//--------------------------
	//	Fanning counter.
	tempw0 = (*msg1).t0a;         /* 防止短时反复加载 */
	if (((*msg1).cur_mode & 0x03) == 0x01)
		tempw0++;
	else
		tempw0 = 0;            /* 通风工况计时 */
	if (tempw0 > 500)
		tempw0 = 500;   /* 50S */
	if ((tempb2 & 0x02) > ((*msg1).cur_mode & 0x02))
		if (tempw0 < 500)
			tempb2 = 0x01;  //通风不足不许加载 Locking in the fan mode
	(*msg1).t0a = tempw0;
	//-------------------------


	if ((tempb2 & 0x02) >((*msg1).cur_mode & 0x02)) //target is cooling mode but current is not
	{
		if ((*msg1).cur_freq >= 6) /* 频率>6Hz不许加载 */
		{
			tempb2 = 0x01;
			(*msg1).addl = 0x02;    /* 降频 */
		}
	}
	else
		(*msg1).addl = 0x00;  /* 升频 */

	tempb1 = (*msg1).target_mode;  /* 取端口信息 */
	tempb1 &= 0x0c;	//extract 1100
	tempb2 |= tempb1;     /* 取工况信息 */
	(*msg1).target_modem = tempb2;
	/* 确定当前工况 */
	tempb1 = (*msg1).target_modem & 0x03;   /* 允许工况 */

	tempb2 = (*msg1).cur_mode & 0x03;   /* 当前工况 */
	(*msg1).cur_mode &= 0xfc; /* 只保留当前端口 */
	switch (tempb2)
	{
	case 0x00: /* 停机-只转通风      */
		if (tempb1 > 0x00)  (*msg1).cur_mode |= 0x01;
		break;
	case 0x01: /* 通风转任意工况 */
		(*msg1).cur_mode |= tempb1;
		break;
	case 0x02: /* 制冷不转制热 */
		if (tempb1 == 0x03) (*msg1).cur_mode |= 0x01;
		else  (*msg1).cur_mode |= tempb1;
		break;
	case 0x03: /* 制热不转制冷 */
		if (tempb1 == 0x02) (*msg1).cur_mode |= 0x01;
		else  (*msg1).cur_mode |= tempb1;
		break;
	default:break;
	}
	tempb1 = (*msg1).target_modem; /* 换端停机，电机保护数据复位 */
	tempb2 = (*msg1).cur_mode;
	if ((tempb1 & 0x0c) != (tempb2 & 0x0c))
	{
		tempb1 &= 0x0c;
		(*msg1).cur_mode = tempb1; /* 换端停机，停机-启动变化亦然 */

		(*msg1).ec1 &= 0xf0;  /* 电机保护数据复位 */
		(*msg1).t0d = 0x00;
		(*msg1).t1d = 0x00;
		(*msg1).t2d = 0x00;
		(*msg1).t3d = 0x00;
		(*msg1).m0a = 0x00;
		(*msg1).m1a = 0x00;
		(*msg1).m2a = 0x00;
		(*msg1).m3a = 0x00;
	}
	/* 停机复位:为软启动准备 */
	tempb1 = (*msg1).cur_mode;
	tempb1 &= 0x03;
	if (tempb1 == 0)   { (*msg1).t0a = 0x00;    (*msg1).addl = 0x02; }
	/* 确定电机工况 */
	switch ((*msg1).cur_mode & 0x03)
	{
	case  0:tempb2 = 0x00; break;
	case  1:tempb2 = 0x10; break;
	case  2:tempb2 = 0x70; break;
	case  3:tempb2 = 0x90; break;
	default:break;
	}
	tempb1 = (*msg1).cur_mode;
	tempb1 &= 0x0f;
	tempb1 |= tempb2;
	(*msg1).cur_mode = tempb1;
	return;
}




void Motor_petection()   /* 电机过流保护、定时、恢复(20min)  */
{
	/* 通风机连续工作时间 */
	/*  (*msg1).t0a++;
	if((*msg1).t0a>=1000)  (*msg1).t0a=1000;
	*/
	if ((*msg1).m0a >= I017) { (*msg1).m0a = 0; (*msg1).ec1 |= 0x01; }
	if ((*msg1).m1a >= I117) { (*msg1).m1a = 0; (*msg1).ec1 |= 0x02; }
	if ((*msg1).m2a >= I217) { (*msg1).m2a = 0; (*msg1).ec1 |= 0x04; }
	if ((*msg1).m3a >= I317) { (*msg1).m3a = 0; (*msg1).ec1 |= 0x08; }

	//UI     t0d;   /* 通风机保护动作计时，单位50mS，20min恢复,下同 */
	//UI     t1d;   /* 冷凝机保护动作计时， */
	//UI     t2d;   /* 压缩机保护动作计时， */
	//UI     t3d;   /* 电加热保护动作计时， */
	tempb1 = (*msg1).ec1;
	if (tempb1 & 0x01) (*msg1).t0d++;
	if (tempb1 & 0x02) (*msg1).t1d++;
	if (tempb1 & 0x04) (*msg1).t2d++;
	if (tempb1 & 0x08) (*msg1).t3d++;

	//	If any over-current is over time, mark error into ec1
	if ((*msg1).t0d >= (UI)24000) { (*msg1).t0d = 0; (*msg1).ec1 &= 0xfe; }
	if ((*msg1).t1d >= (UI)24000) { (*msg1).t1d = 0; (*msg1).ec1 &= 0xfd; }
	if ((*msg1).t2d >= (UI)24000) { (*msg1).t2d = 0; (*msg1).ec1 &= 0xfb; }
	if ((*msg1).t3d >= (UI)24000) { (*msg1).t3d = 0; (*msg1).ec1 &= 0xf7; }

	return;
}