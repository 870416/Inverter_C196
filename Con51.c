#include < reg51.h >
/****************************************************************
说明:  空调电源操纵板用，DA50202.C,曹庆祥,2002/11/11
为了解决压缩机启动后的干扰问题而改动，2002/12/27，
1）放宽接收标准；
2）改一次发送为两次发送；
3）标签为CSMA50202E或CSMA50202F。
为了解决运行过程中状态波动问题（如压力保护误动作）而改动，03/06/21
1）增加变量status_o，num_m；
2）采样连续四次相同方为有效信号(4*15=60mS)；
3）标签为CSMA50202E*。
晶振:16MHz,状态周期:1/8μS,HSO周期:1μS.
P0口:
P07:X24XX时钟线;
P06:X24XX数据线;
P05:通讯控制线，高发低收;
P04:DS1820数据线;
P03:数码管L1控制，高亮;
P02:数码管L2控制，高亮;
P01:数码管L3控制，高亮;
P00:发光管控制;
P1口:
P17:端口选择线低一端高二端；
P16:故障信号线    ，低有效；
P15:二端启动信号线，低有效；
P14:一端启动信号线，低有效；
P13:压力保护信号线，  高有效;
P12:温控信号线    ，  高有效;
P11:制冷信号线    ，低有效;
P10:通风信号线    ，低有效;
P2口：
P2.0-7:数码管和发光管正端控制线
显示驱动,对照表(数据/显示数字)
0/3FH,1/30H,2/5BH,3/4FH,4/66H,5/6DH,6/7DH,7/07H,
8/7FH,9/6FH.A./F7H,b./FCH,C./B9H,d./DEH,E./F9H,F./F1H,
H./F6H,L./B8H,O./DCH,P./F3H,U./BEH.
发送数据定义：
7-4/频率修正，32/00和11无效、01一端、10二端，10/工况
因为PO口未加上拉电阻而致错；
因位定义有问题引起定时器不能工作(包含文件由io51.h改用reg51.h后好)。
因中断定义方式与原来使用的方式不符而使中断不能进行
*******************************************************************/

#define  BYTE     unsigned char
#define  UI       unsigned int
#define  UL       unsigned long 
#define  NULL     0
#define  BUSY     1


#define  DISABLE  IE&=0x7f
#define  ENABLE   IE|=0x80
#define  _U_      0xbe
#define  _F_      0xf1
#define  _S_      0x6d
#define  _E_      0xf9
#define  _C_      0xb9
#define  E_NUM    200
#define  L_NUM    40

/* 485控制口为P0.5 */
#define E_485R { P0&=0xdf;REN=1; }
#define E_485T { P0|=0x20;REN=0; }

/* U/U1,F/频率,S/实际工况,E/内部故障，C/通讯和电机状态 */
BYTE data disp_da[4], ta[5], temp0, temp1, temp2;
BYTE data Tic_5ms, disp_dta_switcher;
BYTE data status_o, num_m; /* 采样滤波用,同则累计，异则刷新 */

BYTE data Send_count, Rev_count;
BYTE data Send_buff[5]; /* [4]为采样状态 */
BYTE data Rev_buff[22];
BYTE data DISP_DTA[16];
BYTE Comm_watchdog;
BYTE case_switcher, disp_switcher;
BYTE Sending_Tic;
/* #define R_TF0     TCON &=0xdf */

void delay5() interrupt 1  /* 5mS定时中断 */
{
	DISABLE;
	TH0 = 0xe5; /* 每周期12T，5mS定时，16MHz */
	TL0 = 0xf5;
	TF0 = 0;
	TR0 = 1;
	Tic_5ms++;
	Tic_5ms %= 200;
	ENABLE;
	return;
}



void rs_int() interrupt 4  /* 通讯中断 */
{
	BYTE data rev_temp;
	DISABLE;
	if (TI)
	{
		TI = 0;
		Send_count++;
		if (Send_count == 4) { E_485R; Rev_count = 0; }
		else 
			SBUF = Send_buff[Send_count];
	}
	if (RI)
	{
		RI = 0;
		rev_temp = SBUF;
		if (Rev_count != 22) /* 防止意外错误，造成程序混乱 */
		{
			if ((Rev_count == 0) && (rev_temp != 0x55));
			else 
			{ 
				Rev_buff[Rev_count] = rev_temp;
				Rev_count++; 
			}
		}
	}
	ENABLE;
	return;
}


void sys_init()
{
	TMOD &= 0xf0;
	TMOD |= 0x01;  /* 置T0为16位定时计数方式 */
	IE = 0xc0;
	IE |= 0x02;    /* 允许T0定时中断 */
	IE |= 0x10;    /* 允许串行通讯中断 */
	TH0 = 0xff;
	TL0 = 0x00;
	Tic_5ms = 0;
	TF0 = 0;    /* 清中断标志 */
	TR0 = 1;    /* 启动定时计数 */
	{         /* 通讯初始化 */
		TMOD &= 0x0f;
		TMOD |= 0x20; /* 8位定时，自动重装载 */
		TH1 = 0xf7;    /* 9600bps，TH1=*/
		TL1 = TH1;
		TF1 = 0;
		TR1 = 1;
		SCON = 0x50;  /* 0x50，模式1 */
		PCON = 0xc0;  /* smod=1 */
	}
	E_485R;
}

//disp_dta_switcher 1...12
void update_disp_dta(BYTE disp_dta_switcher)
{
	BYTE temp1, temp2;
	Comm_watchdog++;  /* E_NUM*15mS，通讯故障计数 */
	if (Comm_watchdog >= E_NUM) /* 通讯故障显示	SOS  */
	{
		disp_da[0] = DISP_DTA[5];
		disp_da[1] = DISP_DTA[0];
		disp_da[2] = DISP_DTA[5];
	}
	else
	{
		if (Rev_buff[10] & 0x03)    /* .snow */
		{  /* 正常滚动显示9-11B通讯信息 */
			temp2 = disp_dta_switcher; //1 - 12
			temp2--;//0-11
			temp2 /= 4;
			temp2 += 9;//9,A,B
			disp_da[0] = DISP_DTA[temp2];
			temp1 = Rev_buff[temp2];
			temp1 >>= 4;
			disp_da[1] = DISP_DTA[temp1];
			temp1 = Rev_buff[temp2];
			temp1 &= 0x0f;
			disp_da[2] = DISP_DTA[temp1];
		}
		else
		{   /* 停机滚动显示10-12B通讯信息 */
			temp2 = disp_dta_switcher; //1-12
			temp2--; //0-11
			temp2 /= 4; //0-2
			temp2 += 10; //A,B,C
			disp_da[0] = DISP_DTA[temp2];
			temp1 = Rev_buff[temp2];
			temp1 >>= 4;
			disp_da[1] = DISP_DTA[temp1];
			temp1 = Rev_buff[temp2];
			temp1 &= 0x0f;
			disp_da[2] = DISP_DTA[temp1];
		}
	}
}

void main()
{
	
	BYTE disp_dta_switcher;
	sys_init();
	status_o = 0x00;
	num_m = 0x00;   /* 采样滤波用 */
	for (temp0 = 0; temp0 < 4; temp0++)     disp_da[temp0] = 0;
	for (temp0 = 0; temp0 < 22; temp0++)  Rev_buff[temp0] = 0;
	for (temp0 = 0; temp0 < 5; temp0++)  { ta[temp0] = 0; Send_buff[temp0] = 0; }
	Send_buff[0] = 0xa5;  /* 头 */
	Send_buff[3] = 0xfd;  /* 尾 */
	Rev_count = 0;
	Send_count = 0;
	Comm_watchdog = E_NUM;  /* 初始为通讯未通状态 */

	P0 |= 0x0f;/* 曾写成P0=0xff，使通讯进入发送状态，错！！*/
	P1 = 0xff;
	P2 = 0xff;
	disp_dta_switcher = 0x01;
	DISP_DTA[0] = 0x3f;/* 0-F */
	DISP_DTA[1] = 0x30;
	DISP_DTA[2] = 0x5b;
	DISP_DTA[3] = 0x4f;
	DISP_DTA[4] = 0x66;
	DISP_DTA[5] = 0x6d;
	DISP_DTA[6] = 0x7d;
	DISP_DTA[7] = 0x07;
	DISP_DTA[8] = 0x7f;
	DISP_DTA[9] = 0x6f;
	DISP_DTA[10] = 0xf7;
	DISP_DTA[11] = 0xfc;
	DISP_DTA[12] = 0xb9;
	DISP_DTA[13] = 0xde;
	DISP_DTA[14] = 0xf9;
	DISP_DTA[15] = 0xf1;
	while (1)
	{
		if ((Rev_count >= 4) && (Sending_Tic == 0)) 
		{
			Sending_Tic++; 
			Comm_watchdog = 0;
		}
		/* 主机发完14字节，定时即开始，因总是出现接收错误导致压缩机停机 */
		if (case_switcher != Tic_5ms)
		{
			case_switcher = Tic_5ms;

			if (Sending_Tic != 0) 
				Sending_Tic++;

			if (Sending_Tic == 12) /* 12×5mS */
			{
				Rev_count = 0;
				E_485T;   /* 开始发送 */
				Send_count = 0;
				SBUF = Send_buff[Send_count];
			}
			if (Sending_Tic >= 14) /* 14×5mS */
			{
				Sending_Tic = 0;  /* 退出定时状态 */
				Rev_count = 0;
				E_485T;   /* 开始发送 */
				Send_count = 0;
				SBUF = Send_buff[Send_count];
			}

			switch (case_switcher % 3)	// visit each case per 5ms
			{
			case 0:
			{
				
				disp_switcher++;
				if (disp_switcher == 30)  /* 每30×15=450mS改变一次显示内容 */
				{
					disp_switcher = 0;

					disp_dta_switcher++;  /* 通讯数据循环显示 */
					if (disp_dta_switcher >= 12) 
						disp_dta_switcher = 0x01;
					update_disp_dta(disp_dta_switcher);
				}

				P2 = disp_da[0]; P0 &= 0xf0; P0 |= 0x08;/* 驱动显示L1 */

				break;
			}
			case 1:
			{  /* 状态采样，如果通讯数据不可靠，乍办？用上次命令 */
				if (Send_buff[1] & 0x03) /* 工作时只关注工作端命令 */
				{
					temp1 = Send_buff[1];
					temp1 &= 0x0c;
					Send_buff[4] = temp1; /* 端口命令不变 */
					if (temp1 == 0x08)  P1 |= 0x80;  /* 置两选一控制口 */
					else           P1 &= 0x7f;
					temp1 = P1;    /* 读数据 */
					temp1 = P1;
					temp1 &= 0x0f; /* 取当前状态 */

					if (temp1 == status_o)  /* 连续四次以上相同，方为有效 */
					{
						num_m++;
						if (num_m > 4) num_m = 4;
					}
					else
					{
						num_m = 1;
						status_o = temp1;
					}

					if (num_m == 4)  /* 信号有效，则执行 */
					{
						temp1 = status_o;
						if (temp1 & 0x01); /* 优先级：停机↓保护↓制冷↓通风 */
						else  { Send_buff[4] &= 0xfc; Send_buff[4] |= 0x01; }
						if (temp1 & 0x02); /* 低有效 */
						else  { Send_buff[4] &= 0xfc; Send_buff[4] |= 0x02; }
						if ((temp1 & 0x0c) == 0x00); /* 过温和过压时降为通风运行 */
						else
							if (Send_buff[4] & 0x03)
							{
							Send_buff[4] &= 0xfc; Send_buff[4] |= 0x01;
							}
					}
					else   Send_buff[4] = Send_buff[1]; /* 无有效信号，则命令顺延 */

				}
				else  /* 停机时先关注启动信号 */
				{
					temp1 = P1;    /* 读数据 */
					temp1 &= 0x30; /* 取启动信号 */
					temp1 >>= 2;
					temp1 ^= 0x0c; /* 因低有效,需取反 */
					if (temp1 == 0x0c)   temp1 = 0x00; /* 竞争无效 */
					if (temp1 != 0x00) { temp1 |= 0x01; Send_buff[4] = temp1; }
					else   Send_buff[4] = Send_buff[1]; /* 无启动信号，则命令顺延 */
					/* 有启动信号，必有通风信号，下一采样循环再取实际工况 */
				}
				DISABLE; /* 防止互反的两数据不一致,~取反 */
				if ((Send_count != 1) && (Send_count != 2))
				{
					Send_buff[1] = Send_buff[4];
					Send_buff[2] = ~Send_buff[4];
				}
				ENABLE;
				P2 = disp_da[1]; P0 &= 0xf0; P0 |= 0x04; /* 驱动显示L2 */
				break;
			}
			case 2:
			{
				temp1 = 0x00;        /* 故障灯控制 */
				if (Comm_watchdog >= E_NUM)
					temp1 += 0x04;     /* 通讯故障指示 */
				if (((Send_buff[1] & 0x03) != 0) && ((Rev_buff[10] & 0x03) == 0))
					temp1++;     /* 有令不行-电源故障指示 */
				if (temp1 == 0) ta[4] = 0;
				else ta[4] += temp1;
				if (ta[4] >= L_NUM) { P1 &= 0xbf; ta[4] = L_NUM; }
				else  P1 |= 0x40;

				P2 = disp_da[2]; P0 &= 0xf0; P0 |= 0x02; /* 驱动显示L3 */
				break;
			}
			default:break;
			}
		}

	}
	/*
			开关状态采样；
			开关状态分析；
			工况确定和发送数据生成；
			接收数据判误；
			接收数据处理；
			发光管和数码管控制；
			继电器控制；
			温度采集；
			数据存储；
			*/
	return;
}

