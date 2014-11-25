/****************************************************************
   文件名:data.c
   系统说明,参量定义
   适用于机车空调节器电源  WJDA502_002，
   天津滨海线5KVA应急电源用
   *****************************************************************/
/****************************************************************
说明:

晶振:16MHz,状态周期:1/8μS,HSO周期:1μS.
27C256地址区:0000H-7FFFH(实际为2000H-7FFFH,共24K);
6264地址区:8000H-9FFFH(共8K);
8255地址区:0A000H-0BFFFH(实际为0A000H-0A003H,共4);
8255口分配:
PA口:输入口,故障采样
PA7:U1L,进线电压欠压;
PA6:U2H,中间电压过压;
PA5:未用;
PA4:I1H,进线电流过流;
PA3:I2H,中间电流过流;
PA2:F1V,前级逆变V路故障;
PA1:F1U,前级逆变U路故障;
PA0:F2A,后级逆变故障;
PB口:输出口,显示驱动,对照表(B口数据/显示数字)
0/3FH,1/30H,2/5BH,3/4FH,4/66H,5/6DH,6/7DH,7/07H,8/7FH,9/6FH.A./F7H,
b_11./FCH,C.12./B9H,d.13./DEH,E.14./F9H,F.15./F1H,H.16./F6H,L.17./B8H,O.18./DCH,P.19./F3H,U.20./BEH.
PC口:
PC7-PC1:未用;
PC0:N1ON,前级逆变控制,高允许,低禁止.

HSO口:
HSO.0:对应通风机控制信号,高接通继电器;
HSO.1:对应冷凝机控制信号,高接通继电器;
HSO.2:对应压缩机控制信号,高接通继电器;
HSO.3:对应加热器控制信号,高接通继电器;
HSO.4:对应A端控制信号,高接通继电器;
HSO.5:对应B端控制信号,高接通继电器;

P0口:
P0.7:U2C,中间直流电压;        1V-200V
P0.6:I2C,中间直流电流;        1V-4A
P0.5:U1B,输入直流电压;        1V-40V
P0.4:I1C,输入直流电流;        1V-20A
P0.3:IM4,电热器电流;          1V-2.5A
P0.2:IM3,压缩机电流;          1V-2.5A
P0.1:IM2,冷凝机电流;          1V-0.5A
P0.0:IM1,通风机电流;          1V-0.25A

P1口:(P1.7-P1.0,高则高,一直保持一致性,不变，)
UT0..VT0..WT0..NT0..UB0..VB0..WB0..NB0

P2口:
P2.3:工作方式设置开关1；
P2.4:工作方式设置开关2；
P2.6:RS485口线(高发送低接收),;
P2.7:DS1820口线;
*******************************************************************/
#ifndef DATA_HH
#define DATA_HH

//#include _SFR_H_
#include <KC_SFRS.H>
#include <KC_FUNCS.H>


#define OUTPUT_DEBUG 1

#define  uchar    unsigned char
#define  BYTE     unsigned char
#define  UI       unsigned int
#define  UL       unsigned long 
#define  NULL     0
#define  BUSY     1


#define ADDR_8255  0xA000 
/* +0/A，故障口；+1/B，显示口；+2/C，前级控制口；+3/8255控制口  */
#define ADDR_DIS  0x6000  /* 27256，显示信息      ,               */
#define ADDR_MOD  0x6020  /* 27256，开关模式首地址,               */
#define ADDR_WID  0x6100  /* 27256，电压频率对照表首地址,         */
#define ADDR_SIN  0x7000  /* 27256，正弦表首地址,                 */
//below stores in 6264
#define ADDR_MEM1 0x8000  /* 6264，系统数据存贮区首地址，256,     It's also sent to 51 MCU*/
#define ADDR_MEM2 0x8100  /* 6264，接收区首地址，        256,     */
#define ADDR_CA0  0x9000  /* 6264，计算区0首地址，2K,             */  
#define ADDR_CA1  0x9800  /* 6264，计算区1首地址，2K,             */    


#define FREQ_MAX_LIMIT    500     /* 目标频率*10 */         
#define F_STARTC  50      /* 启动频率*10 */  


extern BYTE    rsb0, rsb1, rsb2, rsp_c;
/* 通讯用，0/共用，1/发送计数，2/接收计数，rsp_c/sp_con映射 */
extern BYTE    *ptrb0, *PWM_out_ptr, *Ptr_8255, *send_buf_ptr, *rec_buf_ptr;
/*  0/公用,1/开关模式,2/故障口，3/发送区首指针，4/接收区首指针  */
extern UI      *ptrw0, *ptrw1, *ptrw2,  *ptrw4;
/* 0/公用，1/正弦表头，2/正弦表尾，3/时间计算表，4/时间发送表 */
extern BYTE    io1, io2;         /*  端口影射      */
extern BYTE    PWM_shutdown; /*  逆变控制变量  */
extern BYTE p_erro, s_erro, s_set;
extern BYTE tempb0, tempb1, tempb2;

extern UI   tempw0, tempw1, tempw2, tempw3, tempw4, tempw5, tempw6, tempw7;

typedef struct { UI lo;  UI  hi; } BW;  /* 代码效率高 */
extern BW  bw;
typedef union
{
	UL     ll;
	BW     bw;
}  UNA;
extern   UNA tempbw0, tempbw1;
#define  MIN_PULSE     0x1c   /* 最窄脉冲  28   */
#define  HALF_MIN_PULSE (MIN_PULSE>>1)
#define  AVG_ANSWER    0x0e   /* 最快响应 14     */
#define  TIMER0_MODIFY 0x06   /* 休止时间修正,2*Tdead */
#define  NUM6          0x06   /* 分区数       */
#define  NUM3          0x03   /* 分节数        */
extern BYTE     num2[2];    /* 分段数，0-当前，1-预备 */
extern BYTE     s_ii1, s_ii2, s_ii3;
extern UI       Sys_tic_10ms;
extern BYTE     s_control, s_ok, Cur_addr_ca;
/* 7/首次计算，6/数据有效，5/当前发送区，4/未用，*/
/* 3/升压允许，2/升频    ，1/逆变允许  ，0/逆变过程，*/
/* 为提高编译效率，将位6，5单列为s_ok，Cur_addr_ca */

typedef struct
{
	BYTE   head1;/* 0x55 */
	BYTE   u1;   /* 前级电压 */
	BYTE   u2_l; /* 中间电压低位 */
	BYTE   u2_h; /* 中间电压高位 */
	BYTE   i1;   /* 前级电流 */
	BYTE   i2;   /* 中间电流 */
	BYTE   tb;  /* 机箱温度 */
	BYTE   target_freq; /* 目标频率 */
	BYTE   target_mode; /* 目标工况,32/端口,10/工况:00-停机,01-通风,10-制冷,11-制热 */
	BYTE   cur_freq; /* 实际频率,cur_freqc/10, */
	BYTE   cur_mode; /* 实际工况：7/电热，6/压缩，5/冷凝，4/通风，32/端口,10/工况:00-停机,01-通风,10-制冷,11-制热 */ 
	BYTE   ec1; /* 7/命令无效，6/C通讯错 ，5/B通讯错 ，4/A通讯错  */
	/* 3/电热过流，2/压缩过流，1/冷凝过流，0/通风过流 */
	BYTE   ec2; /* 内部故障代码 */
	BYTE   end1; /* 0xfd */

	BYTE   target_modem;/* 机组允许工况，同上，高4位不用 */

	UI     target_freqc; /* target_freq×10,目标频率(控制用) */
	UI     cur_freqc; /* 实际频率(控制用) */
	UI     pre_freqc; /* 上次频率(控制用) */

	BYTE   addl;  /* 加载前降频至<6Hz，0x02/降频2，0x00/不降   */
	UI     t0a;   /* 总通风时间，停机时清零，>50S则=50S */

	UI     t0d;   /* 通风机保护动作计时，单位50mS，20min恢复,下同 */
	UI     t1d;   /* 冷凝机保护动作计时， */
	UI     t2d;   /* 压缩机保护动作计时， */
	UI     t3d;   /* 电加热保护动作计时， */

	UI     im0;   /* 通风机电流×20,每50mS采样一次，下同 */
	UI     im1;   /* 冷凝机电流×20 */
	UI     im2;   /* 压缩机电流×20 */
	UI     im3;   /* 电加热电流×20 */

	UI     m0a;   /* 通风机电流积分 */
	UI     m1a;   /* 冷凝机电流积分 */
	UI     m2a;   /* 压缩机电流积分 */
	UI     m3a;   /* 电加热电流积分 */

	UI     bt10;  /* 10ms时钟，主时基：分时采样、电机保护 */
	UI     bt20;  /* 20ms时钟，变频用，分时通讯           */
	UI     bt1m;  /* 1min时钟,45分定时用   */
	UI     dt45m; /* 延时45min */

	BYTE   dis0;  /* 显示用 */
	BYTE   dis1;
	BYTE   dis2;
	BYTE   dis3;

}	MESSAGE1;
extern MESSAGE1 *msg1;

#define I013   9   /* I0n×20×1.3，0.36A */
#define I113  22   /* I1n×20×1.3，0.86A */
#define I213  99   /* I2n×20×1.3，3.80A */
#define I313  99   /* I3n×20×1.3，3.80A */

#define I017    300  /* 2×(I0n×20×1.7)×(I0n×20×1.7)， */
#define I117   1710  /* 2×(I1n×20×1.7)×(I1n×20×1.7)， */  
#define I217  33385  /* 2×(I2n×20×1.7)×(I2n×20×1.7)， */  
#define I317  33385  /* 2×(I3n×20×1.7)×(I3n×20×1.7)， */  

typedef struct
{
	BYTE   ext_setting_mode;  /* 外部设定工况 */
	BYTE   rs_ea; /* A口连续通讯错误次数×0.1S */
	BYTE   rs_eb; /* B口连续通讯错误次数×0.1S */
	BYTE   rs_ec; /* C口连续通讯错误次数×0.1S */
	BYTE   rs_er; /* 连续无有效命令 次数×0.1S，10S停机 */

	BYTE   ext_setting_modeA; /* A端目标工况， 7654/频率偏移，F=50-X， */
	BYTE   ext_setting_modeB; /* B端目标工况， 7654/频率偏移，F=50-X， */
	BYTE   ext_setting_modeC; /* C端目标工况， 7654/频率偏移，F=50-X， */

}	MESSAGE2;
extern MESSAGE2 *msg2;

#define E_watchdog  /* { watchdog=0x1e;watchdog=0xe1;} */;  /*  设置看门狗 */
#define E_485r      { io2&=0xbf;ioport2=io2;rsp_c|=0x08;sp_con=rsp_c;}         
#define E_485t      { io2|=0x40;ioport2=io2;rsp_c&=0xf7;sp_con=rsp_c;}
/* 避免读-改-写操作，特别是对多功能口，曾因直接对sp_con进行操作而引起通讯错*/

#endif 
