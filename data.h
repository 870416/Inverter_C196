/****************************************************************
   �ļ���:data.c
   ϵͳ˵��,��������
   �����ڻ����յ�������Դ  WJDA502_002��
   ��������5KVAӦ����Դ��
   *****************************************************************/
/****************************************************************
˵��:

����:16MHz,״̬����:1/8��S,HSO����:1��S.
27C256��ַ��:0000H-7FFFH(ʵ��Ϊ2000H-7FFFH,��24K);
6264��ַ��:8000H-9FFFH(��8K);
8255��ַ��:0A000H-0BFFFH(ʵ��Ϊ0A000H-0A003H,��4);
8255�ڷ���:
PA��:�����,���ϲ���
PA7:U1L,���ߵ�ѹǷѹ;
PA6:U2H,�м��ѹ��ѹ;
PA5:δ��;
PA4:I1H,���ߵ�������;
PA3:I2H,�м��������;
PA2:F1V,ǰ�����V·����;
PA1:F1U,ǰ�����U·����;
PA0:F2A,��������;
PB��:�����,��ʾ����,���ձ�(B������/��ʾ����)
0/3FH,1/30H,2/5BH,3/4FH,4/66H,5/6DH,6/7DH,7/07H,8/7FH,9/6FH.A./F7H,
b_11./FCH,C.12./B9H,d.13./DEH,E.14./F9H,F.15./F1H,H.16./F6H,L.17./B8H,O.18./DCH,P.19./F3H,U.20./BEH.
PC��:
PC7-PC1:δ��;
PC0:N1ON,ǰ��������,������,�ͽ�ֹ.

HSO��:
HSO.0:��Ӧͨ��������ź�,�߽�ͨ�̵���;
HSO.1:��Ӧ�����������ź�,�߽�ͨ�̵���;
HSO.2:��Ӧѹ���������ź�,�߽�ͨ�̵���;
HSO.3:��Ӧ�����������ź�,�߽�ͨ�̵���;
HSO.4:��ӦA�˿����ź�,�߽�ͨ�̵���;
HSO.5:��ӦB�˿����ź�,�߽�ͨ�̵���;

P0��:
P0.7:U2C,�м�ֱ����ѹ;        1V-200V
P0.6:I2C,�м�ֱ������;        1V-4A
P0.5:U1B,����ֱ����ѹ;        1V-40V
P0.4:I1C,����ֱ������;        1V-20A
P0.3:IM4,����������;          1V-2.5A
P0.2:IM3,ѹ��������;          1V-2.5A
P0.1:IM2,����������;          1V-0.5A
P0.0:IM1,ͨ�������;          1V-0.25A

P1��:(P1.7-P1.0,�����,һֱ����һ����,���䣬)
UT0..VT0..WT0..NT0..UB0..VB0..WB0..NB0

P2��:
P2.3:������ʽ���ÿ���1��
P2.4:������ʽ���ÿ���2��
P2.6:RS485����(�߷��͵ͽ���),;
P2.7:DS1820����;
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
/* +0/A�����Ͽڣ�+1/B����ʾ�ڣ�+2/C��ǰ�����ƿڣ�+3/8255���ƿ�  */
#define ADDR_DIS  0x6000  /* 27256����ʾ��Ϣ      ,               */
#define ADDR_MOD  0x6020  /* 27256������ģʽ�׵�ַ,               */
#define ADDR_WID  0x6100  /* 27256����ѹƵ�ʶ��ձ��׵�ַ,         */
#define ADDR_SIN  0x7000  /* 27256�����ұ��׵�ַ,                 */
//below stores in 6264
#define ADDR_MEM1 0x8000  /* 6264��ϵͳ���ݴ������׵�ַ��256,     It's also sent to 51 MCU*/
#define ADDR_MEM2 0x8100  /* 6264���������׵�ַ��        256,     */
#define ADDR_CA0  0x9000  /* 6264��������0�׵�ַ��2K,             */  
#define ADDR_CA1  0x9800  /* 6264��������1�׵�ַ��2K,             */    


#define FREQ_MAX_LIMIT    500     /* Ŀ��Ƶ��*10 */         
#define F_STARTC  50      /* ����Ƶ��*10 */  


extern BYTE    rsb0, rsb1, rsb2, rsp_c;
/* ͨѶ�ã�0/���ã�1/���ͼ�����2/���ռ�����rsp_c/sp_conӳ�� */
extern BYTE    *ptrb0, *PWM_out_ptr, *Ptr_8255, *send_buf_ptr, *rec_buf_ptr;
/*  0/����,1/����ģʽ,2/���Ͽڣ�3/��������ָ�룬4/��������ָ��  */
extern UI      *ptrw0, *ptrw1, *ptrw2,  *ptrw4;
/* 0/���ã�1/���ұ�ͷ��2/���ұ�β��3/ʱ������4/ʱ�䷢�ͱ� */
extern BYTE    io1, io2;         /*  �˿�Ӱ��      */
extern BYTE    PWM_shutdown; /*  �����Ʊ���  */
extern BYTE p_erro, s_erro, s_set;
extern BYTE tempb0, tempb1, tempb2;

extern UI   tempw0, tempw1, tempw2, tempw3, tempw4, tempw5, tempw6, tempw7;

typedef struct { UI lo;  UI  hi; } BW;  /* ����Ч�ʸ� */
extern BW  bw;
typedef union
{
	UL     ll;
	BW     bw;
}  UNA;
extern   UNA tempbw0, tempbw1;
#define  MIN_PULSE     0x1c   /* ��խ����  28   */
#define  HALF_MIN_PULSE (MIN_PULSE>>1)
#define  AVG_ANSWER    0x0e   /* �����Ӧ 14     */
#define  TIMER0_MODIFY 0x06   /* ��ֹʱ������,2*Tdead */
#define  NUM6          0x06   /* ������       */
#define  NUM3          0x03   /* �ֽ���        */
extern BYTE     num2[2];    /* �ֶ�����0-��ǰ��1-Ԥ�� */
extern BYTE     s_ii1, s_ii2, s_ii3;
extern UI       Sys_tic_10ms;
extern BYTE     s_control, s_ok, Cur_addr_ca;
/* 7/�״μ��㣬6/������Ч��5/��ǰ��������4/δ�ã�*/
/* 3/��ѹ����2/��Ƶ    ��1/�������  ��0/�����̣�*/
/* Ϊ��߱���Ч�ʣ���λ6��5����Ϊs_ok��Cur_addr_ca */

typedef struct
{
	BYTE   head1;/* 0x55 */
	BYTE   u1;   /* ǰ����ѹ */
	BYTE   u2_l; /* �м��ѹ��λ */
	BYTE   u2_h; /* �м��ѹ��λ */
	BYTE   i1;   /* ǰ������ */
	BYTE   i2;   /* �м���� */
	BYTE   tb;  /* �����¶� */
	BYTE   target_freq; /* Ŀ��Ƶ�� */
	BYTE   target_mode; /* Ŀ�깤��,32/�˿�,10/����:00-ͣ��,01-ͨ��,10-����,11-���� */
	BYTE   cur_freq; /* ʵ��Ƶ��,cur_freqc/10, */
	BYTE   cur_mode; /* ʵ�ʹ�����7/���ȣ�6/ѹ����5/������4/ͨ�磬32/�˿�,10/����:00-ͣ��,01-ͨ��,10-����,11-���� */ 
	BYTE   ec1; /* 7/������Ч��6/CͨѶ�� ��5/BͨѶ�� ��4/AͨѶ��  */
	/* 3/���ȹ�����2/ѹ��������1/����������0/ͨ����� */
	BYTE   ec2; /* �ڲ����ϴ��� */
	BYTE   end1; /* 0xfd */

	BYTE   target_modem;/* ������������ͬ�ϣ���4λ���� */

	UI     target_freqc; /* target_freq��10,Ŀ��Ƶ��(������) */
	UI     cur_freqc; /* ʵ��Ƶ��(������) */
	UI     pre_freqc; /* �ϴ�Ƶ��(������) */

	BYTE   addl;  /* ����ǰ��Ƶ��<6Hz��0x02/��Ƶ2��0x00/����   */
	UI     t0a;   /* ��ͨ��ʱ�䣬ͣ��ʱ���㣬>50S��=50S */

	UI     t0d;   /* ͨ�������������ʱ����λ50mS��20min�ָ�,��ͬ */
	UI     t1d;   /* ����������������ʱ�� */
	UI     t2d;   /* ѹ��������������ʱ�� */
	UI     t3d;   /* ����ȱ���������ʱ�� */

	UI     im0;   /* ͨ���������20,ÿ50mS����һ�Σ���ͬ */
	UI     im1;   /* ������������20 */
	UI     im2;   /* ѹ����������20 */
	UI     im3;   /* ����ȵ�����20 */

	UI     m0a;   /* ͨ����������� */
	UI     m1a;   /* �������������� */
	UI     m2a;   /* ѹ������������ */
	UI     m3a;   /* ����ȵ������� */

	UI     bt10;  /* 10msʱ�ӣ���ʱ������ʱ������������� */
	UI     bt20;  /* 20msʱ�ӣ���Ƶ�ã���ʱͨѶ           */
	UI     bt1m;  /* 1minʱ��,45�ֶ�ʱ��   */
	UI     dt45m; /* ��ʱ45min */

	BYTE   dis0;  /* ��ʾ�� */
	BYTE   dis1;
	BYTE   dis2;
	BYTE   dis3;

}	MESSAGE1;
extern MESSAGE1 *msg1;

#define I013   9   /* I0n��20��1.3��0.36A */
#define I113  22   /* I1n��20��1.3��0.86A */
#define I213  99   /* I2n��20��1.3��3.80A */
#define I313  99   /* I3n��20��1.3��3.80A */

#define I017    300  /* 2��(I0n��20��1.7)��(I0n��20��1.7)�� */
#define I117   1710  /* 2��(I1n��20��1.7)��(I1n��20��1.7)�� */  
#define I217  33385  /* 2��(I2n��20��1.7)��(I2n��20��1.7)�� */  
#define I317  33385  /* 2��(I3n��20��1.7)��(I3n��20��1.7)�� */  

typedef struct
{
	BYTE   ext_setting_mode;  /* �ⲿ�趨���� */
	BYTE   rs_ea; /* A������ͨѶ���������0.1S */
	BYTE   rs_eb; /* B������ͨѶ���������0.1S */
	BYTE   rs_ec; /* C������ͨѶ���������0.1S */
	BYTE   rs_er; /* ��������Ч���� ������0.1S��10Sͣ�� */

	BYTE   ext_setting_modeA; /* A��Ŀ�깤���� 7654/Ƶ��ƫ�ƣ�F=50-X�� */
	BYTE   ext_setting_modeB; /* B��Ŀ�깤���� 7654/Ƶ��ƫ�ƣ�F=50-X�� */
	BYTE   ext_setting_modeC; /* C��Ŀ�깤���� 7654/Ƶ��ƫ�ƣ�F=50-X�� */

}	MESSAGE2;
extern MESSAGE2 *msg2;

#define E_watchdog  /* { watchdog=0x1e;watchdog=0xe1;} */;  /*  ���ÿ��Ź� */
#define E_485r      { io2&=0xbf;ioport2=io2;rsp_c|=0x08;sp_con=rsp_c;}         
#define E_485t      { io2|=0x40;ioport2=io2;rsp_c&=0xf7;sp_con=rsp_c;}
/* �����-��-д�������ر��ǶԶ๦�ܿڣ�����ֱ�Ӷ�sp_con���в���������ͨѶ��*/

#endif 
