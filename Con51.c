#include < reg51.h >
/****************************************************************
˵��:  �յ���Դ���ݰ��ã�DA50202.C,������,2002/11/11
Ϊ�˽��ѹ����������ĸ���������Ķ���2002/12/27��
1���ſ���ձ�׼��
2����һ�η���Ϊ���η��ͣ�
3����ǩΪCSMA50202E��CSMA50202F��
Ϊ�˽�����й�����״̬�������⣨��ѹ���������������Ķ���03/06/21
1�����ӱ���status_o��num_m��
2�����������Ĵ���ͬ��Ϊ��Ч�ź�(4*15=60mS)��
3����ǩΪCSMA50202E*��
����:16MHz,״̬����:1/8��S,HSO����:1��S.
P0��:
P07:X24XXʱ����;
P06:X24XX������;
P05:ͨѶ�����ߣ��߷�����;
P04:DS1820������;
P03:�����L1���ƣ�����;
P02:�����L2���ƣ�����;
P01:�����L3���ƣ�����;
P00:����ܿ���;
P1��:
P17:�˿�ѡ���ߵ�һ�˸߶��ˣ�
P16:�����ź���    ������Ч��
P15:���������ź��ߣ�����Ч��
P14:һ�������ź��ߣ�����Ч��
P13:ѹ�������ź��ߣ�  ����Ч;
P12:�¿��ź���    ��  ����Ч;
P11:�����ź���    ������Ч;
P10:ͨ���ź���    ������Ч;
P2�ڣ�
P2.0-7:����ܺͷ�������˿�����
��ʾ����,���ձ�(����/��ʾ����)
0/3FH,1/30H,2/5BH,3/4FH,4/66H,5/6DH,6/7DH,7/07H,
8/7FH,9/6FH.A./F7H,b./FCH,C./B9H,d./DEH,E./F9H,F./F1H,
H./F6H,L./B8H,O./DCH,P./F3H,U./BEH.
�������ݶ��壺
7-4/Ƶ��������32/00��11��Ч��01һ�ˡ�10���ˣ�10/����
��ΪPO��δ������������´�
��λ��������������ʱ�����ܹ���(�����ļ���io51.h����reg51.h���)��
���ж϶��巽ʽ��ԭ��ʹ�õķ�ʽ������ʹ�жϲ��ܽ���
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

/* 485���ƿ�ΪP0.5 */
#define E_485R { P0&=0xdf;REN=1; }
#define E_485T { P0|=0x20;REN=0; }

/* U/U1,F/Ƶ��,S/ʵ�ʹ���,E/�ڲ����ϣ�C/ͨѶ�͵��״̬ */
BYTE data disp_da[4], ta[5], temp0, temp1, temp2;
BYTE data Tic_5ms, disp_dta_switcher;
BYTE data status_o, num_m; /* �����˲���,ͬ���ۼƣ�����ˢ�� */

BYTE data Send_count, Rev_count;
BYTE data Send_buff[5]; /* [4]Ϊ����״̬ */
BYTE data Rev_buff[22];
BYTE data DISP_DTA[16];
BYTE Comm_watchdog;
BYTE case_switcher, disp_switcher;
BYTE Sending_Tic;
/* #define R_TF0     TCON &=0xdf */

void delay5() interrupt 1  /* 5mS��ʱ�ж� */
{
	DISABLE;
	TH0 = 0xe5; /* ÿ����12T��5mS��ʱ��16MHz */
	TL0 = 0xf5;
	TF0 = 0;
	TR0 = 1;
	Tic_5ms++;
	Tic_5ms %= 200;
	ENABLE;
	return;
}



void rs_int() interrupt 4  /* ͨѶ�ж� */
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
		if (Rev_count != 22) /* ��ֹ���������ɳ������ */
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
	TMOD |= 0x01;  /* ��T0Ϊ16λ��ʱ������ʽ */
	IE = 0xc0;
	IE |= 0x02;    /* ����T0��ʱ�ж� */
	IE |= 0x10;    /* ������ͨѶ�ж� */
	TH0 = 0xff;
	TL0 = 0x00;
	Tic_5ms = 0;
	TF0 = 0;    /* ���жϱ�־ */
	TR0 = 1;    /* ������ʱ���� */
	{         /* ͨѶ��ʼ�� */
		TMOD &= 0x0f;
		TMOD |= 0x20; /* 8λ��ʱ���Զ���װ�� */
		TH1 = 0xf7;    /* 9600bps��TH1=*/
		TL1 = TH1;
		TF1 = 0;
		TR1 = 1;
		SCON = 0x50;  /* 0x50��ģʽ1 */
		PCON = 0xc0;  /* smod=1 */
	}
	E_485R;
}

//disp_dta_switcher 1...12
void update_disp_dta(BYTE disp_dta_switcher)
{
	BYTE temp1, temp2;
	Comm_watchdog++;  /* E_NUM*15mS��ͨѶ���ϼ��� */
	if (Comm_watchdog >= E_NUM) /* ͨѶ������ʾ	SOS  */
	{
		disp_da[0] = DISP_DTA[5];
		disp_da[1] = DISP_DTA[0];
		disp_da[2] = DISP_DTA[5];
	}
	else
	{
		if (Rev_buff[10] & 0x03)    /* .snow */
		{  /* ����������ʾ9-11BͨѶ��Ϣ */
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
		{   /* ͣ��������ʾ10-12BͨѶ��Ϣ */
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
	num_m = 0x00;   /* �����˲��� */
	for (temp0 = 0; temp0 < 4; temp0++)     disp_da[temp0] = 0;
	for (temp0 = 0; temp0 < 22; temp0++)  Rev_buff[temp0] = 0;
	for (temp0 = 0; temp0 < 5; temp0++)  { ta[temp0] = 0; Send_buff[temp0] = 0; }
	Send_buff[0] = 0xa5;  /* ͷ */
	Send_buff[3] = 0xfd;  /* β */
	Rev_count = 0;
	Send_count = 0;
	Comm_watchdog = E_NUM;  /* ��ʼΪͨѶδͨ״̬ */

	P0 |= 0x0f;/* ��д��P0=0xff��ʹͨѶ���뷢��״̬������*/
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
		/* ��������14�ֽڣ���ʱ����ʼ�������ǳ��ֽ��մ�����ѹ����ͣ�� */
		if (case_switcher != Tic_5ms)
		{
			case_switcher = Tic_5ms;

			if (Sending_Tic != 0) 
				Sending_Tic++;

			if (Sending_Tic == 12) /* 12��5mS */
			{
				Rev_count = 0;
				E_485T;   /* ��ʼ���� */
				Send_count = 0;
				SBUF = Send_buff[Send_count];
			}
			if (Sending_Tic >= 14) /* 14��5mS */
			{
				Sending_Tic = 0;  /* �˳���ʱ״̬ */
				Rev_count = 0;
				E_485T;   /* ��ʼ���� */
				Send_count = 0;
				SBUF = Send_buff[Send_count];
			}

			switch (case_switcher % 3)	// visit each case per 5ms
			{
			case 0:
			{
				
				disp_switcher++;
				if (disp_switcher == 30)  /* ÿ30��15=450mS�ı�һ����ʾ���� */
				{
					disp_switcher = 0;

					disp_dta_switcher++;  /* ͨѶ����ѭ����ʾ */
					if (disp_dta_switcher >= 12) 
						disp_dta_switcher = 0x01;
					update_disp_dta(disp_dta_switcher);
				}

				P2 = disp_da[0]; P0 &= 0xf0; P0 |= 0x08;/* ������ʾL1 */

				break;
			}
			case 1:
			{  /* ״̬���������ͨѶ���ݲ��ɿ���է�죿���ϴ����� */
				if (Send_buff[1] & 0x03) /* ����ʱֻ��ע���������� */
				{
					temp1 = Send_buff[1];
					temp1 &= 0x0c;
					Send_buff[4] = temp1; /* �˿������ */
					if (temp1 == 0x08)  P1 |= 0x80;  /* ����ѡһ���ƿ� */
					else           P1 &= 0x7f;
					temp1 = P1;    /* ������ */
					temp1 = P1;
					temp1 &= 0x0f; /* ȡ��ǰ״̬ */

					if (temp1 == status_o)  /* �����Ĵ�������ͬ����Ϊ��Ч */
					{
						num_m++;
						if (num_m > 4) num_m = 4;
					}
					else
					{
						num_m = 1;
						status_o = temp1;
					}

					if (num_m == 4)  /* �ź���Ч����ִ�� */
					{
						temp1 = status_o;
						if (temp1 & 0x01); /* ���ȼ���ͣ���������������ͨ�� */
						else  { Send_buff[4] &= 0xfc; Send_buff[4] |= 0x01; }
						if (temp1 & 0x02); /* ����Ч */
						else  { Send_buff[4] &= 0xfc; Send_buff[4] |= 0x02; }
						if ((temp1 & 0x0c) == 0x00); /* ���º͹�ѹʱ��Ϊͨ������ */
						else
							if (Send_buff[4] & 0x03)
							{
							Send_buff[4] &= 0xfc; Send_buff[4] |= 0x01;
							}
					}
					else   Send_buff[4] = Send_buff[1]; /* ����Ч�źţ�������˳�� */

				}
				else  /* ͣ��ʱ�ȹ�ע�����ź� */
				{
					temp1 = P1;    /* ������ */
					temp1 &= 0x30; /* ȡ�����ź� */
					temp1 >>= 2;
					temp1 ^= 0x0c; /* �����Ч,��ȡ�� */
					if (temp1 == 0x0c)   temp1 = 0x00; /* ������Ч */
					if (temp1 != 0x00) { temp1 |= 0x01; Send_buff[4] = temp1; }
					else   Send_buff[4] = Send_buff[1]; /* �������źţ�������˳�� */
					/* �������źţ�����ͨ���źţ���һ����ѭ����ȡʵ�ʹ��� */
				}
				DISABLE; /* ��ֹ�����������ݲ�һ��,~ȡ�� */
				if ((Send_count != 1) && (Send_count != 2))
				{
					Send_buff[1] = Send_buff[4];
					Send_buff[2] = ~Send_buff[4];
				}
				ENABLE;
				P2 = disp_da[1]; P0 &= 0xf0; P0 |= 0x04; /* ������ʾL2 */
				break;
			}
			case 2:
			{
				temp1 = 0x00;        /* ���ϵƿ��� */
				if (Comm_watchdog >= E_NUM)
					temp1 += 0x04;     /* ͨѶ����ָʾ */
				if (((Send_buff[1] & 0x03) != 0) && ((Rev_buff[10] & 0x03) == 0))
					temp1++;     /* �����-��Դ����ָʾ */
				if (temp1 == 0) ta[4] = 0;
				else ta[4] += temp1;
				if (ta[4] >= L_NUM) { P1 &= 0xbf; ta[4] = L_NUM; }
				else  P1 |= 0x40;

				P2 = disp_da[2]; P0 &= 0xf0; P0 |= 0x02; /* ������ʾL3 */
				break;
			}
			default:break;
			}
		}

	}
	/*
			����״̬������
			����״̬������
			����ȷ���ͷ����������ɣ�
			������������
			�������ݴ���
			����ܺ�����ܿ��ƣ�
			�̵������ƣ�
			�¶Ȳɼ���
			���ݴ洢��
			*/
	return;
}

