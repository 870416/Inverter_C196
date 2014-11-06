#include "data.h"
#include "errs.h"


/* ���жϹ��ϴ�����������������*/
void Exception_handler()
{
	/*  ���Ͽ����ݶ�ȡ         */
	p_erro = *(Ptr_8255 + 0);
	p_erro |= 0x20;     /* PA.5λ����δ��  */
	tempb0 = s_control;
	tempb0 |= 0x0f;
	if ((*msg1).dt45m >= 45)
		tempb0 &= 0xf1; /* ��ʱ��,��ֹ���� */
	tempb1 = p_erro;
	tempb1 &= s_erro;
	(*msg1).ec2 = tempb1;   /* �����ź��⴫ */
	if ((s_erro & 0x07) != 0x07)
		(*msg1).cur_mode &= 0x0c; /* IGBT����ͣ�� */
	if (tempb1 & 0x80)
		;
	else
	{
		tempb0 &= 0xfb;
	} /* .7��   ǰ��Ƿѹʱ������ѹ   */
	/*
	if(tempb1 & 0x40);
	else { tempb0&=0xf7;}
	*/
	/* .6��   �󼶹�ѹ   */
	/* ��CPU�Բ��������ص�������ǰ��Ƶ���ضϣ������ڱ�ѹ���͵�� */
	if (tempb1 & 0x10);
	else { tempb0 &= 0xf3; } /* .4��   ǰ������   */
	if (tempb1 & 0x08);
	else { tempb0 &= 0xfb; } /* .3��   �󼶹���   */
	if (tempb1 & 0x04);
	else { tempb0 &= 0xf1; } /* .2��   IGBT����   */
	if (tempb1 & 0x02);
	else { tempb0 &= 0xf1; } /* .1��   IGBT����   */
	if (tempb1 & 0x01);
	else { tempb0 &= 0xf1; } /* .0��   IGBT����   */
	tempw0 = (*msg1).u2_h;
	tempw0 <<= 8;
	tempw0 += (*msg1).u2_l;
	/*  if((*msg1).u1<= 40) { tempb0&=0xf3;}  */ /* ǰ����ѹ���㣡   */
	if (s_set != 0xfd)  /* ����ʱ���� */
	{
		if ((*msg1).u1 >= 145) { tempb0 &= 0xf3; }   /* ǰ����ѹ���ߣ�   */
		if (tempw0 <= 300)     { tempb0 &= 0xfb; }   /* �м��ѹ���㣡   */
	}
	if (tempw0 >= 580) { tempb0 &= 0xf7; }   /* �м��ѹ���ߣ�   */
	if (tempw0 >= 600) { tempb0 &= 0xf1; }   /* �м��ѹΣ�գ�   */
	if ((*msg1).i1 >= 75) { tempb0 &= 0xf3; }   /* ǰ������Σ�գ�   */
	if ((*msg1).i2 >= 20) { tempb0 &= 0xf1; }   /* �м����Σ�գ�   */
	tempb1 = 0x00;  /* ǰ������ */
	if (tempb0 & 0x08)
		tempb1 = 0x01;
	*(Ptr_8255 + 3) = tempb1;

	/* �����PWM_shutdown���������ʹ���Ʋ�����Ӧ�е�ë�̣��ù��ɱ������������
	�����жϵ�ԭ�򣬿��Ʊ���Ӧһ�Զ��� */
	if (tempb0 & 0x02) 
		tempb1 = 0x00;
	else 
		tempb1 = 0xff;
	if ((*msg1).cur_freqc == F_STARTC) tempb1 = 0xff;
	PWM_shutdown = tempb1;
	s_control = tempb0;
	return;
}




void treatmess()  /* ��Ϣ�����������ơ�������� */
{
	/* ȷ��Ŀ��Ƶ�ʡ�����Ŀ��Ƶ�� */
	tempb1 = (*msg2).ext_setting_mode;
	tempb1 &= 0xf0;
	tempb1 >>= 4;

	(*msg1).target_freq = 50 - tempb1;
	(*msg1).target_freqc = (*msg1).target_freq * 10;
	/* ȷ�������Ŀ�깤�� */
	tempb1 = (*msg2).ext_setting_mode;
	(*msg1).target_mode = tempb1 & 0x0f;

	/* ȷ������������� */
	tempb2 = (*msg1).target_mode;
	tempb2 &= 0x03;

	tempb1 = (*msg1).ec1;
	/* ͨ�����ͣ�� */
	if (tempb1 & 0x01)
		tempb2 = 0x00;

	/* �����������תͨ�� */
	if (tempb1 & 0x06)
		if (tempb2 == 0x02)
			tempb2 = 0x01;

	/* ���ȹ�������תͨ�� */
	if (tempb1 & 0x08)
		if (tempb2 == 0x03)
			tempb2 = 0x01;

	//--------------------------
	//	Fanning counter.
	tempw0 = (*msg1).t0a;         /* ��ֹ��ʱ�������� */
	if (((*msg1).cur_mode & 0x03) == 0x01)
		tempw0++;
	else
		tempw0 = 0;            /* ͨ�繤����ʱ */
	if (tempw0 > 500)
		tempw0 = 500;   /* 50S */
	if ((tempb2 & 0x02) > ((*msg1).cur_mode & 0x02))
		if (tempw0 < 500)
			tempb2 = 0x01;  //ͨ�粻�㲻����� Locking in the fan mode
	(*msg1).t0a = tempw0;
	//-------------------------


	if ((tempb2 & 0x02) >((*msg1).cur_mode & 0x02)) //target is cooling mode but current is not
	{
		if ((*msg1).cur_freq >= 6) /* Ƶ��>6Hz������� */
		{
			tempb2 = 0x01;
			(*msg1).addl = 0x02;    /* ��Ƶ */
		}
	}
	else
		(*msg1).addl = 0x00;  /* ��Ƶ */

	tempb1 = (*msg1).target_mode;  /* ȡ�˿���Ϣ */
	tempb1 &= 0x0c;	//extract 1100
	tempb2 |= tempb1;     /* ȡ������Ϣ */
	(*msg1).target_modem = tempb2;
	/* ȷ����ǰ���� */
	tempb1 = (*msg1).target_modem & 0x03;   /* ������ */

	tempb2 = (*msg1).cur_mode & 0x03;   /* ��ǰ���� */
	(*msg1).cur_mode &= 0xfc; /* ֻ������ǰ�˿� */
	switch (tempb2)
	{
	case 0x00: /* ͣ��-ֻתͨ��      */
		if (tempb1 > 0x00)  (*msg1).cur_mode |= 0x01;
		break;
	case 0x01: /* ͨ��ת���⹤�� */
		(*msg1).cur_mode |= tempb1;
		break;
	case 0x02: /* ���䲻ת���� */
		if (tempb1 == 0x03) (*msg1).cur_mode |= 0x01;
		else  (*msg1).cur_mode |= tempb1;
		break;
	case 0x03: /* ���Ȳ�ת���� */
		if (tempb1 == 0x02) (*msg1).cur_mode |= 0x01;
		else  (*msg1).cur_mode |= tempb1;
		break;
	default:break;
	}
	tempb1 = (*msg1).target_modem; /* ����ͣ��������������ݸ�λ */
	tempb2 = (*msg1).cur_mode;
	if ((tempb1 & 0x0c) != (tempb2 & 0x0c))
	{
		tempb1 &= 0x0c;
		(*msg1).cur_mode = tempb1; /* ����ͣ����ͣ��-�����仯��Ȼ */

		(*msg1).ec1 &= 0xf0;  /* ����������ݸ�λ */
		(*msg1).t0d = 0x00;
		(*msg1).t1d = 0x00;
		(*msg1).t2d = 0x00;
		(*msg1).t3d = 0x00;
		(*msg1).m0a = 0x00;
		(*msg1).m1a = 0x00;
		(*msg1).m2a = 0x00;
		(*msg1).m3a = 0x00;
	}
	/* ͣ����λ:Ϊ������׼�� */
	tempb1 = (*msg1).cur_mode;
	tempb1 &= 0x03;
	if (tempb1 == 0)   { (*msg1).t0a = 0x00;    (*msg1).addl = 0x02; }
	/* ȷ��������� */
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




void Motor_petection()   /* ���������������ʱ���ָ�(20min)  */
{
	/* ͨ�����������ʱ�� */
	/*  (*msg1).t0a++;
	if((*msg1).t0a>=1000)  (*msg1).t0a=1000;
	*/
	if ((*msg1).m0a >= I017) { (*msg1).m0a = 0; (*msg1).ec1 |= 0x01; }
	if ((*msg1).m1a >= I117) { (*msg1).m1a = 0; (*msg1).ec1 |= 0x02; }
	if ((*msg1).m2a >= I217) { (*msg1).m2a = 0; (*msg1).ec1 |= 0x04; }
	if ((*msg1).m3a >= I317) { (*msg1).m3a = 0; (*msg1).ec1 |= 0x08; }

	//UI     t0d;   /* ͨ�������������ʱ����λ50mS��20min�ָ�,��ͬ */
	//UI     t1d;   /* ����������������ʱ�� */
	//UI     t2d;   /* ѹ��������������ʱ�� */
	//UI     t3d;   /* ����ȱ���������ʱ�� */
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