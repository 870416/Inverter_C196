#include	"data.h"
#include	"proc.h"

void read_adport(port0_num)  /* ��ѯ��ʽ����,6�� */
BYTE port0_num;
{
	unsigned int ad_res_temp;
	UI min = 0xffff;    /* ��Сֵ */
	UI max = 0x0000;    /* ���ֵ */
	unsigned int ad_res = 0x0000;    /* �ۼӺ� */
	port0_num += 0x08;  /* 10λ���������������ж�*/

	for (tempb1 = 0; tempb1 < 0x06; tempb1++)
	{
		ad_command = port0_num;
		while (ad_result_lo & 0x08)
			E_watchdog;
		ad_res_temp = ad_result_hi;
		ad_res_temp <<= 8;
		ad_res_temp += ad_result_lo;
		ad_res_temp >>= 6;
		if (ad_res_temp < min) min = ad_res_temp;
		if (ad_res_temp > max) max = ad_res_temp;
		ad_res += ad_res_temp;
		E_watchdog;

	}
	ad_res -= min;
	ad_res -= max;
	ad_res >>= 2;
	port0_num -= 0x08;
	switch (port0_num)   /* 5*k/2^10 */
	{
	case 0:  /* k=20��0.25 */
	{
		ad_res *= 5; ad_res >>= 10; ad_res *= 5; (*msg1).im0 = ad_res;
		min = (*msg1).m0a; min >>= 10; (*msg1).m0a -= min;/* -1/1024 */
		if (ad_res > I013)
		{
			ad_res -= I013;
			tempbw0.ll = ad_res;
			tempbw0.ll *= ad_res;
			if (tempbw0.ll > I017) { (*msg1).m0a = I017; (*msg1).m0a++; }
			else  (*msg1).m0a += tempbw0.bw.lo;
		}
		break;
	}
	case 1:  /* k=20��0.5 */
	{
		ad_res *= 10; ad_res >>= 10; ad_res *= 5; (*msg1).im1 = ad_res;
		min = (*msg1).m1a; min >>= 10; (*msg1).m1a -= min;/* -1/1024 */
		if (ad_res > I113)
		{
			ad_res -= I113;
			tempbw0.ll = ad_res;
			tempbw0.ll *= ad_res;
			if (tempbw0.ll > I117) { (*msg1).m1a = I117; (*msg1).m1a++; }
			else  (*msg1).m1a += tempbw0.bw.lo;
		}
		break;
	}
	case 2:  /* k=20��2.5 */
	{
		ad_res *= 50; ad_res >>= 10; ad_res *= 5; (*msg1).im2 = ad_res;
		min = (*msg1).m2a; min >>= 10; (*msg1).m2a -= min;/* -1/1024 */
		if (ad_res > I213)
		{
			ad_res -= I213;
			tempbw0.ll = ad_res;
			tempbw0.ll *= ad_res;
			if (tempbw0.ll > I217) { (*msg1).m2a = I217; (*msg1).m2a++; }
			else  (*msg1).m2a += tempbw0.bw.lo;
		}
		break;
	}
	case 3:  /* k=20��2.5 */
	{
		ad_res *= 50; ad_res >>= 10; ad_res *= 5; (*msg1).im3 = ad_res;
		min = (*msg1).m3a; min >>= 10; (*msg1).m3a -= min;/* -1/1024 */
		if (ad_res > I313)
		{
			ad_res -= I313;
			tempbw0.ll = ad_res;
			tempbw0.ll *= ad_res;
			if (tempbw0.ll > I317) { (*msg1).m3a = I317; (*msg1).m3a++; }
			else  (*msg1).m3a += tempbw0.bw.lo;
		}
		break;
	}
	case 4: ad_res *= 25; ad_res >>= 8; (*msg1).i1 = ad_res; break;
	case 5: ad_res *= 25; ad_res >>= 7; (*msg1).u1 = ad_res; break;
	case 6: ad_res *= 5; ad_res >>= 8; (*msg1).i2 = ad_res; break;
	case 7: ad_res *= 31; ad_res >>= 5; (*msg1).u2_l = ad_res;
		ad_res >>= 8; (*msg1).u2_h = ad_res; break;
	default:                                             break;
	}
	return;
}



void proc_RS485_buff()
{
	uchar idx = 0;
	uchar temp = 0;
	(*msg2).ext_setting_modeA = 0x00;
	(*msg2).ext_setting_modeB = 0x00;
	(*msg2).ext_setting_modeC = 0x00;

	for (idx = 10; idx < 100;)
	{
		if ((*(rec_buf_ptr + idx) == 0xaa) && ((*msg2).ext_setting_modeC == 0x00))
		{  /*  C�������б�  */
			idx++;
			temp += *(rec_buf_ptr + idx);
			(*msg2).ext_setting_modeC = *(rec_buf_ptr + idx);
			idx++;
			temp += *(rec_buf_ptr + idx);
			if (temp == 0xff)
			{
				idx++;
				temp = *(rec_buf_ptr + idx);
				if (temp != 0xfd) (*msg2).ext_setting_modeC = 0x00;
			}
			else (*msg2).ext_setting_modeC = 0x00;
		}

		if ((*(rec_buf_ptr + idx) == 0x5a) && ((*msg2).ext_setting_modeB == 0x00))
		{  /*  B�������б�  */
			idx++;
			temp = *(rec_buf_ptr + idx);
			(*msg2).ext_setting_modeB = temp;
			idx++;
			temp += *(rec_buf_ptr + idx);
			if (temp == 0xff)
			{
				idx++;
				temp = *(rec_buf_ptr + idx);
				if (temp != 0xfd) (*msg2).ext_setting_modeB = 0x00;
			}
			else (*msg2).ext_setting_modeB = 0x00;
		}

		if ((*(rec_buf_ptr + idx) == 0xa5) && ((*msg2).ext_setting_modeA == 0x00))
		{  /*  A�������б�  */
			idx++;
			temp = *(rec_buf_ptr + idx);
			(*msg2).ext_setting_modeA = temp;
			idx++;
			temp += *(rec_buf_ptr + idx);
			if (temp == 0xff)
			{
				idx++;
				if (*(rec_buf_ptr + idx) != 0xfd) 
					(*msg2).ext_setting_modeA = 0x00;
			}
			else
				(*msg2).ext_setting_modeA = 0x00;
		}
		idx++;
	}




	idx = 0x00;   /* ~Ϊ��λȡ����!Ϊ�߼��ǣ������ */
	/* Ϊ1��Ч,6/C����Ч��5/B��?��4/A��?��2/C���,1/B��?,0/A��? */
	if ((*msg2).ext_setting_modeA == 0x00)  
		idx |= 0x01;
	if ((*msg2).ext_setting_modeB == 0x00)
		idx |= 0x02;
	if ((*msg2).ext_setting_modeC == 0x00)  
		idx |= 0x04;

	if (idx & 0x01) /* A��ͨѶ�����������ʶ */
	{
		(*msg2).rs_ea++; 
		idx |= 0x10; /* ͨѶ����������Ч */
		if ((*msg2).rs_ea >= 100) 
			(*msg2).rs_ea = 100;
	}
	else
	{
		(*msg2).rs_ea = 0; /* �˿���Ϊ0x00��������Ч��������Ч */
		if ((*msg2).ext_setting_modeA & 0x0c) 
			idx &= 0xef;
		else 
			idx |= 0x10;
	}
	if (idx & 0x02) /* B��ͨѶ�����������ʶ */
	{
		(*msg2).rs_eb++; idx |= 0x20;
		if ((*msg2).rs_eb >= 100) (*msg2).rs_eb = 100;
	}
	else
	{
		(*msg2).rs_eb = 0;
		if ((*msg2).ext_setting_modeB & 0x0c) idx &= 0xdf;
		else idx |= 0x20;
	}
	if (idx & 0x04) /* C��ͨѶ�����������ʶ */
	{
		(*msg2).rs_ec++; idx |= 0x40;
		if ((*msg2).rs_ec >= 100) (*msg2).rs_ec = 100;
	}
	else
	{
		(*msg2).rs_ec = 0;
		if ((*msg2).ext_setting_modeC & 0x0c) idx &= 0xbf;
		else idx |= 0x40;
	}



	(*msg2).ext_setting_mode = 0x00;
	if (idx & 0x40)  /* ����ȷ�������ȼ���C�� > B�� > A�� */
	{
		if (idx & 0x20)
		{
			if (idx & 0x10);
			else 
				(*msg2).ext_setting_mode = (*msg2).ext_setting_modeA;
		}
		else
			(*msg2).ext_setting_mode = (*msg2).ext_setting_modeB;
	}
	else
		(*msg2).ext_setting_mode = (*msg2).ext_setting_modeC;


	if ((*msg2).ext_setting_mode == 0x00) /* ��ʱ����Ч�����򲻸ı乤�� */
		(*msg2).ext_setting_mode = (*msg1).cur_mode;

	if ((idx & 0x70) != 0x70)
		(*msg2).rs_er = 0;      /* �оͲ�Ϊ�� */
	else
	{
		(*msg2).rs_er++; /* ��������Ч�����ʱ */
		if ((*msg2).rs_er >= 100)
		{
			(*msg2).rs_er = 100; 
			(*msg2).ext_setting_mode = 0x00;
		}
	}

	if ((*msg2).rs_er >= 100)
		(*msg1).ec1 |= 0x80; /* ͨѶ�����ʶ */
	else  
		(*msg1).ec1 &= 0x7f;
	if ((*msg2).rs_ec >= 100) 
		(*msg1).ec1 |= 0x40;
	else
		(*msg1).ec1 &= 0xbf;
	if ((*msg2).rs_eb >= 100) 
		(*msg1).ec1 |= 0x20;
	else
		(*msg1).ec1 &= 0xdf;
	if ((*msg2).rs_ea >= 100) 
		(*msg1).ec1 |= 0x10;
	else
		(*msg1).ec1 &= 0xef;

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
	tempw0 = (*msg1).cur_freqc;
	if (tempw0 > 500) tempw0 = 500;	//TODO: 500 is invalid value?? since max limit = 350?
	/* tempw0=300;        */
	/* �ֶ���ȷ��,ת��Ƶ��<8K */
	if (tempw0 < 111)  num2[1] = 120;
	else if (tempw0 < 167)  num2[1] = 80;
	else if (tempw0 < 222)  num2[1] = 60;
	else if (tempw0 < 278)  num2[1] = 48;
	else if (tempw0 < 333)  num2[1] = 40;
	else if (tempw0 < 444)  num2[1] = 30;
	else if (tempw0 < 556)  num2[1] = 24;
	else if (tempw0 < 667)  num2[1] = 20;
	else                    num2[1] = 15;

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

	// Read corresponding output voltage for the freq. from v/f table
	ptrw1 = (UI *)ADDR_WID;
	tempbw0.ll = *(ptrw1 + tempw0); /* ����ֱ���������ֳ� */
	tempbw0.ll *= 1060;  /* ��ѹ���� 530V*2    */

	/* ȡ����������Ƶ */
	tempbw0.ll /= 530;
	tempw0 = tempbw0.bw.lo;

	/* ���ָ�� */
	tempw4 = 240;
	tempw4 /= num2[1];
	tempb0 = tempw4;  /* ˫�ֽ� */
	tempw4 >>= 1;
	tempw5 = 240;     /* ��Ҫ�ø�����ʽ */
	tempw5 -= tempb0;
	tempw5 += tempw4;

	ptrw1 = (UI *)ADDR_SIN;
	ptrw2 = (UI *)ADDR_SIN;
	ptrw1 += tempw4;       /*  SIN��A��*/
	ptrw2 += tempw5;       /*  SIN��60-A��*/

	if (s_target_mode == (BYTE)0xff)
		ptrw3 = (UI *)ADDR_CA0;
	else
		ptrw3 = (UI *)ADDR_CA1;


	tempbw0.ll = (*msg1).cur_freqc;
	tempbw0.ll *= num2[1]; /* 10F*N */
	tempw7 = tempbw0.bw.lo;
	tempbw0.ll = 0x196e6b;       /* 10*fosc/6/16 */
	tempbw0.ll /= tempw7;
	tempw1 = tempbw0.bw.lo;      /* Ts */

	tempbw0.ll = tempw0;
	tempbw0.ll *= tempw1;
	tempw2 = tempbw0.bw.lo;  /* TS*2^10*M ���� */
	tempw3 = tempbw0.bw.hi;  /* TS*2^10*M ���� */

	/* ʱ����� */
	for (tempb1 = 0; tempb1 < num2[1]; tempb1++)
	{
		tempw4 = *ptrw1;
		tempbw0.ll = tempw4;
		tempbw0.ll *= tempw2;
		if (tempw3 == 0)
		{
			tempbw1.ll = 0;
		}
		else
		{
			tempbw1.ll = tempw4;
			tempbw1.ll *= tempw3;
		}
		tempbw1.ll += tempbw0.bw.hi;
		tempbw1.ll >>= 4;    /* /2^20 */
		tempw4 = tempbw1.bw.lo;
		ptrw1 += tempb0;

		tempw5 = *ptrw2;
		tempbw0.ll = tempw5;
		tempbw0.ll *= tempw2;
		if (tempw3 == 0)
		{
			tempbw1.ll = 0;
		}
		else
		{
			tempbw1.ll = tempw5;
			tempbw1.ll *= tempw3;
		}
		tempbw1.ll += tempbw0.bw.hi;
		tempbw1.ll >>= 4;    /* /2^20 */
		tempw5 = tempbw1.bw.lo;
		ptrw2 -= tempb0;
		tempw7 = tempw4;
		tempw7 += tempw5;
		if (tempw1 >= tempw7)
		{
			tempw6 = tempw1;
			tempw6 -= tempw7;
		}
		else
		{
			tempw6 = 0;
			tempw7 -= tempw1;
			tempw7 >>= 1;
			if (tempw4 < tempw7)
			{
				tempw5 -= tempw7;
				tempw5 -= tempw7;
			}
			else
			{
				if (tempw5 < tempw7)
				{
					tempw4 -= tempw7;
					tempw4 -= tempw7;
				}
				else
				{
					tempw4 -= tempw7;
					tempw5 -= tempw7;
				}
			}
		}
		if (tempw6 <= TIMER0_MODIFY)  
			tempw6 = 0;
		else  
			tempw6 -= TIMER0_MODIFY;

		/* ��С�������� */
		if (tempw4 <= HALF_MIN_PULSE)
			tempw4 = 0;
		else if (tempw4 <= MIN_PULSE) 
			tempw4 = MIN_PULSE;

		if (tempw5 <= HALF_MIN_PULSE)
			tempw5 = 0;
		else if (tempw5 <= MIN_PULSE) 
			tempw5 = MIN_PULSE;

		if (tempw6 <= HALF_MIN_PULSE)
			tempw6 = 0;
		else if (tempw6 <= MIN_PULSE) 
			tempw6 = MIN_PULSE;

		*ptrw3 = tempw5;/* !!,˳��!! */
		ptrw3++;
		*ptrw3 = tempw4;
		ptrw3++;
		*ptrw3 = tempw6;
		ptrw3++;
	}
	s_ok = 0xff; /* ��������Ч��־ */
	return;
}