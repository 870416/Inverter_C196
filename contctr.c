#include "data.h"
#include "contctr.h"
#include "data.h"

void mc_off(unsigned int cur_mode)    /* �Ӵ�������֮�ֶ� */
{
	if (cur_mode & 0x10);  /* ͨ����ֶ� */
	else
	{
		disable(); hso_command = 0x00; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x20);  /* �������ֶ� */
	else
	{
		disable(); hso_command = 0x01; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x40);  /* ѹ�����ֶ� */
	else
	{
		disable(); hso_command = 0x02; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x80);  /* ����ȷֶ� */
	else
	{
		disable(); hso_command = 0x03; hso_time = timer1 + 4; enable();
	}

	cur_mode &= 0x0c;
	switch (cur_mode)
	{
	case 0x04:  /* һ�� */
	{
		disable(); hso_command = 0x05; hso_time = timer1 + 4; enable();
		break;
	}
	case 0x08:  /* ���� */
	{
		disable(); hso_command = 0x04; hso_time = timer1 + 4; enable();
		break;
	}
	default:    /* ��Чͣ�� */
	{
		disable(); hso_command = 0x04; hso_time = timer1 + 4; enable();
		//tempb2 += 2; tempb2 -= 2;
		disable(); hso_command = 0x05; hso_time = timer1 + 4; enable();
		break;
	}
	}
	return;
}

void mc_on(unsigned int cur_mode)    /* �Ӵ�������֮�պ� */
{

	if (cur_mode & 0x10)  /* ͨ��� */
	{
		disable(); hso_command = 0x20; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x20)  /* ������ */
	{
		disable(); hso_command = 0x21; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x40)  /* ѹ���� */
	{
		disable(); hso_command = 0x22; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x80)  /* ����� */
	{
		disable(); hso_command = 0x23; hso_time = timer1 + 4; enable();
	}

	cur_mode &= 0x0c;
	switch (cur_mode)  /* �ȶϺ�� */
	{
	case 0x04:  /* һ�� */
	{
		disable(); hso_command = 0x24; hso_time = timer1 + 4; enable();
		break;
	}
	case 0x08:  /* ���� */
	{
		disable(); hso_command = 0x25; hso_time = timer1 + 4; enable();
		break;
	}
	default:    /* ��Чͣ�� */
	{
		break;
	}
	}
	return;
}