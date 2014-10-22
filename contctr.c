#include "data.h"
#include "contctr.h"
#include "data.h"

void mc_off(unsigned int cur_mode)    /* 接触器控制之分断 */
{
	if (cur_mode & 0x10);  /* 通风机分断 */
	else
	{
		disable(); hso_command = 0x00; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x20);  /* 冷凝机分断 */
	else
	{
		disable(); hso_command = 0x01; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x40);  /* 压缩机分断 */
	else
	{
		disable(); hso_command = 0x02; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x80);  /* 电加热分断 */
	else
	{
		disable(); hso_command = 0x03; hso_time = timer1 + 4; enable();
	}

	cur_mode &= 0x0c;
	switch (cur_mode)
	{
	case 0x04:  /* 一端 */
	{
		disable(); hso_command = 0x05; hso_time = timer1 + 4; enable();
		break;
	}
	case 0x08:  /* 二端 */
	{
		disable(); hso_command = 0x04; hso_time = timer1 + 4; enable();
		break;
	}
	default:    /* 无效停机 */
	{
		disable(); hso_command = 0x04; hso_time = timer1 + 4; enable();
		//tempb2 += 2; tempb2 -= 2;
		disable(); hso_command = 0x05; hso_time = timer1 + 4; enable();
		break;
	}
	}
	return;
}

void mc_on(unsigned int cur_mode)    /* 接触器控制之闭合 */
{

	if (cur_mode & 0x10)  /* 通风机 */
	{
		disable(); hso_command = 0x20; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x20)  /* 冷凝机 */
	{
		disable(); hso_command = 0x21; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x40)  /* 压缩机 */
	{
		disable(); hso_command = 0x22; hso_time = timer1 + 4; enable();
	}
	if (cur_mode & 0x80)  /* 电加热 */
	{
		disable(); hso_command = 0x23; hso_time = timer1 + 4; enable();
	}

	cur_mode &= 0x0c;
	switch (cur_mode)  /* 先断后合 */
	{
	case 0x04:  /* 一端 */
	{
		disable(); hso_command = 0x24; hso_time = timer1 + 4; enable();
		break;
	}
	case 0x08:  /* 二端 */
	{
		disable(); hso_command = 0x25; hso_time = timer1 + 4; enable();
		break;
	}
	default:    /* 无效停机 */
	{
		break;
	}
	}
	return;
}