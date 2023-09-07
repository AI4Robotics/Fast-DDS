#include "rtps_msg.h"

RTPSMsg::RTPSMsg()
{
	pro = 0;
	transtime = 0;
	resttime = 200;
	startime = clock();		// 接受时间 or 生成时间
	num = 0;
}

short int RTPSMsg::get_priority()
{
	return pro;
}

float RTPSMsg::ave_prio()
{
	return (float)pro / 8;
}

int RTPSMsg::get_transtime()
{
	return transtime;
}

int RTPSMsg::get_resttime()
{
	return resttime;
}

void RTPSMsg::set_pro(short int x)
{
	pro = x;
}

void RTPSMsg::set_transtime()	// 设置传输时间 T_trans = T_n - T_a
{
	clock_t nowtime = clock();
	transtime = (((double)nowtime - (double)startime) / CLOCKS_PER_SEC) * 1000;//ms;
}

clock_t RTPSMsg::get_startime()
{
	return startime;
}

void RTPSMsg::set_startime(clock_t x)
{
	startime = x;
}

int RTPSMsg::get_num()
{
	return num;
}

void RTPSMsg::set_num(int x)
{
	num = x;
}

void RTPSMsg::set_resttime()	// 剩余传输时间
{
	int a = get_priority();
	if (a == 5)
		resttime = 15 - transtime;	// 优先级为DEP包的单跳最大传输时延-传输时间
	else if (a == 4)
		resttime = 45 - transtime;
	else if (a == 1)
		resttime = 95 - transtime;
	else if (a == 0)
		resttime = 195 - transtime;
}

void RTPSMsg::set_transtime(int x, int y)
{
	transtime = x;
}