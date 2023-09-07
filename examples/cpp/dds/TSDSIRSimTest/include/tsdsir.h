#pragma once
#include "rtps_msg.h"

void judge_has_control();   // 判断是否有离散信息
float cal_transtime();      // 计算平均已传输时间
void schedule(int total_num, int discrete_num);            // 调度过程
//struct link* InsertNode1(struct link* head, RTPS nodeData);    //子系统1排序
//struct link* InsertNode2(struct link* head, RTPS nodeData);    //子系统1排序
void change_sort1(int num); // 子系统1 切换排序
void change_sort2(int num); // 子系统2 切换排序

void tsdsir(std::string datafile);              // 混杂切换
void insertdata(RTPSMsg x);     // 新报文插入

