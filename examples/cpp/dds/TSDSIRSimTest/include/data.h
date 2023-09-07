#pragma once
#include <vector>
#include "rtps_msg.h"

#define DATANUM 10  // 待发送队列长度
#define TOTALNUM 10000   // 数据包总数
#define SECNUM 500      // 控制离散包比重5%


extern struct link* data_h, * data_t;
extern int secCounter;
extern RTPSMsg generator_new_data;
extern int alldatanum;

void make_DataSeq(std::string data_path, int total_num, int discrete_min, int discrete_max, int gap);
void init_queue();        // 初始化发送队列
void generator(std::vector<int> data_seq);       // 生成数据发送序列