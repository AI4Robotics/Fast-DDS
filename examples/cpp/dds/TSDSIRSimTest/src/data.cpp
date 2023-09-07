#include <iostream>
#include <random>
#include <algorithm>
#include <fstream>
#include <cmath>
#include "data.h"

struct link* data_h, * data_t;
int secCounter = 0;
RTPSMsg generator_new_data;
int dnum = 11;

/**
 * 生成测试的待发送数据序列
 *  索引：数据包序号（1~10000），数据：数据包优先级（0，1，4，5）
 *  离散数据包（pro=5）占比：0%~50%
 *  保存数据到文件，供不同的调度算法测试
*/
void make_DataSeq(std::string data_path, int total_num, int discrete_min, int discrete_max, int gap)
{
    std::ofstream make_data_seq;
    // int total_num = 10000, discrete_min = 0, discrete_max = 5000, gap = 500;
    for (int i = discrete_min; i <= discrete_max; i += gap) {
        std::string str = std::to_string(int(i * 1.0 / total_num * 100));
        std::cout << "Percentage of discrete packets: " << str + "%.txt" << std::endl;
		// ~/source/ai4robotics_lab/Fast-DDS/Fast-DDS/examples/cpp/dds/TSDSIRSimTest/data
        make_data_seq.open(data_path + "input_data_" + str + "%.txt", std::ios::trunc);

        std::vector<int> total_seq(total_num, -1);
        std::vector<int> discrete_seq;
        std::default_random_engine e(static_cast<unsigned int>(time(nullptr)));
        // std::default_random_engine e;
        std::uniform_int_distribution<unsigned int> u(0, total_num-1);
        int n = 0;
        while (n < i)
        {
            int temp = u(e);
            if (!std::count(discrete_seq.begin(), discrete_seq.end(), temp))
            {
                discrete_seq.push_back(temp);
                n++;
            }
        }
        std::sort(discrete_seq.begin(), discrete_seq.end());
        for (std::vector<int>::iterator it = discrete_seq.begin(); it != discrete_seq.end(); it++)
        {
            std::cout << (*it) << "\t";
            total_seq[(*it)] = 5;
        }
        std::cout << std::endl << "============================================================" << std::endl;
        for (std::vector<int>::iterator it = total_seq.begin(); it != total_seq.end(); it++)
        {
            if (*it == -1)
            {
                int a = rand() % 3;
                switch (a)
                {
                    case 0:
                    {
                        *it = 0;
                        break;
                    }
                    case 1:
                    {
                        *it = 1;
                        break;
                    }
                    case 2:
                    {
                        *it = 4;
                        break;
                    }
                }
            } 
            std::cout << (*it) << "\t";
            make_data_seq << (*it) << std::endl;
        }
        std::cout << std::endl;
        make_data_seq.close();
    }
}

/**
 * 初始化队列
 * 头指针 data_h, 尾指针 data_t, 长度为10的循环队列
*/
void init_queue()
{
	struct link* datap,* dataptr;
	RTPSMsg dataself;
	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(1);
	dataself.set_transtime(0,1);
	dataself.set_num(1);
	dataself.set_resttime();
	datap->next = NULL;       
	datap->s = dataself;
	data_h = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(5);//5
	dataself.set_transtime(0,1);//11
	dataself.set_num(2);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(4);//4
	dataself.set_transtime(0, 1);//23
	dataself.set_num(3);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(4);
	dataself.set_transtime(0, 1);
	dataself.set_num(4);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(4);
	dataself.set_transtime(0, 1);
	dataself.set_num(5);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(5);//5
	dataself.set_transtime(0, 1);//14
	dataself.set_num(6);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(1);
	dataself.set_transtime(0, 1);
	dataself.set_num(7);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(0);
	dataself.set_transtime(0, 1);
	dataself.set_num(8);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(1);
	dataself.set_transtime(0, 1);
	dataself.set_num(9);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	datap = (struct link*)malloc(sizeof(struct link));
	if (datap == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	dataself.set_pro(0);
	dataself.set_transtime(0, 1);
	dataself.set_num(10);
	dataself.set_resttime();
	datap->next = NULL;
	datap->s = dataself;
	dataptr->next = datap;
	dataptr = datap;

	data_t = dataptr;
	data_t->next = data_h;
	dnum = 11;	// 新生成的数据的序号从11开始
}

/**
 * 生成数据发送序列
 * total_num 待发送数据包总数
 * discrete_num 待发送数据中离散数据包总数
*/
void generator(std::vector<int> data_seq)
{
	generator_new_data.set_pro(data_seq[dnum-11]);
	generator_new_data.set_resttime();
	generator_new_data.set_startime(clock());
	generator_new_data.set_num(dnum);
	dnum++;
}
