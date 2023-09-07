#include <ctime>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include "tsdsir.h"
#include "data.h"

#define TMAX 0.8		// 控制 传输时间上下界的实数系数 R_max 和 R_min
#define TMIN 0.2
#define TPRO5 15		// 各优先级数据包的单跳剩余传输时间
#define TPRO4 45
#define TPRO1 95
#define TPRO0 195

std::ofstream resultOut;	// 调度结果输出
std::ofstream statisticsOut;	// 调度结果输出
std::vector<int> data_seq;
int all_data = 0, discrete_data = 0;	// 文件中全部数据个数和离散数据个数
std::string result_path = "/home/study/source/ai4robotics_lab/Fast-DDS/Fast-DDS/examples/cpp/dds/TSDSIRSimTest/result/";

/**
 * 统计 调度算法的丢包率和数据包的平均延迟
 * 	drop_all_num	总丢包数
 * 	delay_all_num	所有数据包的延迟时间
 * 	all_num			数据包总个数
 * */ 
int drop_5_num = 0, drop_4_num = 0, drop_1_num = 0, drop_0_num = 0;	// 各优先级数据的丢包个数
int all_5_num = 0, all_4_num = 0, all_1_num = 0, all_0_num = 0;	// 各优先级数据的总个数
double delay_5_sum = 0, delay_4_sum = 0, delay_1_sum = 0, delay_0_sum = 0;

short int choose_system = 1;			// 当前子系统
bool has_control = 0, has_neartime = 0;	// 含有离散数据和即将超时数据的标志
float aver_transtime = 0;				// 平均已等待时间
float sum_transtime = 0;				// 总传输时间
RTPSMsg newdata;							// 新生成的数据
float caltime;							// 计算 平均已等待时间
double rtmax = 0, rtmin = 0;
double boundary_time = 0;					// 上下界计算的时间
double num5 = 0, num4 = 0, num1 = 0, num0 = 0;	// 各优先级数据帧数目（当前在队列中的数据）
int transtimecalnum = 0;

int read_data(std::string datafile)
{
	discrete_data = 0, all_data = 0, data_seq.clear();
	std::ifstream r;
	std::cout << datafile << std::endl;
	r.open(datafile, std::ios::in);
	if (r.is_open())
	{
		std::cout << "======文件打开成功======" << std::endl;
		int temp;
		while (r >> temp)
		{
			// std::cout << temp << std::endl;
			if (temp == 5)
			{
				discrete_data++;
			}
			all_data++;
			data_seq.push_back(temp);
		}
		std::cout << "all_data: " << all_data 
			<< ", discrete_data: " << discrete_data << std::endl;
		std::cout << "len of vector: " << data_seq.size() << std::endl;
		return 1;		
	}
	else
	{
		std::cout << "======文件打开失败======" << std::endl;
		return 0;
	}
}

void print(struct link* head) {
	int i = 1;
	struct link* temp = head;
	std::cout << "--------------------------------------------------------------" << std::endl;
	std::cout << "| #" << i << " num: " << temp->s.get_num() << ", pro: " << temp->s.get_priority() 
			<< ", starttime: " << temp->s.get_startime() << ", transtime: " << temp->s.get_transtime() 
			<< ", restime: " << temp->s.get_resttime() << " |" << std::endl;
	temp = temp->next;
	while (temp != data_h)
	{
		std::cout << "--------------------------------------------------------------" << std::endl;
		std::cout << "| #" << ++i  << " num: " << temp->s.get_num() << ", pro: " << temp->s.get_priority() 
			<< ", starttime: " << temp->s.get_startime() << ", transtime: " << temp->s.get_transtime() 
			<< ", restime: " << temp->s.get_resttime() << " |" << std::endl;
		
		temp = temp->next;
	}
	std::cout << "--------------------------------------------------------------" << std::endl;
}

// 判断是否有控制信息，是否有快超时报文以计算上下界
void judge_has_control()
{
	struct link* a;
	a = data_h;
	has_control = 0; 
	has_neartime = 0;
	num5 = 0; num4 = 0; num1 = 0; num0 = 0; boundary_time = 0;
	for(int i = 0; i < DATANUM; i++)
	{
		if (a->s.get_priority() == 5)
		{
			has_control = 1;	// 包含控制信息/离散数据
			num5++;
			if (a->s.get_resttime() <= 1)	// 优先级为5的数据包 剩余时间小于 1
				has_neartime = 1;
		}	
		else if (a->s.get_priority() == 4)
		{
			num4++;
			if (a->s.get_resttime() <= 1)	
				has_neartime = 1;
		}
		else if (a->s.get_priority() == 1)
		{
			num1++;
			if (a->s.get_resttime() <= 4)	// 优先级为1的数据包 剩余时间小于 4
				has_neartime = 1;
		}
		else if (a->s.get_priority() == 0)
		{
			num0++;
			if (a->s.get_resttime() <= 8)	// 优先级为0的数据包 剩余时间小于 8
				has_neartime = 1;
		}
		a = a->next;
	}
	boundary_time = TPRO0 * (num0 / DATANUM) + TPRO1 * (num1 / DATANUM) + TPRO4 * (num4 / DATANUM) + TPRO5 * (num5 / DATANUM);
	resultOut << boundary_time * TMAX << ", ";
	resultOut << boundary_time * TMIN << ", ";
}

// 计算平均已等待时间
float cal_transtime()
{
	sum_transtime = 0;
	struct link* a;
	a = data_h;
	for (int i = 0; i < DATANUM; i++)
	{
		sum_transtime += ((a->s.ave_prio() + 1) * a->s.get_transtime());
		a = a->next;
	}
	//std::cout << "总传输时间" << sum_transtime << std::endl;
	aver_transtime = sum_transtime / DATANUM;
	resultOut << aver_transtime << std::endl;
	//std::cout << "平均已等待时间" << aver_transtime << std::endl;
	return aver_transtime;
}

// 新报文插入
void insertdata(RTPSMsg x)
{
	struct link* temp, * h = data_h, * ptr;
	temp = (struct link*)malloc(sizeof(struct link));
	if (temp == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	temp->s = x;
	temp->next = NULL;

	// 子系统1 优先级优先队列
	if (choose_system == 1)
	{	
		// 头节点数据的优先级 小于 新数据的优先级(优先级相等，剩余时间较小的优先/已等待时间较大的优先)，则新数据插入到当前头节点之前
		if (h->s.get_priority() < x.get_priority() || (h->s.get_priority() == x.get_priority() && h->s.get_transtime() <= x.get_transtime()))
		{
			data_t->next = temp;
			temp->next = h;
			data_h = temp;
			return;
		}
		else
		{
			ptr = h;
			h = h->next;
			while (h != data_h && h->s.get_priority() > x.get_priority())
			{
				ptr = h;
				h = h->next;
			}
			if (h->s.get_priority() == x.get_priority())
			{
				while (h != data_h && h->s.get_transtime() >= x.get_transtime() && h->s.get_priority() == x.get_priority())
				{
					ptr = h;
					h = h->next;
				}
				if (h == data_h)
				{
					ptr->next = temp;
					temp->next = data_h;
					data_t = temp;
				}
				else
				{
					ptr->next = temp;
					temp->next = h;
					return;
				}
			}
			else if (h->s.get_priority() < x.get_priority())
			{
				ptr->next = temp;
				temp->next = h;
				return;
			}
			else
			{
				ptr->next = temp;
				temp->next = data_h;
				data_t = temp;
			}
		}
	}
	// 子系统2 时间优先队列
	else
	{
		if (h->s.get_resttime() > x.get_resttime() || (h->s.get_priority() <= x.get_priority() && h->s.get_resttime() == x.get_resttime()))
		{
			data_t->next = temp;
			temp->next = h;
			data_h = temp;
			return;
		}
		else
		{
			ptr = h;
			h = h->next;
			while (h != data_h && h->s.get_resttime() < x.get_resttime())
			{
				ptr = h;
				h = h->next;
			}
			if (h->s.get_resttime() == x.get_resttime())
			{
				while (h != data_h && h->s.get_priority() >= x.get_priority()&& h->s.get_resttime() == x.get_resttime())
				{
					ptr = h;
					h = h->next;
				}
				if (h == data_h)
				{
					ptr->next = temp;
					temp->next = data_h;
					data_t = temp;
				}
				else
				{
					ptr->next = temp;
					temp->next = h;
					return;
				}
			}
			else if (h->s.get_resttime() > x.get_resttime())
			{
				ptr->next = temp;
				temp->next = h;
				return;
			}
			else
			{
				ptr->next = temp;
				temp->next = data_h;
				data_t = temp;
			}
		}
	}
}

// 子系统1 排序（插入排序）
struct link* InsertNode1(struct link* head, RTPSMsg nodeData)
{
	struct link* p = head, * pr = head, * temp = NULL;

	p = (struct link*)malloc(sizeof(struct link));
	if (p == NULL)
	{
		std::cout<<"No enough meomory!"<<std::endl;
		exit(0);
	}
	p->next = NULL;				// 待插入节点指针域为空指针
	p->s = nodeData;

	if (head == NULL)    		// 若原链表为空，插入节点作为头节点
	{
		head = p;       		
	}
	else      					// 若原链表不为空
	{
		while (pr->s.get_transtime() > nodeData.get_transtime() && pr->next != NULL)
		{
			temp = pr;        	// 保存当前节点
			pr = pr->next;    	// pr指向当前节点的下一个节点
		}
		if (pr->s.get_transtime() <= nodeData.get_transtime())
		{
			if (pr == head)     // 在头节点插入新节点
			{
				p->next = head; // 新节点指针域指向原链表头节点
				head = p;       // 头指针指向新节点
			}
			else
			{
				pr = temp;				// temp的传输时间大于当前p节点的传输时间
				p->next = pr->next;		// 新节点指针域指向下一个节点
				pr->next = p;           // 让前一节点指针域指向新节点
			}
		}
		else        			// 若在表尾插入新节点
		{
			pr->next = p;    	// 末节点指针域指向新节点
		}
	}

	return head;
}

// 子系统1 切换排序
void change_sort1(int num)
{
	std::cout << "******************切换到子系统1******************" << std::endl;
	struct link* pr;
	choose_system = 1;
	struct link* a[8];
	for (int i = 0; i < 8; i++)
	{
		a[i] = NULL;
	}
	struct link* t = data_h;
	for (int i = 0; i < num; i++)
	{
		a[t->s.get_priority()] = InsertNode1(a[t->s.get_priority()], t->s);
		t=t->next;
	}
	t = data_h;
	for (int i = 5; i >=0; i--)	// 按优先级大小，连接多个优先级的子链表，构成按优先级排序的链表
	{
		while(a[i] != NULL)
		{
			t->s = a[i]->s;
			pr = a[i];
			a[i] = a[i]->next;
			free(pr);
			t = t->next;
		}
	}

}

// 子系统2 排序（插入排序）
struct link* InsertNode2(struct link* head, RTPSMsg nodeData)
{
	struct link* p = head, * pr = head, * temp = NULL;

	p = (struct link*)malloc(sizeof(struct link));
	if (p == NULL)
	{
		std::cout << "No enough meomory!" << std::endl;
		exit(0);
	}
	p->next = NULL;			// 待插入节点指针赋值为空指针
	p->s = nodeData;

	if (head == NULL)    	// 若原链表为空，插入节点作头节点
	{
		head = p;        
	}
	else        			// 若原链表不为空
	{
		while (pr->s.get_resttime() < nodeData.get_resttime() && pr->next != NULL)
		{
			temp = pr;        
			pr = pr->next;    
		}
		if (pr->s.get_resttime() == nodeData.get_resttime())
		{
			while (pr->s.get_priority() > nodeData.get_priority() && pr->next != NULL)
			{
				if (pr->s.get_resttime() != nodeData.get_resttime())
					break;
		  		temp = pr;        
				pr = pr->next;    
			}

		}
		if (pr->s.get_resttime() > nodeData.get_resttime())
		{
			if (pr == head)
			{
				p->next = head;
				head = p;
			}
			else
			{
				pr = temp;
				p->next = pr->next;
				pr->next = p;    
			}
		}
		else
		{
			pr->next = p;
		}
	}
	return head;
}

// 子系统2 切换排序
void change_sort2(int num)
{
	std::cout << "#############切换到子系统2#############" << std::endl;
	//send_h = 1;
	choose_system = 2;
	struct link* h = NULL;
	struct link* pr = NULL;
	struct link* t = data_h;
	for (int i = 0; i < num; i++)
	{
		h = InsertNode2(h, t->s);
		t = t->next;
	}
	t = data_h;
	while (h != NULL)
	{
		t->s = h->s;
		pr = h;
		h = h->next;
		free(pr);
		t = t->next;
	}
}

// 调度过程(可理解为转发队首数据包，再读入一个数据包)
void schedule()
{
	struct link* a, * b;

	if (data_h->s.get_priority() == 5)
	{
		all_5_num++;
		delay_5_sum += data_h->s.get_transtime();
		if (data_h->s.get_resttime() < 0) { drop_5_num++; }
	} 
	else if (data_h->s.get_priority() == 4)
	{
		all_4_num++;
		delay_4_sum += data_h->s.get_transtime();
		if (data_h->s.get_resttime() < 0) { drop_4_num++; }
	}
	else if (data_h->s.get_priority() == 1)
	{
		all_1_num++;
		delay_1_sum += data_h->s.get_transtime();
		if (data_h->s.get_resttime() < 0) { drop_1_num++; }
	}
	else if (data_h->s.get_priority() == 0)
	{
		all_0_num++;
		delay_0_sum += data_h->s.get_transtime();
		if (data_h->s.get_resttime() < 0) { drop_0_num++; }
	}
	
	int is_loss = data_h->s.get_resttime() < 0 ? 1 : 0;
	resultOut << data_h->s.get_priority() << ", " << data_h->s.get_transtime() << ", "
		<< data_h->s.get_resttime() << ", " << is_loss << ", ";
	std::cout << "-------------> send data's num: " << data_h->s.get_num() 
		<< ", pro: " << data_h->s.get_priority() 
		<< ", transtime: " << data_h->s.get_transtime() << std::endl;
	
	// 发送队首数据包（删除队首节点）
	data_t->next = data_h->next;
	b = data_h;
	data_h = data_h->next;
	free(b);
	// print(data_h);

	// (1~DATANUM-1) 待发送队列时间增加
	a = data_h;
	for (int i = 1; i < DATANUM; i++)
	{	
		// 为模拟待发送队列中大量数据的情况，每转发一个数据包，余下数据包传输时间+5ms
		a->s.set_transtime(a->s.get_transtime() + 5, 1);
		a->s.set_resttime();
		a = a->next;
	}

	//读入新报文
	generator(data_seq);
	newdata = generator_new_data;
	insertdata(newdata);	//读下一个数据并插入
	std::cout << "<++++++++++++++++++++++++++++ generate data's num: " << newdata.get_num() << ", pro:" << newdata.get_priority() << std::endl;
	// print(data_h);
	caltime = cal_transtime();	// t_ave
}

/**
 * 混杂切换系统调度
 * datafile	待发送数据的优先级序列
*/
void tsdsir(std::string datafile)
{
	// 初始化
	choose_system = 1;
	drop_5_num = 0, drop_4_num = 0, drop_1_num = 0, drop_0_num = 0;	// 各优先级数据的丢包个数
	all_5_num = 0, all_4_num = 0, all_1_num = 0, all_0_num = 0;	// 各优先级数据的总个数
	delay_5_sum = 0, delay_4_sum = 0, delay_1_sum = 0, delay_0_sum = 0;
	// 读取文件，获取待发送数据的优先级序列，总数，离散数，离散占比
	if (read_data(datafile))
	{
		std::cout << "数据读入成功" << std::endl;
		std::string str = std::to_string(int(discrete_data * 1.0 / all_data * 100));
        std::cout << "Percentage of discrete packets: " << str + "%" << std::endl;
		// TODO---------------------------
		statisticsOut.open(result_path + "tsdsir_statistics_" + str + "%.csv", std::ios::trunc);
        resultOut.open(result_path + "tsdsir_result_" + str + "%.csv", std::ios::trunc);
		resultOut << "t_max, t_min, t_prio, t_trans, t_rest, is_loss, t_ave" << std::endl;

		judge_has_control();	// t_max, t_min
		rtmax = TMAX * boundary_time;
		rtmin = TMIN * boundary_time;
		std::cout << "01.计算当前队列的T_max: " << rtmax << ", T_min: " 
			<< rtmin << "--------------------------" << std::endl;
		if (choose_system == 1)
		{
			std::cout << "02.按(优先级)降序排列--------------------------" << std::endl;
			change_sort1(DATANUM);	// 先按优先级降序排序，再按传输时间降序排序/剩余时间升序
			// print(data_h);
			std::cout << "03.开始调度--------------------------" << std::endl;
			schedule();		// t_prio, t_trans, t_rest, is_loss, t_ave
			struct link* ht = data_h;
			for (int j = 0; j < DATANUM; j++)
			{
				//std::cout << ht->s.get_priority() << ht->s.get_resttime() << " " << std::endl;
				ht = ht->next;
			}
		}
		int k = 1;
		while(k < all_data)	// 正式调度
		{
			judge_has_control();
			rtmax = TMAX * boundary_time;
			rtmin = TMIN * boundary_time;
			std::cout << "01.计算当前队列的T_max: " << rtmax << ", T_min: " 
				<< rtmin << "--------------------------" << std::endl;
			// 判断是否满足子系统1切换到子系统2的条件
			if (((choose_system == 1) && (caltime >=rtmax))||((choose_system == 1) && (has_neartime==1)))
			{
				// std::cout << "切换到子系统2" << std::endl;
				std::cout << "02.按[时间优先]降序排列--------------------------" << std::endl;
				change_sort2(DATANUM);	//满足切换
				// print(data_h);
				std::cout << "03.开始调度--------------------------" << std::endl;
				schedule(); 
				k++;
				// std::cout << "tou" << data_h << std::endl;
			}
			// 判断是否满足子系统2切换到子系统1的条件
			else if (((choose_system == 2) && (caltime <= rtmin))|| ((choose_system == 2) && (has_control == 1)))
			{
				std::cout << "02.按(优先级)降序排列--------------------------" << std::endl;
				change_sort1(DATANUM);
				// print(data_h);
				std::cout << "03.开始调度--------------------------" << std::endl;
				schedule(); 
				k++;
				//std::cout << "tou" << data_h << std::endl;
			}
			else	//不满足切换条件
			{
				std::cout << "02+03.不用切换子系统，开始调度--------------------------" << std::endl;
				schedule();
				k++;
			}
		}
		
		int drop_all_num = drop_0_num+drop_1_num+drop_4_num+drop_5_num;
		double delay_all_sum = delay_0_sum+delay_1_sum+delay_4_sum+delay_5_sum;
		statisticsOut << "priority, sum, drop_sum, drop_rate, delay_ave," << std::endl;
		statisticsOut << "5, " << all_5_num << ", " << drop_5_num << ", " << drop_5_num*1.0/all_5_num << ", " << delay_5_sum/all_5_num << std::endl;
		statisticsOut << "4, " << all_4_num << ", " << drop_4_num << ", " << drop_4_num*1.0/all_4_num << ", " << delay_4_sum/all_4_num << std::endl;
		statisticsOut << "1, " << all_1_num << ", " << drop_1_num << ", " << drop_1_num*1.0/all_1_num << ", " << delay_1_sum/all_1_num << std::endl;
		statisticsOut << "0, " << all_0_num << ", " << drop_0_num << ", " << drop_0_num*1.0/all_0_num << ", " << delay_0_sum/all_0_num << std::endl;
		statisticsOut << "sum, " << all_data << ", " << drop_all_num << ", " 
				<< drop_all_num*1.0/all_data << ", " 
				<< delay_all_sum/all_data << std::endl;
		// 输出统计结果
		statisticsOut.close();
		resultOut.close();
		return;
	}
	else
	{
		std::cout << "数据读入失败" << std::endl;
		return;
	}
}