#pragma once
#include <ctime>

#define PRO_NUM 8   // 优先级别总数

class RTPSMsg           // 数据包格式
{
    private:
        short int pro;      // 优先级
        int transtime;      // 已传输时间
        int resttime;       // 剩余传输时间
        clock_t startime;   // 接受时间 / 生成时间
        int num;            // 数据报文编号

    public:
        RTPSMsg();
        short int get_priority();
        float ave_prio();
        int get_transtime();
        int get_resttime();
        clock_t get_startime();
        void set_pro(short int x);
        void set_transtime();
        void set_transtime(int x, int y);
        void set_resttime();
        void set_startime(clock_t x);
        int get_num();
        void set_num(int x);
};

struct link     // 关于RTPS数据包的链表结构
{
    RTPSMsg s;
    struct link* next;
};

