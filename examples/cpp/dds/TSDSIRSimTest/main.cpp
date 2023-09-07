#include <iostream>
#include <string>
#include <ctime>
#include "rtps_msg.h"
#include "tsdsir.h"
#include "data.h"

std::string data_path = "/home/study/source/ai4robotics_lab/Fast-DDS/Fast-DDS/examples/cpp/dds/TSDSIRSimTest/data/";

/**
 * Test TSDSIR by Simulation
*/
int main(int argc, char* argv[])
{
    // 生成一组待发送数据的优先级序列，供多种调度算法测试
    int total_num = 10000, discrete_min = 0, discrete_max = 5000, gap = 500;
    make_DataSeq(data_path, total_num, discrete_min, discrete_max, gap);   // 0, 500, 1000, 1500, ... 

    for (int i = discrete_min; i <= discrete_max; i += gap)
    {
        srand(time(NULL));
        init_queue();
        std::string str = std::to_string(int(i * 1.0 / total_num * 100));
        std::string datafile = data_path + "input_data_" + str + "%.txt";
        tsdsir(datafile); // 对不同离散占比的数据分别调用 tsdsir 调度算法

        std::cout << "======discrete num: " << i << " test end.======" << std::endl;
    }
    return 0;
}