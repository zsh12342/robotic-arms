#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

#include "lcm_logger.h" // 请替换成实际的头文件名

int main(int argc, char *argv[])
{
    const char *binFilePath = "/tmp/lcm_log.bin";
    const char *csvFilePath = "/tmp/lcm_log.csv";
    int start_count = -1;
    int end_count = -1;
    std::string help_msg = "Usage: " + std::string(argv[0]) + " -i <binFilePath> -o <csvFilePath> -s <start_count> -t <end_count>\n";
    int opt;
    while ((opt = getopt(argc, argv, "i:o:s:t:h")) != -1)
    {
        switch (opt)
        {
        case 'i':
            binFilePath = optarg;
            break;
        case 'o':
            csvFilePath = optarg;
            break;
        case 's':
            start_count = std::atoi(optarg);
            break;
        case 't':
            end_count = std::atoi(optarg);
            break;
        case 'h':
            // 显示帮助信息
            std::cout << help_msg << std::endl;
            return 0;
        default:
            std::cerr << help_msg << std::endl;
            return 1;
        }
    }

    std::cout << "input binFilePath: " << binFilePath << std::endl;
    std::cout << "output csvFilePath: " << csvFilePath << std::endl;

    // 创建LogWriter对象
    LogWriter logWriter;

    // 转换为CSV格式的log文件
    logWriter.bin2Plotjuggler(binFilePath, csvFilePath, start_count, end_count);

    return 0;
}
