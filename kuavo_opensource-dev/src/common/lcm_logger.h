#ifndef LOG_WRITER_H
#define LOG_WRITER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unordered_set>
#include <thread>
#include <random>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <iomanip>
#include <queue>
// Log entry structure
struct rawLogEntry
{
    double timestamp;
    std::string name;
    Eigen::VectorXd vector;
};
// Log entry structure
struct LogEntry
{
    double timestamp;
    std::string name;
    double value;
};
extern std::mutex bufferMutex;
class BinLogReader
{
public:
    BinLogReader(const std::string &filename) : filename_(filename) {}
    std::vector<rawLogEntry> readBinEntries(int start_index=-1, int end_index=-1);

private:
    std::string filename_;
};

// Log writer class
class LogWriter
{
public:
    LogWriter(uint64_t max_file_size_MB = 300) : max_file_size(max_file_size_MB * 1024 * 1024), running(true), bufferTime(100){};
    ~LogWriter(){};

    void start(const std::string &filename);
    void startBinLog(const std::string &filename);

    void stop();
    void stopLogBin();

    void logData(const rawLogEntry logdata);
    void logData(const std::string &name, const double value);
    void logData(const std::string &name, const Eigen::VectorXd &data);
    void bin2Plotjuggler(const std::string &bin_file, const std::string &csv_file, int start_index = -1, int end_index = -1);

    inline void set_timestamp(const std::time_t &timestamp){
        this->timestamp = timestamp;
    }

    inline void set_logfilename_suffix(const std::string &suffix){
        this->logfilename_suffix = suffix;
    }

private:
    int file_count = 0;
    int max_count = 5;      // 保存到/tmp目录的日志最大数量
    int max_save_count = 4; // 最后合并存储进硬盘的日志数量, 存储总大小会是 max_save_count * max_file_size
    uint64_t max_file_size; // 100MB
    std::ofstream logBinFile;
    std::ofstream logFile;
    std::string log_filename;
    std::string bin_log_filename;
    std::string origin_filename;
    std::string logfilename_suffix;
    std::thread logThread, binLogThread;
    std::condition_variable cv;
    std::map<std::string, std::map<std::string, double>> data;
    std::unordered_map<std::string, int> headerIndices;
    std::vector<std::string> headerNames;
    bool running;
    int bufferTime;
    std::vector<rawLogEntry> buffer;
    std::queue<rawLogEntry> buffer_queue;
    std::mutex bufferMutex;
    std::mutex logMutex;
    std::vector<std::string> new_headernames;
    std::time_t timestamp;
    void writBinaryData(std::vector<rawLogEntry> &buffer);

    void logThreadFunc();
    void processData(std::vector<rawLogEntry> &entries);
};

#endif // LOG_WRITER_H
