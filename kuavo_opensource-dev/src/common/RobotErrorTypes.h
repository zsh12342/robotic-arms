// RobotErrorManager.h

#ifndef ROBOT_ERROR_MANAGER_H
#define ROBOT_ERROR_MANAGER_H

#include <map>
#include <string>
#include <queue>
#include <string>
#include <mutex>
#include <iostream>

// 自定义机器人错误类型
enum RobotErrorType
{
    ROBOT_OK,
    ERROR_CRITICAL,
    ERROR_MAJOR,
    ERROR_MINOR,
    ERROR_UNKNOWN,
    ERROR_MPC_FAILED,
    // 添加其他错误类型...
};
struct RobotException
{
    std::string message;
    RobotErrorType error_type;
    RobotException(RobotErrorType error_type_, const std::string &msg) : message(msg), error_type(error_type_) {}
};
extern const std::map<RobotErrorType, std::string> errorTypeDescriptions;

class RobotErrorManager
{
public:
    // 构造函数
    RobotErrorManager();

    // 析构函数
    ~RobotErrorManager();

    // 触发机器人异常并将异常放入队列
    void trigger(RobotErrorType errorType, const std::string &message = "");
    // void processErrorQueue();

    RobotErrorType CurrentRobotState();
    bool isError();
    RobotException getFirstError();


private:
    // 异常结构体

    // 异常队列
    std::queue<RobotException> errorQueue;
    std::string prev_error_message;

    // 互斥锁，用于保护异常队列
    std::mutex errorQueueMutex;
};
extern RobotErrorManager KuavoErrorManager;
#endif // ROBOT_ERROR_TYPES_H
