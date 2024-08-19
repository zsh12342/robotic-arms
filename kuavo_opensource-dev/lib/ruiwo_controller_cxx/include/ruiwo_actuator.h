#ifndef RUIWO_ACTUATOR_CPP_H
#define RUIWO_ACTUATOR_CPP_H

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <iostream>
#include <iterator>
#include <time.h>
#include <chrono>
#include "ruiwoSDK.h"

class RuiWoActuator
{
 public:
    enum class State {
        None,
        Enabled,
        Disabled
    };

    struct MotorStateData {
        uint8_t id;
        State   state;
        MotorStateData():id(0x0), state(State::None) {}
        MotorStateData(uint8_t id_, State state_):id(id_), state(state_) {}
    };    
    using MotorStateDataVec = std::vector<MotorStateData>;
public:
    RuiWoActuator(std::string unused = "");
    ~RuiWoActuator();
    int initialize();
    void enable();
    void disable();
    void go_to_zero();
    void set_zero();
    void set_positions(const std::vector<uint8_t> &index, const std::vector<double> &positions, const std::vector<double> &torque, const std::vector<double> &velocity);
    void set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque);
    void set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity);
    void set_joint_state(int index, const std::vector<float> &state);
    std::vector<std::vector<float>> get_joint_state();
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    void close();
    MotorStateDataVec get_motor_state();
    
private:
    void control_thread();
    void get_parameter();
    void get_config(const std::string &config_file);
    float measure_head_torque(float position);
    std::string get_home_path();
    void interpolate_move(const std::vector<float> &start_positions, const std::vector<float> &target_positions, float speed, float dt);
    std::vector<std::vector<float>> interpolate_positions_with_speed(const std::vector<float> &a, const std::vector<float> &b, float speed, float dt);
    void send_positions(const std::vector<int> &index, const std::vector<float> &pos, const std::vector<float> &torque, const std::vector<float> &velocity);
    std::vector<int> get_joint_addresses(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<bool> get_joint_online_status(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<std::vector<int>> get_joint_parameters(const YAML::Node &config, const std::string &joint_type, int count);
    // 关节ID
    std::vector<int> Left_joint_address = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    std::vector<int> Right_joint_address = {0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
    std::vector<int> Head_joint_address = {0x0D, 0x0E};
    // 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel] vel的pid只有在servo模式下才起作用
    static inline const std::vector<std::vector<int>> Left_joint_parameter = {{0, 25, 8, 0, 0, 0, 0},
                                                                              {0, 20, 6, 0, 0, 0, 0},
                                                                              {0, 20, 6, 0, 0, 0, 0},
                                                                              {0, 10, 3, 0, 0, 0, 0},
                                                                              {0, 10, 3, 0, 0, 0, 0},
                                                                              {0, 10, 3, 0, 0, 0, 0}};
    static inline const std::vector<std::vector<int>> Right_joint_parameter = {{0, 25, 8, 0, 0, 0, 0},
                                                                               {0, 20, 6, 0, 0, 0, 0},
                                                                               {0, 20, 6, 0, 0, 0, 0},
                                                                               {0, 10, 3, 0, 0, 0, 0},
                                                                               {0, 10, 3, 0, 0, 0, 0},
                                                                               {0, 10, 3, 0, 0, 0, 0}};
    static inline const std::vector<std::vector<int>> Head_joint_parameter = {{0, 4, 3, 0, 0, 0, 0},
                                                                              {0, 10, 6, 0, 0, 0, 0}};
    // 反转电机ID
    static inline  std::vector<int> Negtive_joint_address_list = {0x01, 0x03, 0x05, 0x06, 0x07, 0x0C, 0x0D};
    // 小臂电机ID
    static inline  std::vector<int> Unnecessary_go_zero_list = {0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C};
    // ptm:力控模式 servo:伺服模式
    static inline  std::string Control_mode = "ptm";

    std::vector<bool> Left_joint_online;
    std::vector<bool> Right_joint_online;
    std::vector<bool> Head_joint_online;
    std::vector<std::vector<int>> Joint_parameter_list;
    std::vector<int> Joint_address_list;
    std::vector<bool> Joint_online_list;

    // RUIWOTools ruiwo;
    bool thread_running;
    bool thread_end;
    std::thread control_thread_;
    std::mutex sendpos_lock;
    std::mutex recvpos_lock;
    std::mutex sendvel_lock;
    std::mutex recvvel_lock;
    std::mutex sendtor_lock;
    std::mutex recvtor_lock;
    std::mutex state_lock;
    std::mutex update_lock;

    bool target_update;
    std::vector<float> target_positions;
    std::vector<float> target_velocity;
    std::vector<float> target_torque;
    std::vector<int> target_pos_kp;
    std::vector<int> target_pos_kd;
    std::vector<int> target_vel_kp;
    std::vector<int> target_vel_kd;
    std::vector<int> target_vel_ki;

    std::vector<float> old_target_positions;
    std::vector<float> current_positions;
    std::vector<float> current_torque;
    std::vector<float> current_velocity;
    std::vector<std::vector<float>> joint_status;

    float head_low_torque;
    float head_high_torque;
};

#endif // RUIWO_ACTUATOR_H