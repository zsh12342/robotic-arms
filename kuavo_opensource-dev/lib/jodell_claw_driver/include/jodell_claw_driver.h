#include <stdio.h>
#include <memory>
#include <algorithm>  // For std::transform
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <any>
#include <string.h>
#include <vector>
#include <iostream>
#include "jodell_tool.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <fcntl.h>
#include <unordered_map>
#include <termios.h>
#include <functional> // For std::function
#define MAX_SIZE 256

#define CLAW_LOG(...) do { \
        fprintf(stderr, "[ClawController] "); \
        fprintf(stderr, __VA_ARGS__); \
        fprintf(stderr, "\n"); \
    } while(0)

namespace ClawController
{
    template <typename T>
    bool isInRange(T value, T min = 0, T max = 255)
    {
        return value >= min && value <= max;
    };


    enum ClawType
    {
        LeftClaw = 1,
        RightClaw = 2,
    };
    class ClawParameters
    {
    public:
        ClawParameters(ClawType claw_type, double pos, double speed, double tau)
            : claw_type(claw_type), pos(pos), speed(speed), tau(tau) {}
        ClawType claw_type;
        double pos;
        double speed;
        double tau;
        double vol = 0;
        double status = 0;
    };
    using ClawDataPtr = std::shared_ptr<ClawParameters>;
    using ClawDataListPtr = std::shared_ptr<std::vector<ClawDataPtr>>;
    class ClawController
    {
    public:
        ClawController();

        /// @brief 初始化 jodell claw
        /// @param port_name 端口名称
        /// @return  = 0: success
        ///          > 0: 1 rserialOperation fail
        ///               2 thread start failed
        ///          < 0: -errno, 系统错误码的负数值，可通过 strerror() 函数获取错误信息   
        int init(const std::string &port_name = "/dev/claw_serial");

        // std::vector<double> get_positions();

        double getClawPos(ClawType claw_type);
        double getClawSpeed(ClawType claw_type);
        double getClawTorque(ClawType claw_type);
        double getClawTemperature(ClawType claw_type);
        double getClawVoltage(ClawType claw_type);
        double getClawStatus(ClawType claw_type);


        template<typename T>
        void setPositions(const std::vector<T> &target_position) {
            if (isInRange(target_position[0]) && isInRange(target_position[1]))
            {
                setTarClawParameter(target_position, &ClawParameters::pos);
            }
            else
            {
                CLAW_LOG("setTarClawPositions fail, target_position out of range!");
            }
        }
        template<typename T>
        void setSpeeds(const std::vector<T> &target_speed) {
            if (isInRange(target_speed[0]) && isInRange(target_speed[1]))
            {

                setTarClawParameter(target_speed, &ClawParameters::speed);
            }
            else
            {
                CLAW_LOG("setTarClawSpeeds fail, target_speed out of range!");
            }
        }
        template<typename T>
        void setTorques(const std::vector<T> &target_torque) {

            if (isInRange(target_torque[0]) && isInRange(target_torque[1]))
            {
                setTarClawParameter(target_torque, &ClawParameters::tau);
            }
            else
            {
                CLAW_LOG("setTarClawTorques fail, target_torque out of range!");
            }

        }


        std::vector<double> getPositions();
        std::vector<double> getSpeeds();
        std::vector<double> getTorques();
        std::vector<double> getVoltages();
        std::shared_ptr<std::unordered_map<std::string, std::vector<std::any>>> getEndEffectorStatePtr();
        void control_thread();
        void stopControlThread();

        template<typename T>
        inline std::vector<T> getClawParameter(T ClawParameters::*member_ptr) {
            std::lock_guard<std::mutex> lock_w(w_mtx_);
            std::vector<T> claw_parameter;

            for (const auto &claw_data : *cur_claw_data_ptr_list_) {
                claw_parameter.push_back(claw_data.get()->*member_ptr);
            }

            return claw_parameter;
        }

        template<typename T1, typename T2>
        inline void setTarClawParameter(const std::vector<T1> &target_value, T2 ClawParameters::*member_ptr) {
            if (target_value.size() != tar_claw_data_ptr_list_->size()) {
                printf("Please input correct values!\n");
                return;
            }
            std::lock_guard<std::mutex> lock_w(w_mtx_);
            std::lock_guard<std::mutex> lock_r(r_mtx_);
            auto cur_claw_data_ptr = cur_claw_data_ptr_list_;
            
            for (size_t i = 0; i < tar_claw_data_ptr_list_->size(); ++i) {
                T2 cur_value = cur_claw_data_ptr->at(i).get()->*member_ptr;
                // should use a threshold to determine whether the value is need to updated or not
                if (abs(cur_value - target_value[i]) > 10) {
                    try
                    {
                         tar_claw_data_ptr_list_->at(i).get()->*member_ptr = static_cast<T2>(target_value[i]);
                         updated = true;
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                   
                }
            }
        }
        template<typename T>
        inline void setTarClawParameters(const std::vector<T> &target_pos, const std::vector<T> &target_speed, const std::vector<T> &target_tau) {
            if (target_pos.size() != tar_claw_data_ptr_list_->size() || target_speed.size() != tar_claw_data_ptr_list_->size() || target_tau.size() != tar_claw_data_ptr_list_->size()) {
                printf("Please input correct values!\n");
                return;
            }

            std::lock_guard<std::mutex> lock_w(w_mtx_);
            std::lock_guard<std::mutex> lock_r(r_mtx_);
            auto cur_claw_data_ptr = cur_claw_data_ptr_list_;
            

            for (size_t i = 0; i < tar_claw_data_ptr_list_->size(); ++i) {
                double cur_pos = cur_claw_data_ptr->at(i).get()->pos;
                double cur_speed = cur_claw_data_ptr->at(i).get()->speed;
                double cur_tau = cur_claw_data_ptr->at(i).get()->tau;
                auto update_flag = abs(cur_pos - target_pos[i]) > 10 && abs(cur_speed - target_speed[i]) > 10 && abs(cur_tau - target_tau[i]) > 10;
                if (update_flag) {
                    tar_claw_data_ptr_list_->at(i).get()->pos = static_cast<double>(target_pos[i]);
                    tar_claw_data_ptr_list_->at(i).get()->speed = static_cast<double>(target_speed[i]);
                    tar_claw_data_ptr_list_->at(i).get()->tau = static_cast<double>(target_tau[i]);
                    updated = true;
                }
            }
        }
        inline void read_claw_data() {
            std::lock_guard<std::mutex> lock_w(w_mtx_);

            // Check if end_effector_state_ptr_ is valid and get reference to it
            if (!end_effector_state_ptr_) return;
            auto &state_map = *end_effector_state_ptr_;

            // Iterate through claw data list
            for (size_t i = 0; i < cur_claw_data_ptr_list_->size(); ++i) {
                auto &claw_data = cur_claw_data_ptr_list_->at(i);

                // Retrieve claw data
                auto pos = getClawPos(claw_data->claw_type);
                auto speed = getClawSpeed(claw_data->claw_type);
                auto tau = getClawTorque(claw_data->claw_type);
                auto vol = getClawVoltage(claw_data->claw_type);
                auto status = getClawStatus(claw_data->claw_type);

                // Update claw data
                claw_data->pos = pos;
                claw_data->speed = speed;
                claw_data->tau = tau;
                claw_data->vol = vol;
                claw_data->status = status;

                // Update end effector state map
                state_map["positions"][i] = pos;
                state_map["velocities"][i] = speed;
                state_map["torques"][i] = tau;
                state_map["voltages"][i] = vol;
                state_map["status"][i] = status;
            }
        }

        inline void write_claw_data()
        {
            std::lock_guard<std::mutex> lock_w(w_mtx_);
            std::lock_guard<std::mutex> lock_r(r_mtx_);
            if (updated)
            {
                for (auto &tar_claw_data : *tar_claw_data_ptr_list_)
                {
                    runWithParam(tar_claw_data->claw_type, static_cast<int>(tar_claw_data->pos), static_cast<int>(tar_claw_data->speed), static_cast<int>(tar_claw_data->tau));
                }
                updated = false;
            }
        }

    private:
        int claw_id_l_;
        int claw_id_r_;
        int com_num_; // hardware binding
        int baud_rate_;
        std::thread controlThread_;
        std::atomic<bool> stopThread_;
        std::mutex r_mtx_;
        std::mutex w_mtx_;
        bool updated{true};
        ClawDataPtr cur_left_claw_data_ptr_;
        ClawDataPtr cur_right_claw_data_ptr_;
        ClawDataPtr tar_left_claw_data_ptr_;
        ClawDataPtr tar_right_claw_data_ptr_;
        ClawDataListPtr cur_claw_data_ptr_list_;
        ClawDataListPtr tar_claw_data_ptr_list_;
        std::shared_ptr<std::unordered_map<std::string, std::vector<std::any>>> end_effector_state_ptr_;

        void enableClaw(ClawType claw_type, bool enable);
    };
}
