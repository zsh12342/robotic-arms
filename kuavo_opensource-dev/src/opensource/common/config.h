#pragma once
#include <iostream>
#include <fstream>
#include <json.hpp>
#include <cmath>
#include "utils.h"
#include "drake/common/eigen_types.h"
#include <vector>
#include "robot_state.h"
#include "RobotErrorTypes.h"
using json = nlohmann::json;

#define MOTOR_CONTROL_MODE_TORQUE 0
#define MOTOR_CONTROL_MODE_VELOCITY 1
#define MOTOR_CONTROL_MODE_POSITION 2
#define BIT_17 (1 << 17)
#define BIT_17_9 (BIT_17 * 9)
#define BIT_17_10 (BIT_17 * 10)
#define BIT_17_18 (BIT_17 * 18)
#define BIT_17_20 (BIT_17 * 20)
#define BIT_17_36 (BIT_17 * 36)

#define AK10_9_MC (40)
#define AK70_10_MC (26.1) // 手册是 23.2
#define PA81_MC (60)
#define PA100_MC (110)
#define PA72_MC (21)
#define PA50_MC (16.8)
#define CK_MC (18)

#define AK10_9_C2T (1.26)
#define AK70_10_C2T (1.23)
#define PA81_C2T (1.25)
#define PA100_C2T (1.2) // 1.2
#define PA72_C2T (3)
#define PA50_C2T (1.8)
#define PA100_18_C2T (2.0)
#define PA100_20_C2T (2.4)
#define CK_C2T (2.1) // 1.4

#define LEG_DOF 6
#define LEGS_TOTEL_JOINT 12

namespace HighlyDynamic
{

    extern uint16_t nq_f, nv_f;
    extern uint8_t NUM_ARM_JOINT, NUM_JOINT;
    constexpr auto Setzero_script = "Setzero.sh";

    enum MotorDriveType
    {
        EC_MASTER,
        DYNAMIXEL,
        REALMAN,
        RUIWO,
    };
    struct MotorInfo
    {
        std::vector<uint8_t> joint_ids;
        std::vector<std::string> motor_type;
        std::vector<MotorDriveType> driver;
        std::vector<uint32_t> encoder_range;
        std::vector<double> max_current;
        std::vector<double> c2t_coeff;
        std::vector<double> min_joint_position_limits;
        std::vector<double> max_joint_position_limits;
        std::vector<EndEffectorType> end_effector_type;
        void resize(uint8_t num_joints)
        {
            joint_ids.resize(num_joints);
            motor_type.resize(num_joints);
            driver.resize(num_joints);
            encoder_range.resize(num_joints);
            max_current.resize(num_joints);
            c2t_coeff.resize(num_joints);
            min_joint_position_limits.resize(num_joints);
            max_joint_position_limits.resize(num_joints);
        }
    };
    extern MotorInfo motor_info;
    class JSONConfigReader
    {
    private:
        json data;
        std::string filename_;
        bool is_loaded{false};
        int  motor_info_config_size_{0};    // the value is sizeof array "MOTOR_TYPE" from the config file.
                                            // Not the NUM_JOINT  
    private:
        JSONConfigReader()=default;
        JSONConfigReader(const JSONConfigReader&)=delete;
        JSONConfigReader& operator=(const JSONConfigReader&)=delete;
        void configHardware();
        bool load(const std::string &filepath);
        
    public:
        static JSONConfigReader& getInstance();
        bool init(const std::string &filepath);
        void reload();
        Eigen::VectorXd getEigenVector(const std::string &key);
        template <typename T>
        T getValue(const std::string &key)
        {
            if (data.contains(key))
            {
                if constexpr (std::is_same<T, Eigen::Matrix<double, -1, 1>>::value)
                    return getEigenVector(key);
                else
                {
                    if constexpr (std::is_same<T, uint8_t>::value)
                    {
                        int value = data[key].get<int>();
                        return static_cast<uint8_t>(value);
                    }
                    return data[key].get<T>();
                }
            }
            else
            {
                std::cerr << "Key not found: " << key << std::endl;
                return T{};
            }
        }
        json::reference operator[](const std::string &key)
        {
            return data[key];
        }

        int get_motor_info_config_size()
        {
            return motor_info_config_size_;
        }
        int getMotorCountByType(MotorDriveType type);
        int getRuiwoMotorCount();
        int getRealmanMotorCount();
        int getDynamixelMotorCount();
        int getEcMasterMotorCount();
        int getJointCount();
        int getArmJointCount();
        int getMotorCount();
    };
    #define RobotConfig JSONConfigReader::getInstance()
    extern std::vector<std::string> end_frames_name;
    extern std::vector<std::string> contact_frames_name;

}
