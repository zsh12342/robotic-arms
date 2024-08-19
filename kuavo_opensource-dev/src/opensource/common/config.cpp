#include "config.h"
#include "env_utils.h"

#include <cstdint>
namespace HighlyDynamic
{
    uint16_t nq_f = 7, nv_f = 6;
    uint8_t NUM_ARM_JOINT, NUM_JOINT;

    MotorInfo motor_info;
    
    struct motor_config
    {
        uint32_t encoder_range;
        double max_current;
        double c2t_coeff;
        MotorDriveType driver;
    };

    std::map<std::string, motor_config> g_motor_name_map = {
                {"PA100", {BIT_17_10, PA100_MC, PA100_C2T, EC_MASTER}},
                {"PA81", {BIT_17_10, PA81_MC, PA81_C2T, EC_MASTER}},
                {"PA72", {BIT_17_36, PA72_MC, PA72_C2T, EC_MASTER}},
                {"PA50", {BIT_17_36, PA50_MC, PA50_C2T, EC_MASTER}},
                {"AK10_9", {BIT_17_9, AK10_9_MC, AK10_9_C2T, EC_MASTER}},
                {"CK", {BIT_17_36, CK_MC, CK_C2T, EC_MASTER}},
                {"dynamixel", {BIT_17_36, CK_MC, CK_C2T, DYNAMIXEL}},
                {"realman", {BIT_17_36, CK_MC, CK_C2T, REALMAN}},
                {"ruiwo_elmo", {BIT_17_36, CK_MC, CK_C2T, EC_MASTER}},
                {"ruiwo", {BIT_17_36, CK_MC, CK_C2T, RUIWO}},
                {"PA100_20", {BIT_17_20, PA100_MC, PA100_20_C2T, EC_MASTER}},
                {"PA100_18", {BIT_17_18, PA100_MC, PA100_18_C2T, EC_MASTER}}};

    std::vector<std::string> end_frames_name ;
    std::vector<std::string> contact_frames_name ;
    
    // the value is sizeof array "MOTOR_TYPE" from the config file.
    // Not the NUM_JOINT
    int g_motor_info_config_size = 0;

    JSONConfigReader& JSONConfigReader::getInstance() 
    {
        static JSONConfigReader instance;
        return instance;
    }
    
    void JSONConfigReader::reload()
    {
        load(filename_);
    }
    
    bool JSONConfigReader::init(const std::string &filepath)
    {
        filename_ = filepath;
        try {
            if(!load(filepath)) {
                return false;
            }
            g_motor_info_config_size = get_motor_info_config_size();
            end_frames_name = getValue<std::vector<std::string>>("end_frames_name");
            contact_frames_name = getValue<std::vector<std::string>>("contact_frames_name");

            configHardware();
        }
        catch (const std::exception &e) {
            std::cerr << "[ConfigReader] Read config file fail, path: " << filepath << std::endl;
            std::cerr << "[ConfigReader] Exception: " << e.what() << std::endl;
            return false;
        }
        
        std::cout << "[ConfigReader] CURRENT_SOURCE_DIR: " << env_utils::GetSourceRootPath() << std::endl;
        std::cout << "[ConfigReader] CONFIG_ROOT_PATH: " << env_utils::GetConfigRootPath() << std::endl;
        std::cout << "[ConfigReader] CONFIG_PATH: " << filepath << std::endl;
        std::cout << "[ConfigReader] NUM_JOINT: " << static_cast<int>(NUM_JOINT) << std::endl;
        std::cout << "[ConfigReader] ROBOT_VERSION: " << env_utils::GetRobotVersion() << std::endl;
        std::cout << std::endl;

        return true;
    }

    bool JSONConfigReader::load(const std::string &filepath)
    {
        std::ifstream file(filepath);
        if (file.is_open())
        {
            file >> data;
        }
        else
        {
            std::cerr << "[ConfigReader] Failed to open config file: " << filepath << std::endl;
            return false;
        }

        return true;
    }
    void JSONConfigReader::configHardware()
    {
        NUM_ARM_JOINT = getValue<uint8_t>("NUM_ARM_JOINT");
        NUM_JOINT = getValue<uint8_t>("NUM_JOINT");
        motor_info.resize(NUM_JOINT);

        std::vector<std::string> MOTORS_TYPE = getValue<std::vector<std::string>>("MOTORS_TYPE");
        std::vector<double> min_limits = getValue<std::vector<double>>("min_joint_position_limits");
        std::vector<double> max_limits = getValue<std::vector<double>>("max_joint_position_limits");

        motor_info_config_size_ =  MOTORS_TYPE.size();

        if (!(MOTORS_TYPE.size() == min_limits.size() && MOTORS_TYPE.size() == max_limits.size())) {
            throw std::runtime_error("[CONFIGURATION ERROR]: Please check the `MOTORS_TYPE`, `max_joint_position_limits` and"
                                     " `min_joint_position_limits` fields in the `kuavo.json`, size of them must be equal.");
        }

        for (uint8_t i = 0; i < NUM_JOINT; i++)
        {
            motor_config motor = g_motor_name_map[MOTORS_TYPE[i]];
            motor_info.joint_ids[i] = i + 1;
            motor_info.motor_type[i] = MOTORS_TYPE[i];
            motor_info.driver[i] = motor.driver;
            motor_info.encoder_range[i] = motor.encoder_range;
            motor_info.max_current[i] = motor.max_current;
            motor_info.c2t_coeff[i] = motor.c2t_coeff;
            motor_info.min_joint_position_limits[i] = min_limits[i];
            motor_info.max_joint_position_limits[i] = max_limits[i];
        }

        std::vector<std::string> end_effector_type = RobotConfig.getValue<std::vector<std::string>>("EndEffectorType");
        std::map<std::string, EndEffectorType> end_effector_type_map = {{"none", EndEffectorType::none},
                                                                        {"jodell", EndEffectorType::jodell},
                                                                        {"qiangnao", EndEffectorType::qiangnao}};
        for (auto &name : end_effector_type)
        {
            // std::cout << "EndEffectorType: " << name << std::endl;
            motor_info.end_effector_type.push_back(end_effector_type_map[name]);
        }
    }

    Eigen::VectorXd JSONConfigReader::getEigenVector(const std::string &key)
    {
        if (data.contains(key))
        {
            std::vector<double> std_vector = data[key].get<std::vector<double>>();
            Eigen::Map<Eigen::VectorXd> eigen_vector(std_vector.data(), std_vector.size());
            return eigen_vector;
        }
        else
        {
            std::cerr << "Key not found: " << key << std::endl;
            return {};
        }
    }

    int JSONConfigReader::getMotorCountByType(MotorDriveType type)
    {
        int count = 0;
        std::vector<std::string> motors = getValue<std::vector<std::string>>("MOTORS_TYPE");
        for (uint8_t i = 0; i < motors.size(); i++)
        {   
            if(g_motor_name_map.find(motors[i]) != g_motor_name_map.end()) {
                if (g_motor_name_map[motors[i]].driver == type) {
                    count++;
                }
            }  
        }

        return count;
    }

    int JSONConfigReader::getRuiwoMotorCount()
    {
        return getMotorCountByType(MotorDriveType::RUIWO);
    }

    int JSONConfigReader::getRealmanMotorCount()
    {
        return getMotorCountByType(MotorDriveType::REALMAN);
    }

    int JSONConfigReader::getDynamixelMotorCount()
    {
        return getMotorCountByType(MotorDriveType::DYNAMIXEL);
    }

    int JSONConfigReader::getEcMasterMotorCount()
    {
        return getMotorCountByType(MotorDriveType::EC_MASTER);
    }
    int JSONConfigReader::getJointCount()
    {
        return NUM_JOINT;
    }
    int JSONConfigReader::getArmJointCount()
    {
        return NUM_ARM_JOINT;
    }
    
    int JSONConfigReader::getMotorCount()
    {
        std::vector<std::string> motors = getValue<std::vector<std::string>>("MOTORS_TYPE");
        return motors.size();
    }
}
std::string GetAbsolutePath(const std::string &path)
{
    std::string abs_path = path;
    if (!abs_path.empty() && abs_path.find_first_of("/") != 0)
    {
        // 拼接配置文件相对路径
        std::filesystem::path relative_path = path;

        // 生成配置文件绝对路径
        std::string config_root_path = HighlyDynamic::env_utils::GetSourceRootPath();
        abs_path = (config_root_path + "/" + relative_path.string());
    }

    return abs_path;
}
