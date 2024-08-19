#ifndef KUAVO_ENV_UTILS_H_
#define KUAVO_ENV_UTILS_H_
#include <stdint.h>
#include <string>
#include <fstream>
#include <unistd.h> 
#include <fcntl.h>
#include <cstdio>
#include <json.hpp>

using json = nlohmann::json;

namespace HighlyDynamic {
namespace env_utils {

/// @brief 获取版本号， 
/// @return 比如 4.0
float GetRobotVersion();

/// @brief 获取当前代码仓库的根目录路径
/// @note  注意该值在编译时已经确定，且运行时是不会改变
std::string GetSourceRootPath();

/// @brief 获取配置文件路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return 比如 $HOME/.config/lejuconfig/config/kuavo_v4.0/kuavo.json
std::string GetConfigFilePath();

/// @brief 获取仓库配置文件路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
std::string GetRepoConfigFilePath();

/// @brief 获取仓库配置文件目录路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
std::string GetRepoConfigFileParentPath();

/// @brief 获取配置文件目录路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return 比如 $HOME/.config/lejuconfig/config/kuavo_v4.0
std::string GetConfigFileParentPath();

/// @brief 获取配置文件根目录路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return $HOME/.config/lejuconfig
std::string GetConfigRootPath();

class Environment {
public:
   Environment()=default;
   bool Init();

private:
   void SyncConfig();  
   bool CheckConfigRootPath();
   bool CheckConfigFilePath();
   bool CheckModelFilePath();
   bool CheckRevoConfigFilePath();
   json ReadJsonFile(const std::string& filepath) const {
      std::ifstream ifs(filepath);
      if (!ifs.is_open()) {
         throw std::runtime_error("Unable to open file: " + filepath);
      }
      json data;
      ifs >> data;
      return data;
   }

   bool WriteJsonFile(const std::string& filepath, const json& data) const {
      std::ofstream ofs(filepath, std::ios::out | std::ios::trunc);
      if (!ofs.is_open()) {
         throw std::runtime_error("Unable to open file: " + filepath);
      }
      ofs << data.dump(4); // pretty print with 4 spaces indent

      // Ensure the data is written to disk
      ofs.flush(); // Flush the buffer to the OS
      ofs.close(); // Close the file

      int fd = open(filepath.c_str(), O_WRONLY);
      if (fd == -1) {
         perror("open");
         return false;
      }

      if (fsync(fd) == -1) {
         perror("fsync");
         close(fd);
         return false;
      }

      if (close(fd) == -1) {
         perror("close");
         return false;
      }

      return true;
   }
};

} // namespace env_utils
} // namespace HighlyDynamic

#endif