#include "env_utils.h"
#include "../../common/utils.h"

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <pwd.h>

#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <map>

namespace HighlyDynamic
{
#if ROBOT_VERSION_INT >= 41
    const float  kRobotVersionFloat = 4.1;    
#elif ROBOT_VERSION_INT >= 40
    const float  kRobotVersionFloat = 4.0;
#elif ROBOT_VERSION_INT >= 34
    const float  kRobotVersionFloat = 3.4;
#elif ROBOT_VERSION_INT >= 33
    const float  kRobotVersionFloat = 3.3;
#elif ROBOT_VERSION_INT >= 32
    const float  kRobotVersionFloat = 3.2;
#elif ROBOT_VERSION_INT >= 30
    const float  kRobotVersionFloat = 3;
#elif ROBOT_VERSION_INT >= 23
    const float  kRobotVersionFloat = 2.3;
#elif ROBOT_VERSION_INT >= 20
    const float  kRobotVersionFloat = 2;
#elif ROBOT_VERSION_INT >= 11
    const float  kRobotVersionFloat = 1.1;
#elif ROBOT_VERSION_INT >= 10
    const float  kRobotVersionFloat = 1.0;
#else
#error "Invalid version selected"
#endif
}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

namespace HighlyDynamic {
namespace env_utils {

// 配置根目录格式 => $HOME/.config/lejuconfig
constexpr char kConfigRootPathFormat[] = "%s/.config/lejuconfig";

// 配置文件路径后缀格式 => config/kuavo_v4.0/kuavo.json
constexpr char kConfigFilePathSuffixFormat[] = "config/kuavo_v%0.1f/kuavo.json";

const char* kRepoConfigSuffixFormats[] {
    "src/biped_v2/config/kuavo_v%0.1f",     // => src/biped_v2/config/kuavo_v4.0
    "src/biped_v2/config/kuavo_v%0.0f",     // => src/biped_v2/config/kuavo_v3
    "src/biped_v2/config/kuavo_mt-%0.1f",   // => src/biped_v2/config/kuavo_mt-1.0
};

const char* kModelSuffixFormats[] {
        // Special for => models/biped_gen4.0 ...
        "models/biped_gen%0.1f",

        // Special for => models/biped_gen3 ...
        "models/biped_gen%0.0f",

        // Special for => models/biped_mt-1.0 ...
        "models/biped_mt-%0.1f",

        // And More...
        // "models/biped_gen%0.1f/urdf/",
};

// 默认的配置文件路径
constexpr char kDefaultConfigRootPath[] = "/home/lab/.config/lejuconfig";
// 默认的配置文件路径
static std::string  g_config_root_path;
static std::string  g_config_filepath;

float GetRobotVersion()
{
    return kRobotVersionFloat;
}

std::string GetConfigFilePath()
{
    if (g_config_filepath.empty()) {
        // use default config
        char suffix_path[PATH_MAX]{};
        snprintf(suffix_path, PATH_MAX, kConfigFilePathSuffixFormat, kRobotVersionFloat);
        std::string config_root = GetConfigRootPath();
        g_config_filepath = config_root.append("/").append(suffix_path);
    }
    
    return g_config_filepath;
}

std::string GetRepoConfigFilePath()
{
    std::string repo_config_path = "";
    for(int i = 0; i < ARRAY_SIZE(kRepoConfigSuffixFormats); i++) {
        char suffix_path[PATH_MAX]{};
        snprintf(suffix_path, PATH_MAX, kRepoConfigSuffixFormats[i], kRobotVersionFloat);
        repo_config_path = GetSourceRootPath().append("/").append(suffix_path).append("/kuavo.json");

        if (std::filesystem::exists(repo_config_path)) {
            break;
        }
    }
    
    return repo_config_path;

}

std::string GetConfigFileParentPath()
{
    std::filesystem::path config_filepath(GetConfigFilePath());
    return config_filepath.parent_path().string();
}

std::string GetRepoConfigFileParentPath()
{
    std::filesystem::path repo_config_path(GetRepoConfigFilePath());
    return repo_config_path.parent_path().string();
}

std::string GetSourceRootPath()
{
    // src/opensource/common/env_utils.cc
    static std::filesystem::path env_utils_cc_filepath = __FILE__;
    try {
        std::filesystem::path curr_source_path = env_utils_cc_filepath.parent_path().parent_path().parent_path().parent_path();
        return curr_source_path.string();
    } catch(std::exception& e) {
        std::cout << "GetSourceRootPath exception: " << e.what() << "\n";
    }

   return "";
}

std::string GetConfigRootPath()
{
    return g_config_root_path;
}

/*********************************** Class Environment ************************************/

static void _print_kuavo_ascii_text() {
    
    static std::string g_kuavo_ascii_text = R"(
     __  ___  __    __       ___   ____    ____  ______
    |  |/  / |  |  |  |     /   \  \   \  /   / /  __  \
    |  '  /  |  |  |  |    /  ^  \  \   \/   / |  |  |  |
    |    <   |  |  |  |   /  /_\  \  \      /  |  |  |  |
    |  .  \  |  `--'  |  /  _____  \  \    /   |  `--'  |
    |__|\__\  \______/  /__/     \__\  \__/     \______/
    )";
    
    printf("\033[33m%s\033[0m\n", g_kuavo_ascii_text.c_str());
}

static void _show_error_tips() {
    static std::string g_error_tips_text = 
R"([Environment] 环境初始化失败, 请检查配置路径是否存在:
1. 配置目录:%s,
2. 配置文件:%s,
3. mesh 文件:%s,
4. urdf 文件:%s,
可从仓库拷贝 ~/src/biped_v2/config/kuavo_v* 到 $HOME/.config/lejuconfig/config/,
可从仓库拷贝 ~/models/biped_gen*/ 到 $HOME/.config/lejuconfig/models/.
)";

    std::string config_root_path = GetConfigRootPath();
    std::string config_filepath = GetConfigFilePath();
    std::string mesh_filepath = config_root_path.append("/models/biped_gen*/meshes");
    std::string urdf_filepath = config_root_path.append("/models/biped_gen*/urdf");

    printf("\033[31m");
    printf(g_error_tips_text.c_str(), 
            config_root_path.c_str(), config_filepath.c_str(),
            mesh_filepath.c_str(), urdf_filepath.c_str());
    printf("\033[0m");
}


static bool CreateDirectoriesWithPermissions(const std::filesystem::path &dir_path) {
    try {
        if(std::filesystem::create_directories(dir_path)) {
            auto perms_rwx_xr_r = 
                std::filesystem::perms::owner_all  | 
                std::filesystem::perms::group_exec | std::filesystem::perms::group_read |
                std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加目录的可执行和读权限
            std::filesystem::permissions(dir_path, perms_rwx_xr_r, std::filesystem::perm_options::add);
            
            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[Environment] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }

    return false;   
}

static bool CopyFileWithPermissions(const std::filesystem::path &src_path, 
                        const std::filesystem::path &dst_path, 
                        std::filesystem::copy_options opt = std::filesystem::copy_options::skip_existing) 
{
    try {
        if(std::filesystem::copy_file(src_path, dst_path, opt)) {
            auto perms_rw_rw_rw = 
                std::filesystem::perms::owner_write | std::filesystem::perms::owner_read |
                std::filesystem::perms::group_write | std::filesystem::perms::group_read |
                std::filesystem::perms::others_write | std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加读写权限
            std::filesystem::permissions(dst_path, perms_rw_rw_rw, std::filesystem::perm_options::add);

            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[Environment] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }
    return false;  
}

bool Environment::Init() 
{
    _print_kuavo_ascii_text();

    do {
        // 配置根目录
        if(!CheckConfigRootPath()) break;

        // 检查配置文件
        if(!CheckConfigFilePath()) break;
        
        // 检查 models 文件
        if(!CheckModelFilePath()) break;

        // 检查 Revo 配置文件
        if(!CheckRevoConfigFilePath()) break;
        
        // Sync Config
        std::cout << "[Environment] Init, Sync Config File. \n";
        SyncConfig();
        return true;
    }while (false);
    
    // Last Try: 使用默认的 /home/lab/.config/lejuconfig/
    {
        std::cout << "[Environment] Init, Use Default Config from:/home/lab/.config/lejuconfig/ \n";
        g_config_root_path = kDefaultConfigRootPath;
        char suffix_path[PATH_MAX]{};
        snprintf(suffix_path, PATH_MAX, kConfigFilePathSuffixFormat, kRobotVersionFloat);
        g_config_filepath = std::string(kDefaultConfigRootPath).append("/").append(suffix_path);

        std::map<std::string, std::string> check_items {
            { "CONFIG PATH", g_config_root_path},
            {"CONFIG FILE", g_config_filepath},
            {"RUIWO CONFIG FILE", "/home/lab/.config/lejuconfig/config.yaml"}
        };

        for(auto &[key, value] : check_items) {
            if(!std::filesystem::exists(value)) {
                std::cout << "[Environment] Init, Use Default" << key << " is not exist, path:" << value << "\n";
                _show_error_tips();
                return  false;
            }
            else {
                if(std::filesystem::is_directory(value)) {
                    bool folder_empty = std::distance(std::filesystem::directory_iterator(value), 
                                                      std::filesystem::directory_iterator()) == 0;
                    if(folder_empty) {
                        std::cout << "[Environment] Init, Use Default " 
                            << key << " is empty, path:" << value << "\n";
                        _show_error_tips();
                        return  false;  
                    }        
                } // end of check folder empty. 
            }
        } // end of for.
    }

    return true;
}

void Environment::SyncConfig() {
    auto config_filepath = GetConfigFilePath();
    auto repo_config_filepath = GetRepoConfigFilePath();

    // 读取配置文件
    json config_data;
    json repo_config_data;
    try {
        config_data = ReadJsonFile(config_filepath);
    } catch (const std::exception& e) {
        std::cerr << "[Environment] Error reading config file: " << e.what() << "\n";
        return;
    }

    try {
        repo_config_data = ReadJsonFile(repo_config_filepath);
    } catch (const std::exception& e) {
        std::cerr << "[Environment] Error reading repo config file: " << e.what() << "\n";
        return;
    }

    // 比较配置文件并更新
    for (auto& [key, value] : repo_config_data.items()) {
        if (!config_data.contains(key)) {
            config_data[key] = value;
            std::cout << "[Environment] SyncConfig, Add new key: [" << key << "] from repo config file.\n";
        }
    }

    // 保存配置文件
    try {
        if (WriteJsonFile(config_filepath, config_data)) {
            std::cout << "[Environment] SyncConfig, Sync config file success.\n";
        }
        else {
            std::cerr << "[Environment] SyncConfig, Sync config file fail.\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "[Environment] Error writing config file: " << e.what() << "\n";
        return;
    }

}
bool Environment::CheckConfigRootPath()
{   
    do {
        // FIXME: 当用户为 ROOT/SUDO 时，把配置文件放到 /.config/lejuconfig 目录
        std::string home = getKuavoHomePath();

        char buf[PATH_MAX]{};
        int len = snprintf(buf, PATH_MAX, kConfigRootPathFormat, home.c_str());
        if (len > 0 && len < PATH_MAX) {
            g_config_root_path = buf;
        }
        else {
            // 使用默认的
            g_config_root_path = kDefaultConfigRootPath;
        }
    }while(false);

    // 创建配置目录
    std::filesystem::path dest_dir = g_config_root_path;
    try {
        if (!std::filesystem::exists(dest_dir) && 
            !CreateDirectoriesWithPermissions(dest_dir)) {
            std::cout << "[Environment] CreateConfigRootPath fail, path:" << g_config_root_path << "\n";
            return false;
        }
    }
    catch (const std::filesystem::filesystem_error& ex) {
        std::cout << "[Environment] CreateConfigRootPath, exception:" << ex.what() << "\n";
        return false;
    }

    return true;
}

bool is_directory_empty(const std::filesystem::path& dir_path) {
    if (!std::filesystem::exists(dir_path)) {
        return true;
    }

    if(!std::filesystem::is_directory(dir_path)) {
        return false;
    }

    return std::distance(std::filesystem::directory_iterator(dir_path), std::filesystem::directory_iterator()) == 0;
}

bool copy_directory(const std::filesystem::path& src_dir_path, 
    const std::filesystem::path& dest_dir_path, 
    int depth = 0) {
    static const int max_depth = 10;

    // Check the recursion depth
    if (depth > max_depth) {
        std::cerr << "Error: Maximum recursion depth exceeded.\n";
        return false;
    }

    try {
        // Create destination directory if it doesn't exist
        if (!std::filesystem::exists(dest_dir_path)) {
            CreateDirectoriesWithPermissions(dest_dir_path);
        }

        // Iterate over all entries in the source directory
        for (const auto& entry : std::filesystem::directory_iterator(src_dir_path)) {
            // Get the path of the current entry
            std::filesystem::path src_path = entry.path();
            // Build the destination path
            std::filesystem::path dest_path = dest_dir_path / src_path.filename();

            // If it's a directory, recursively call copy_directory
            if (std::filesystem::is_directory(src_path)) {
                if (!copy_directory(src_path, dest_path, depth + 1)) {
                    return false; // Return false if sub-directory copy fails
                }
            } else { // Otherwise, it's a file, so copy it directly
                CopyFileWithPermissions(src_path, dest_path);
            }
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[Environment] copy_directory exception:" << ex.what() << '\n';
        return false;
    }

    return true;
}

bool Environment::CheckConfigFilePath()
{
    // $HOME/.config/lejuconfig/config/kuavo_v4.0/kuavo.json
    std::string config_filepath = GetConfigFilePath();
    if (std::filesystem::exists(config_filepath)) {
        return true;
    }
    
    // From repo `src/biped_v2/config/kuavo_v4.0/`
    std::string src_dir_path;
    for(int i = 0; i < ARRAY_SIZE(kRepoConfigSuffixFormats); i++) {
		char suffix_path[PATH_MAX]{};
        snprintf(suffix_path, PATH_MAX, kRepoConfigSuffixFormats[i], kRobotVersionFloat);
        src_dir_path = GetSourceRootPath().append("/").append(suffix_path);

        if (std::filesystem::exists(src_dir_path)) {
            break;
        }
	}

    // Repo 文件不存在，无法拷贝 --> Fail!
    if (!std::filesystem::exists(src_dir_path)) {
        auto repo_config_path = std::filesystem::path(src_dir_path).parent_path();
        std::cout << "[Environment] CheckConfigFilePath, Can't copy from repo:" 
                  << repo_config_path.string() << "\n";
        return false;
    }

    // $HOME/.config/lejuconfig/config/kuavo_v4.0/
    std::filesystem::path parent_path = std::filesystem::path(config_filepath).parent_path();
    if (!std::filesystem::exists(parent_path)) {
        // Not exist, copy from repo.
        std::cout << "[Environment] CheckConfigFilePath, copy from:" << src_dir_path << "\n";
        return copy_directory(src_dir_path, parent_path);
    }

    // Only Copy `kuavo.json` from repo `src/biped_v2/config/kuavo_v4.0/kuavo.json`
    std::filesystem::path src_path = src_dir_path / std::filesystem::path("kuavo.json");
    std::cout << "[Environment] CheckConfigFilePath, copy from:" << src_path.string() << "\n";
    return CopyFileWithPermissions(src_path, config_filepath);
}

bool Environment::CheckModelFilePath()
{
    // Get Config Root Path
    std::filesystem::path repo_root_path(GetSourceRootPath());
    std::filesystem::path config_root_path(GetConfigRootPath());

    auto CopyFileLamba = [&](const char *format) -> bool {
        char suffix[PATH_MAX]{};
        snprintf(suffix, PATH_MAX, format, kRobotVersionFloat);
        
        std::filesystem::path dest_path(config_root_path / suffix);
        std::filesystem::path src_path(repo_root_path / suffix);

        // model 文件夹不为空，不拷贝
        if(!is_directory_empty(dest_path)) {
            return true;
        }

        // 仓库没有这个目录，不拷贝
        if(is_directory_empty(src_path)) {
            return false;
        }

        std::cout << "[Environment] CheckModelFilePath, copy from:" << src_path.string() << "\n";
        return copy_directory(src_path, dest_path);
    };

    for(int i = 0; i < ARRAY_SIZE(kModelSuffixFormats); i++) {
        if(CopyFileLamba(kModelSuffixFormats[i])) return true;
    }

    std::cout << "[Environment] CheckModelFilePath, copy model from repo fail. \n";
    return false;
}

bool Environment::CheckRevoConfigFilePath()
{
    std::string revo_filepath = GetConfigRootPath().append("/config.yaml");
    if (std::filesystem::exists(revo_filepath)) {
        return true;
    }

    std::string src_path = GetSourceRootPath().append("/lib/ruiwo_controller/config.yaml");    
    std::cout << "[Environment] CheckRevoConfigFilePath, copy from:" << src_path << "\n";
    return CopyFileWithPermissions(src_path, revo_filepath);
}

} // namespace env_utils
} // namespace HighlyDynamic