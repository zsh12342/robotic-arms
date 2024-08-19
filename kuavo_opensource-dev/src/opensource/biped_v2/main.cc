#include <HighlyDynamicRobot.h>
#include "env_utils.h"

int main(int argc, char *argv[])
{ 
    using namespace HighlyDynamic;
    
    // Check Environment
    env_utils::Environment robot_env;
    if(!robot_env.Init()) {
      return -1;
    }

    // Load Config.
    if(!JSONConfigReader::getInstance().init(env_utils::GetConfigFilePath())) {
      return -1;
    }

    HighlyDynamic::HighlyDynamicRobot robot;
    robot.doMain(argc, argv);

    return 0;
}
