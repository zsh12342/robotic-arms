#include <StandRobot.h>

int main(int argc, char *argv[])
{
  // sched_process(0);
  HighlyDynamic::StandRobot robot;
  robot.doMain(argc, argv);
  return 0;
}
