#include <HighLevelInterface.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HighLevelInterface");

  if(argc != 2)
  {
    ROS_ERROR("Usage: %s <robot_type>. The supported robot types are AL5D and FRANKA.", argv[0]);
    return -1;
  }
  RobotControl::HighLevelInterface::RobotType robotType;

  if(strcmp(argv[1], "AL5D") == 0)
  {
    robotType = RobotControl::HighLevelInterface::RobotType::Al5d;
  }
  else if(strcmp(argv[1], "FRANKA") == 0)
  {
    robotType = RobotControl::HighLevelInterface::RobotType::Franka;
  }
  else if(strcmp(argv[1], "FRANKA_SIMULATION") == 0)
  {
    robotType = RobotControl::HighLevelInterface::RobotType::FrankaSimulation;
  }
  else if(strcmp(argv[1], "FRANKA_COMBINED") == 0)
  {
    robotType = RobotControl::HighLevelInterface::RobotType::FrankaCombined;
  }
  else
  {
    ROS_ERROR("%s is not a supported robot type. The supported robot types are AL5D, FRANKA, FRANKA_SIMULATION & FRANKA_COMBINED.", argv[1]);
    return -1;
  }
  try
  {
    RobotControl::HighLevelInterface::HighLevelInterface robotInterface(
        "high_level_interface", robotType, "emergency_stop");

    ros::spin();
    ros::waitForShutdown();
  }
  catch(const std::runtime_error& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }
  return 0;
}