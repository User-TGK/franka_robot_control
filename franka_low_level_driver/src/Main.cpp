#include <Driver.hpp>

int main(int argc, char** argv)
{
  const std::string nodeName = "FrankaDriver";
  ros::init(argc, argv, nodeName);

  if(argc != 2)
  {
    ROS_ERROR("Usage: %s <robot_hostname>.", argv[0]);
    return -1;
  }
  try
  {
    RobotControl::FrankaLowLevelDriver::Driver driver(nodeName, argv[1]);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }
  catch(const franka::Exception& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }
  return 0;
}