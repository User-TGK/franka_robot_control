#include <Driver.hpp>

int main(int argc, char** argv)
{
  const std::string nodeName = "AL5DDriver";
  ros::init(argc, argv, nodeName);

  if(argc != 2)
  {
    ROS_ERROR("USAGE: rosrun al5d_low_level_driver <path to serialport the SSC32U is connected to>");
    return -1;
  }

  std::vector<RobotControl::Al5dLowLevelDriver::DegreeOfFreedom> degreeOfFreedoms;

  /** Starting DoF positions are unknown to the SSC32U. All currentPositions will be set to zero */
  degreeOfFreedoms.push_back(
      RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(0, 500, 2500, -90, 90, 272.73, 0, 0));// AL5D's - Base servo
  degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(
      1, 500, 2500, 90, -90, 315.79, 0, 0));// AL5D's - Shoulder (Turret) servo
  degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(
      2, 500, 2500, 0, 180, 214.29, 0, 0));// AL5D's - Elbow (Upperarm) servo
  degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(
      3, 500, 2500, 90, -90, 250.00, 0, 0));// AL5D's - Wirst (Forearm) servo
  degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(
      4, 500, 2500, -90, 90, 285.71, 0, 0));// AL5D's - Wrist rotate servo
  degreeOfFreedoms.push_back(
      RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(5, 500, 2500, -90, 90, 285.71, 0, 0));// AL5D's - Gripper

  try
  {
    RobotControl::Al5dLowLevelDriver::Driver driver(nodeName, degreeOfFreedoms, argv[1]);
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService(
        "al5d_emergency_stop", &RobotControl::Al5dLowLevelDriver::Driver::executeEmergencyStop, &driver);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }

  catch(boost::system::system_error& e)
  {
    std::cerr << "SERIAL PORT NOT OPENED PROPERLY: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}