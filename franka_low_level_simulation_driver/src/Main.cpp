#include <Driver.hpp>

int main(int argc, char** argv)
{
  const std::string nodeName = "FrankaSimulationDriver";
  ros::init(argc, argv, nodeName);

  if(argc != 2)
  {
    ROS_ERROR("Usage: %s <robot_hostname>.", argv[0]);
    return -1;
  }

  std::vector<RobotControl::FrankaLowLevelDriver::DegreeOfFreedom> degreeOfFreedoms;

  /** Starting DoF positions are unknown to the SSC32U. All currentPositions will be set to zero */
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      0, 500, 2500, -170, 170, 272.73, 0, 0));// franka's - Base servo Franka_panda_joint1
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      1, 500, 2500, -105, 105, 315.79, 0, 0));// franka's - Shoulder (Turret) servo Franka_panda_joint2
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      2, 500, 2500, -170, 170, 214.29, 0, 0));// franka's - Elbow (Upperarm) servo Franka_panda_joint3
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      3, 500, 2500, -180, 5, 250.00, 0, 0));// franka's - Wirst (Forearm) servo Franka_panda_joint4
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      4, 500, 2500, -170, 170, 285.71, 0, 0));// franka's - Wrist rotate servo Franka_panda_joint5
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      5, 500, 2500, -5, 219, 285.71, 0, 0));// franka's - Gripper Franka_panda_joint6
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      6, 500, 2500, -170, 170, 285.71, 0, 0));// franka's - Gripper Franka_panda_joint7
  degreeOfFreedoms.push_back(RobotControl::FrankaLowLevelDriver::DegreeOfFreedom(
      7, 2000, 0, 0, 2.26, 285.71, 0, 0));// Franka's - Gripper Franka_panda_jointH1 && Franka_panda_jointH2

  try
  {
    RobotControl::FrankaLowLevelDriver::Driver driver(nodeName, degreeOfFreedoms, argv[1]);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }

  catch(boost::system::system_error& e)
  {
    std::cerr << "SERIAL PORT NOT OPENED PROPERLY: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}