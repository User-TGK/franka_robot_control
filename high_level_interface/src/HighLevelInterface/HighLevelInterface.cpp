#include <HighLevelInterface.hpp>
#include <RobotInstanceAl5d.hpp>
#include <RobotInstanceFranka.hpp>
#include <RobotInstanceFrankaSimulation.hpp>

namespace RobotControl
{
namespace HighLevelInterface
{
HighLevelInterface::HighLevelInterface(const std::string& configTopicName,
                                       RobotType robotType,
                                       const std::string& emergencyStopName)
    : emergencyStopHandler(nh.advertiseService(emergencyStopName, &HighLevelInterface::handleEmergencyStopCB, this)),
      preprogrammedPositionHandler(
          nh.advertiseService("robot_position", &HighLevelInterface::preProgrammedPositionCB, this)),
      unlockEmergencyStopHandler(nh.advertiseService("unlock_robot_driver", &HighLevelInterface::handleUnlockCB, this)),
      gripperHandler(nh.advertiseService("gripper", &HighLevelInterface::handleGripper, this)),
      updateManualHandler(nh.advertiseService("update_manual", &HighLevelInterface::updateManual, this))
{
  setRobot(configTopicName, robotType);
}

HighLevelInterface::~HighLevelInterface()
{
}

bool HighLevelInterface::updateManual(high_level_interface::UpdateManualRequest& request,
                        high_level_interface::UpdateManualResponse& response)
{
  for (const auto& robotInstance: robotInstances)
  {
    if (robotInstance->getType() == RobotType::Franka)
    {
      response.success = true;
      response.newManualConfiguration = robotInstance->updateManualPosition(request.update);
    }
  }
  return true;
}

bool HighLevelInterface::preProgrammedPositionCB(high_level_interface::PositionRequest& request,
                                                 high_level_interface::PositionResponse& response)
{
  high_level_interface::UnlockRequest r;
  r.unlock = true;
  high_level_interface::UnlockResponse u;
  handleUnlockCB(r, u);
  for (const auto& robotInstance: robotInstances)
  {
    if(robotInstance->getCurrentState() == RobotState::Locked)
    {
      ROS_ERROR("No configuration goals can be executed while the robot interface is in the Locked state.");
      response.success = false;
      return true;
    }
    robotInstance->toPreprogrammedPosition(request, response);
  }
  return true;
}

bool HighLevelInterface::handleUnlockCB(high_level_interface::UnlockRequest& request,
                                        high_level_interface::UnlockResponse& response)
{
  if(request.unlock)
  {
    for (const auto& robotInstance: robotInstances)
    {
      ros::ServiceClient frankaUnlockClient;
      frankaUnlockMsg.request.unlock = true;

      if(robotInstance->getType() == RobotType::Franka)
      {
        frankaUnlockClient = nh.serviceClient<franka_low_level_driver::UnlockFranka>("unlock_franka");
        frankaUnlockClient.call(frankaUnlockMsg);
        response.success = frankaUnlockMsg.response.success;
      }
      else if (robotInstance->getType() == RobotType::FrankaSimulation)
      {
        frankaUnlockClient = nh.serviceClient<franka_low_level_driver::UnlockFranka>("unlock_franka_simulation");
        frankaUnlockClient.call(frankaUnlockMsg);
        response.success = frankaUnlockMsg.response.success;
      }
      robotInstance->setCurrentState(RobotState::Active);
    }
  }
  else
  {
    response.success = false;
  }
  return true;
}

bool HighLevelInterface::handleEmergencyStopCB(high_level_interface::EmergencyStopRequest& request,
                                               high_level_interface::EmergencyStopResponse& response)
{
  unsigned short nrOfSuccess = 0;
  for (const auto& robotInstance: robotInstances)
  {
    robotInstance->setCurrentState(RobotState::Stopping);
    if(robotInstance->handleEmergencyStopRequest(request, response))
    {
      if (request.stop == true)
      {
        robotInstance->setCurrentState(RobotState::Locked);
      }
      nrOfSuccess ++;
    }
  }
  if (nrOfSuccess == robotInstances.size())
  {
    return true;
  }
  for (auto& robotInstance: robotInstances)
  {
    robotInstance->setCurrentState(RobotState::Active);
  }
  return false;
}

bool HighLevelInterface::handleGripper(high_level_interface::GripperRequest& request,
                                high_level_interface::GripperResponse& response)
{
  unsigned short nrOfSuccess = 0;
  for (const auto& robotInstance: robotInstances)
  {
    if(robotInstance->getCurrentState() == RobotState::Locked)
    {
      if (request.open)
      {
        robotInstance->controlGripper(request, response);
        nrOfSuccess ++;
      }
      else
      {
        ROS_ERROR("The gripper movement can't be executed while the robot interface is in the Locked state.");
        response.success = false;
      }
    }
    nrOfSuccess += robotInstance->controlGripper(request, response);
  }
  if (nrOfSuccess == robotInstances.size())
  {
    response.success = true;
    return true;
  }
  return false;
}
 
void HighLevelInterface::setRobot(const std::string& configTopicName, RobotType robotType)
{
  switch(robotType)
  {
    case RobotType::Al5d:
    {
      robotInstances.push_back(std::make_shared<RobotInstanceAl5d>(configTopicName));
      break;
    }
    case RobotType::Franka:
    {
      robotInstances.push_back(std::make_shared<RobotInstanceFranka>(configTopicName));
      break;
    }
    case RobotType::FrankaSimulation:
    {
      robotInstances.push_back(std::make_shared<RobotInstanceFrankaSimulation>(configTopicName));
      break;
    }
    case RobotType::FrankaCombined:
    {
      robotInstances.push_back(std::make_shared<RobotInstanceFranka>(configTopicName));
      robotInstances.push_back(std::make_shared<RobotInstanceFrankaSimulation>(configTopicName));
      break;
    }
  }
}

}// namespace RobotControl
}// namespace HighLevelInterface