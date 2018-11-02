#include <RobotInstanceFrankaSimulation.hpp>

namespace RobotControl
{
namespace HighLevelInterface
{
RobotInstanceFrankaSimulation::RobotInstanceFrankaSimulation(const std::string& name)
    : RobotInstance(name, RobotType::Franka), configurationClient("FrankaSimulationDriver", true),
    gripperClient(nh.serviceClient<franka_low_level_driver::GripperFranka>("gripper_franka_simulation"))
{
   if (!parsePreprogrammedPositions())
  {
    ROS_ERROR("Failed parsing the preprogrammed positions for the franka simulation.");
    return;
  }
  emergencyStopClient = nh.serviceClient<franka_low_level_driver::EmergencyStopFranka>("franka_simulation_emergency_stop");
  as.registerGoalCallback(boost::bind(&RobotInstanceFrankaSimulation::executeConfigurationAction, this));
  as.start();
}

RobotInstanceFrankaSimulation::~RobotInstanceFrankaSimulation()
{
}

void RobotInstanceFrankaSimulation::copyDoFConfiguration(high_level_interface::DoFConfiguration& goal,
                                               const franka_low_level_driver::DoFConfiguration& origin)
{
  goal.channel = origin.channel;
  goal.angle   = origin.angle;
  goal.speed   = origin.speed;
}

void RobotInstanceFrankaSimulation::copyDoFConfiguration(franka_low_level_driver::DoFConfiguration& goal,
                                               const high_level_interface::DoFConfiguration& origin)
{
  goal.channel = origin.channel;
  goal.angle   = origin.angle;
  goal.speed   = origin.speed;
}

bool RobotInstanceFrankaSimulation::parsePreprogrammedPositions()
{
  std::string packagePath = ros::package::getPath("high_level_interface");
  packagePath += "/config/PreProgrammedFranka.ini";
  std::string positionConfig;
  std::ifstream configFile(packagePath);

  if (configFile.is_open())
  {
    while (getline(configFile, positionConfig))
    {
      std::vector<franka_low_level_driver::DoFConfiguration> doFConfiguration;
      doFConfiguration.reserve(7);

      std::vector<std::string> tokens;
      tokens.reserve(8);

      boost::split(tokens, positionConfig, [](char c) { return c == ' '; });

      if (tokens.size() != 8)
      {
          return false;
      }

      auto position = preProgrammedPositionFromString(tokens.at(0));
      for (unsigned short channel = 1; channel < 8; ++channel)
      {
        try
        {
          franka_low_level_driver::DoFConfiguration singleConfig;
          singleConfig.channel = channel - 1;
          singleConfig.angle = std::stof(tokens.at(channel));
          doFConfiguration.push_back(singleConfig);
        }
        catch (const std::invalid_argument& arg)
        {
          return false;
        }
      }
      preProgrammedConfigurations.insert(std::pair<PreProgrammedPosition, std::vector<franka_low_level_driver::DoFConfiguration>>(position, doFConfiguration));
    }
  }
  return true;
}

bool RobotInstanceFrankaSimulation::toPreprogrammedPosition(high_level_interface::PositionRequest& request,
                                                  high_level_interface::PositionResponse& response)
{
  response.success = false;
  franka_low_level_driver::FrankaConfigurationGoal goal;
  franka_low_level_driver::DoFConfiguration newConfig;
  goal.speedFactor = 0.3;

  try
  {
    auto preProgrammedPosition = preProgrammedPositionFromString(request.position);
    goal.configuration = preProgrammedConfigurations.find(preProgrammedPosition)->second;
    configurationClient.sendGoal(goal);
    response.success = true;
    return true;
  }
  catch(const std::invalid_argument& arg)
  {
    ROS_ERROR("%s", arg.what());
  }
  return false;
}

std::vector<high_level_interface::DoFConfiguration> RobotInstanceFrankaSimulation::updateManualPosition(bool update)
{
  emergencyStopMsg.request.stop = false;
  emergencyStopClient.call(emergencyStopMsg);

  auto find = preProgrammedConfigurations.find(PreProgrammedPosition::Manual);
  if (update)
  {
    find->second = emergencyStopMsg.response.resultConfiguration;
  }
  
  std::vector<high_level_interface::DoFConfiguration> newManualConfiguration;
  for(const auto& doFConfiguration : find->second)
  {
    high_level_interface::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    newManualConfiguration.push_back(newConfig);
  }
  return newManualConfiguration;
}

void RobotInstanceFrankaSimulation::executeConfigurationAction()
{
  auto goal = as.acceptNewGoal();

  if(currentState == RobotState::Locked)
  {
    ROS_ERROR("No configuration goals can be executed while the robot interface is in the Locked state.");
    as.setAborted();
    return;
  }

  franka_low_level_driver::FrankaConfigurationGoal frankaGoal;
  frankaGoal.speedFactor = goal->speedFactor;

  for(const auto& doFConfiguration : goal->configuration)
  {
    franka_low_level_driver::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    frankaGoal.configuration.push_back(newConfig);
  }
  currentState = RobotState::Moving;
  configurationClient.sendGoal(frankaGoal,
                               boost::bind(&RobotInstanceFrankaSimulation::configurationResultCB, this, _1, _2),
                               FrankaConfigClient::SimpleActiveCallback(),
                               boost::bind(&RobotInstanceFrankaSimulation::configurationFeedbackCB, this, _1));
}

void RobotInstanceFrankaSimulation::configurationResultCB(
    const actionlib::SimpleClientGoalState& state,
    const franka_low_level_driver::FrankaConfigurationResultConstPtr& result)
{
  configurationResult.resultConfiguration.clear();
  for(const auto& doFConfiguration : result->resultConfiguration)
  {
    high_level_interface::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    configurationResult.resultConfiguration.push_back(newConfig);
  }
  if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
  {
    as.setAborted(configurationResult);
  }
  else
  {
    as.setSucceeded(configurationResult);
    currentState = RobotState::Active;
  }
}

void RobotInstanceFrankaSimulation::configurationFeedbackCB(
    const franka_low_level_driver::FrankaConfigurationFeedbackConstPtr& feedback)
{
  configurationFeedback.interimConfiguration.clear();
  for(const auto& doFConfiguration : feedback->interimConfiguration)
  {
    high_level_interface::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    configurationFeedback.interimConfiguration.push_back(newConfig);
  }
  configurationFeedback.moving = feedback->moving;
  as.publishFeedback(configurationFeedback);
}

bool RobotInstanceFrankaSimulation::handleEmergencyStopRequest(high_level_interface::EmergencyStopRequest& request,
                                                     high_level_interface::EmergencyStopResponse& response)
{
  emergencyStopMsg.request.stop = request.stop;
  if(emergencyStopClient.call(emergencyStopMsg))
  {
    auto resultConfig = emergencyStopMsg.response.resultConfiguration;
    for(const auto& doF : resultConfig)
    {
      high_level_interface::DoFConfiguration responseDof;
      copyDoFConfiguration(responseDof, doF);
      response.resultConfiguration.push_back(responseDof);
    }
    return true;
  }
  return false;
}

bool RobotInstanceFrankaSimulation::controlGripper(high_level_interface::GripperRequest& request,
                        high_level_interface::GripperResponse& response)
{
  gripperMsg.request.open = request.open;
  if (gripperClient.call(gripperMsg))
  {
    response.success = gripperMsg.response.success;
    return true;
  }
  return false;
}

}// namespace RobotControl
}// namespace HighLevelInterface