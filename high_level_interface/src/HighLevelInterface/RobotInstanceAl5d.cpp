#include <RobotInstanceAl5d.hpp>

namespace RobotControl
{
namespace HighLevelInterface
{
RobotInstanceAl5d::RobotInstanceAl5d(const std::string& name)
    : RobotInstance(name, RobotType::Al5d), configurationClient("AL5DDriver", true)
{
  as.registerGoalCallback(boost::bind(&RobotInstanceAl5d::executeConfigurationAction, this));
  as.start();
}

RobotInstanceAl5d::~RobotInstanceAl5d()
{
}

void RobotInstanceAl5d::copyDoFConfiguration(high_level_interface::DoFConfiguration& goal,
                                             const al5d_low_level_driver::DoFConfiguration& origin)
{
  goal.channel = origin.channel;
  goal.angle   = origin.angle;
  goal.speed   = origin.speed;
}

void RobotInstanceAl5d::copyDoFConfiguration(al5d_low_level_driver::DoFConfiguration& goal,
                                             const high_level_interface::DoFConfiguration& origin)
{
  goal.channel = origin.channel;
  goal.angle   = origin.angle;
  goal.speed   = origin.speed;
}

void RobotInstanceAl5d::executeConfigurationAction()
{
  auto goal = as.acceptNewGoal();

  if(currentState == RobotState::Locked)
  {
    ROS_ERROR("No configuration goals can be executed while the robot interface is in the Locked state.");
    as.setAborted();
    return;
  }

  al5d_low_level_driver::Al5dConfigurationGoal al5dGoal;
  al5dGoal.time = goal->time;

  for(const auto& doFConfiguration : goal->configuration)
  {
    al5d_low_level_driver::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    al5dGoal.configuration.push_back(newConfig);
  }
  currentState = RobotState::Moving;
  configurationClient.sendGoal(al5dGoal,
                               boost::bind(&RobotInstanceAl5d::configurationResultCB, this, _1, _2),
                               Al5dConfigClient::SimpleActiveCallback(),
                               boost::bind(&RobotInstanceAl5d::configurationFeedbackCB, this, _1));
}

void RobotInstanceAl5d::configurationResultCB(const actionlib::SimpleClientGoalState& state,
                                              const al5d_low_level_driver::Al5dConfigurationResultConstPtr& result)
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

void RobotInstanceAl5d::configurationFeedbackCB(
    const al5d_low_level_driver::Al5dConfigurationFeedbackConstPtr& feedback)
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

bool RobotInstanceAl5d::toPreprogrammedPosition(high_level_interface::PositionRequest& request,
                                                high_level_interface::PositionResponse& response)
{
  response.success = false;
  try
  {
    auto preProgrammedPosition = preProgrammedPositionFromString(request.position);
    al5d_low_level_driver::Al5dConfigurationGoal al5dGoal;
    al5d_low_level_driver::DoFConfiguration newConfig;
    newConfig.speed = 80;
    switch(preProgrammedPosition)
    {
      case PreProgrammedPosition::Park:
      {
        newConfig.channel = 0;
        newConfig.angle   = 90;
        al5dGoal.configuration.push_back(newConfig);
        newConfig.channel = 1;
        newConfig.angle   = -20;
        al5dGoal.configuration.push_back(newConfig);
        newConfig.channel = 2;
        newConfig.angle   = 100;
        al5dGoal.configuration.push_back(newConfig);
        newConfig.channel = 3;
        newConfig.angle   = 90;
        al5dGoal.configuration.push_back(newConfig);
        newConfig.channel = 4;
        newConfig.angle   = 0;
        al5dGoal.configuration.push_back(newConfig);
        newConfig.channel = 5;
        newConfig.angle   = 0;
        al5dGoal.configuration.push_back(newConfig);

        configurationClient.sendGoal(al5dGoal);
        response.success = true;
        return true;
      }
      case PreProgrammedPosition::Straight:
      {
        for(unsigned long channel = 0; channel < nrOfJoints; ++channel)
        {
          newConfig.channel = channel;
          newConfig.angle   = 0;
          al5dGoal.configuration.push_back(newConfig);
        }
        configurationClient.sendGoal(al5dGoal);
        response.success = true;
        return true;
      }
      case PreProgrammedPosition::Manual:
      {
        al5dGoal.configuration = manualConfiguration;
        configurationClient.sendGoal(al5dGoal);
        response.success = true;
        return true;
      }
    }
  }
  catch(const std::invalid_argument& arg)
  {
    ROS_ERROR("%s", arg.what());
  }
  return false;
}
std::vector<high_level_interface::DoFConfiguration> RobotInstanceAl5d::updateManualPosition(bool update)
{
  emergencyStopMsg.request.stop = false;
  emergencyStopClient.call(emergencyStopMsg);

  if (update)
  {
    manualConfiguration = emergencyStopMsg.response.resultConfiguration;
  }
  
  std::vector<high_level_interface::DoFConfiguration> newManualConfiguration;
  for(const auto& doFConfiguration: manualConfiguration)
  {
    high_level_interface::DoFConfiguration newConfig;
    copyDoFConfiguration(newConfig, doFConfiguration);
    newManualConfiguration.push_back(newConfig);
  }
  return newManualConfiguration;
}

bool RobotInstanceAl5d::controlGripper(high_level_interface::GripperRequest& request,
                            high_level_interface::GripperResponse& response)
{
  al5d_low_level_driver::Al5dConfigurationGoal gripperGoal;
  al5d_low_level_driver::DoFConfiguration newConfig;
  newConfig.channel = 5;
  if (request.open)
  {
    newConfig.angle = -90;
  }
  else
  {
    newConfig.angle = 90;
  }
  gripperGoal.configuration.push_back(newConfig);
  configurationClient.sendGoal(gripperGoal);
  response.success = true;
  return true;
}

bool RobotInstanceAl5d::handleEmergencyStopRequest(high_level_interface::EmergencyStopRequest& request,
                                                   high_level_interface::EmergencyStopResponse& response)
{
  emergencyStopClient           = nh.serviceClient<al5d_low_level_driver::EmergencyStopAl5d>("al5d_emergency_stop");
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

}// namespace RobotControl
}// namespace HighLevelInterface