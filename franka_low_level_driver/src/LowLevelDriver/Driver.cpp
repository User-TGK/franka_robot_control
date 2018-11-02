#include <cinttypes>
#include <cstdio>

#include <Driver.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{
Driver::Driver(const std::string& name, const std::string& hostName)
    : robot(hostName),
      gripper(hostName),
      as(name, false),
      emergencyService(nh.advertiseService("franka_emergency_stop", &Driver::executeEmergencyStop, this)),
      unlockService(nh.advertiseService("unlock_franka", &Driver::unlockRobot, this)),
      gripperService(nh.advertiseService("gripper_franka", &Driver::controlGripper, this))
{
  setDefaultBehavior(robot);
  as.registerGoalCallback(boost::bind(&Driver::executeConfigurationActionGoal, this));
  as.start();
}

Driver::~Driver()
{
  if(as.isActive())
  {
    as.shutdown();
  }
}

void Driver::executeConfigurationActionGoal()
{
  auto goal = as.acceptNewGoal();

  std::array<double, 7> qGoal = toConfiguration(goal);
  MotionGenerator motionGenerator(goal->speedFactor, qGoal);

  try
  {
    robot.control(motionGenerator);
  }
  catch(const franka::Exception& e)
  {
    ROS_ERROR("%s", e.what());

    auto abortionErrors = robot.readOnce().last_motion_errors;
    if(abortionErrors.joint_reflex || abortionErrors.cartesian_reflex || abortionErrors.instability_detected ||
      abortionErrors.self_collision_avoidance_violation || abortionErrors.power_limit_violation || abortionErrors.joint_p2p_insufficient_torque_for_planning)
    {
      auto hLEmergencyStopClient   = nh.serviceClient<high_level_interface::EmergencyStop>("emergency_stop");
      hLEmergencyStop.request.stop = true;
      hLEmergencyStopClient.call(hLEmergencyStop);
      threatPublisher.publishCriticalThreat();
    }
    else
    {
      try
      {
        robot.automaticErrorRecovery();
      }
      catch (const franka::Exception& arg)
      {
        ROS_ERROR("%s", arg.what());
      }
    }
    result.resultConfiguration = getCurrentConfiguration();
    as.setAborted(result);
    
    return;
  }

  result.resultConfiguration = getCurrentConfiguration();
  as.setSucceeded(result);
}

std::array<double, 7> Driver::toConfiguration(const franka_low_level_driver::FrankaConfigurationGoalConstPtr goal)
{
  std::array<double, 7> convertedConfiguration;
  std::array<bool, 7> set = {false, false, false, false, false, false, false};

  for(const auto& conf : goal->configuration)
  {
    convertedConfiguration[conf.channel] = joints[conf.channel].getRadians(conf.angle);
    set[conf.channel] = true;
  }
  // Validate if all joints are set (LibFranka requires all joints to be set)
  // If there are joints that are not set, set the goal to their initial position
  if (goal->configuration.size() < 7)
  {
    auto initialPose = getCurrentConfiguration();
    for (unsigned short doF = 0; doF < 7; ++doF)
    {
      if (set[doF] == false)
      {
        auto initialMatchingDoF = find_if(initialPose.begin(), initialPose.end(), [doF](const franka_low_level_driver::DoFConfiguration& doFConfig){return doFConfig.channel == doF;});
        convertedConfiguration[doF] = joints[doF].getRadians(initialMatchingDoF->angle);
      }
    }
  }
  return convertedConfiguration;
}

bool Driver::unlockRobot(franka_low_level_driver::UnlockFranka::Request& req,
                         franka_low_level_driver::UnlockFranka::Response& res)
{
  if(req.unlock)
  {
    try
    {
      robot.automaticErrorRecovery();
      res.success = true;
    }
    catch(const franka::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      res.success = false;
    }
  }
  else
  {
    res.success = false;
  }
  return true;
}

bool Driver::controlGripper(franka_low_level_driver::GripperFranka::Request& req, franka_low_level_driver::GripperFranka::Response& res)
{
  try
  {
    res.success = false;
    if (req.open)
    {
      // The input values to this function are: 
      // (1) width: Intended opening width. [m]
      // (2) speed: Closing speed. [m/s]
      gripper.move(0.076, 0.05);
      res.success = true;
    }
    else
    {
      // The input values to this function are:
      // (1) Size of the object to grasp. [m]
      // (2) Closing speed. [m/s]
      // (3) Grasping force. [N]
      if (gripper.grasp(0.067, 0.05, 10))
      {
        res.success = true;
      }
    }
  }
  catch(const franka::Exception& e)
  {
    ROS_ERROR("%s", e.what());
    res.success = false;
  }
  return true;
}

bool Driver::executeEmergencyStop(franka_low_level_driver::EmergencyStopFranka::Request& req,
                                  franka_low_level_driver::EmergencyStopFranka::Response& res)
{
  if(req.stop)
  {
    try
    {
      robot.stop();
      gripper.stop();
    }
    catch(const franka::Exception& e)
    {
      ROS_ERROR("FRANKA EMERGENCY STOP FAILED, REASON: %s", e.what());
      return true;
    }
  }
  res.resultConfiguration = getCurrentConfiguration();
  return true;
}

std::vector<franka_low_level_driver::DoFConfiguration> Driver::getCurrentConfiguration()
{
  auto currentPosition = robot.readOnce().q;

  std::vector<franka_low_level_driver::DoFConfiguration> currentConfiguration;
  std::uint8_t channel = 0;

  for(const auto& angle : currentPosition)
  {
    franka_low_level_driver::DoFConfiguration currentConfig;
    currentConfig.channel = channel;
    currentConfig.angle   = Joint::getDegrees(angle);
    currentConfiguration.push_back(currentConfig);
    channel++;
  }
  return currentConfiguration;
}

void Driver::setDefaultBehavior(franka::Robot& robot)
{
  robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  robot.setFilters(750, 750, 750, 750, 750);
}

}// namespace RobotControl
}// nameSpace FrankaLowLevelDriver