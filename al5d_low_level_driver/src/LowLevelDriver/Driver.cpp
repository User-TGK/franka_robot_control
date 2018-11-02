#include <Driver.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{
Driver::Driver(const std::string& name,
               const std::vector<DegreeOfFreedom>& aDegreeOfFreedoms,
               const std::string& serialPortPath)
    : as(name, false),
      serialCommunicationHandler(serialPortPath),
      actionParser(aDegreeOfFreedoms, 6),
      emergencyStopActivated(false),
      continueAllowed(false),
      timerExpired(false),
      queryMovementStatus("Q"),
      emergencyStopCommand("STOP")

{
  queryMovementStatus += '\r';
  emergencyStopCommand += '\r';

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
  // When receiving a new goal the callback is not allowed to continue straight to handling the result
  continueAllowed = false;

  // Accepting and parsing incoming configuration and calculating the expected finished time
  const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal = as.acceptNewGoal();
  double lastFinishedTime                                         = actionParser.getLastFinishedTime(goal);

  try
  {
    std::string SSC32UCommand = "";
    try
    {
      SSC32UCommand = actionParser.configurationToSSC32UCommandString(goal);
      actionParser.setTargetPositions(goal);
    }
    catch(const std::invalid_argument& a)
    {
      ROS_DEBUG("%s", a.what());
    }
    if(as.isActive() && ros::ok())
    {
      serialCommunicationHandler.write(SSC32UCommand);
    }
  }
  catch(const std::runtime_error& warn)
  {
    ROS_DEBUG("WRITING FAILED BECAUSE: %s.", warn.what());
  }

  resultPollThread = std::thread(&Driver::pollForActionResult, this, lastFinishedTime, 5);
  std::unique_lock<std::mutex> lk(m);

  // Wait untill the resultPollThread (worker thread) tells we can continue
  cv.wait(lk, [&] { return Driver::continueAllowed; });
  this->handleActionResult();

  lk.unlock();
}

void Driver::handleActionResult()
{
  result.resultConfiguration = this->getCurrentConfiguration();

  // If the robot was forced to stop set the action to aborted and reset emergency stop
  if(emergencyStopActivated)
  {
    emergencyStopActivated = false;
    as.setAborted(result);
  }
  // Else publish the result and reset the timer
  else
  {
    timerExpired               = false;
    as.setSucceeded(result);
  }
  resultPollThread.join();
}

bool Driver::executeEmergencyStop(al5d_low_level_driver::EmergencyStopAl5d::Request& req,
                                  al5d_low_level_driver::EmergencyStopAl5d::Response& res)
{
  ROS_DEBUG("EXECUTING AL5D EMERGENCY STOP");
  if(req.stop)
  {
    try
    {
      serialCommunicationHandler.write(emergencyStopCommand);
      emergencyStopActivated = true;
    }
    catch(const std::runtime_error& e)
    {
      ROS_DEBUG("FAILED WRITING THE SSC32U EMERGENCY STOP COMMAND. REASON: %s", e.what());
      return false;
    }
  }
  res.resultConfiguration = getCurrentConfiguration();
  return true;
}

void Driver::pollForActionResult(unsigned long long lastFinishedTime, unsigned long readMargin)
{
  // Double the lastfinishedTime so it will correspond with a loaded gripper
  lastFinishedTime = lastFinishedTime * 2;

  unsigned long long pollingTime = 0;
  double sleepRate               = lastFinishedTime / readMargin;

  m.lock();
  while(!continueAllowed)
  {
    m.unlock();
    ros::Time startTime = ros::Time::now();
    if(emergencyStopActivated)
    {
      this->notifyActionHandler();
      break;
    }
    else if(pollingTime >= lastFinishedTime)
    {
      timerExpired = true;
      this->notifyActionHandler();
      break;
    }
    try
    {
      feedbackThread = std::thread(&Driver::asynchReadFeedback, this);
      serialCommunicationHandler.write(queryMovementStatus);

      feedbackThread.join();

      feedbackMutex.lock();
      publishFeedback(readFeedback);
    }
    catch(const std::runtime_error& error)
    {
      ROS_DEBUG("%s", error.what());
    }
    if(readFeedback == ".")
    {
      this->notifyActionHandler();
      feedbackMutex.unlock();
      break;
    }
    feedbackMutex.unlock();

    if(pollingTime + sleepRate >= lastFinishedTime)
    {
      sleepRate = lastFinishedTime - pollingTime;
    }
    ros::Duration((sleepRate / 1000)).sleep();
    ros::Time endTime  = ros::Time::now();
    ros::Duration diff = endTime - startTime;

    double timeDuration = diff.toSec();
    pollingTime += (timeDuration * 1000);
    m.lock();
  }
  std::map<unsigned short, unsigned long> currentPositions;

  for(const auto& doF : actionParser.getDegreeOfFreedoms())
  {
    currentPositions.insert(std::pair<unsigned short, unsigned long>(doF.getChannel(), doF.getTargetPos()));
  }
  try
  {
    actionParser.updateCurrentPositions(currentPositions);
  }
  catch(const std::runtime_error& e)
  {
    ROS_DEBUG("%s", e.what());
  }
}

void Driver::asynchReadFeedback()
{
  feedbackMutex.lock();
  try
  {
    readFeedback = serialCommunicationHandler.timedRead(15);
  }
  catch(const std::runtime_error& e)
  {
    ROS_DEBUG("%s", e.what());
  }
  feedbackMutex.unlock();
}

std::vector<al5d_low_level_driver::DoFConfiguration> Driver::getCurrentConfiguration()
{
  std::vector<al5d_low_level_driver::DoFConfiguration> currentConfiguration;
  std::string pulseWidthQueryPrefix = "QP";
  std::stringstream singleDoFPWQuery;

  for(const auto& doF : actionParser.getDegreeOfFreedoms())
  {
    al5d_low_level_driver::DoFConfiguration currentConfig;
    singleDoFPWQuery.str("");
    currentConfig.channel = doF.getChannel();

    singleDoFPWQuery << pulseWidthQueryPrefix << std::to_string(doF.getChannel());
    singleDoFPWQuery << '\r';
    serialCommunicationHandler.write(singleDoFPWQuery.str());

    try
    {
      long pulseWidth = 0;
      try
      {
        // Reading a pulse width query can take up to 5ms according to the LynxMotion datasheet
        pulseWidth = std::stoi(serialCommunicationHandler.timedRead(15));
      }
      catch(const std::invalid_argument& a)
      {
        ROS_DEBUG("%s", a.what());
      }

      currentConfig.angle = actionParser.toAngle(doF.getChannel(), pulseWidth * 10);
      currentConfiguration.push_back(currentConfig);
    }
    catch(const std::runtime_error& error)
    {
      ROS_DEBUG("%s", error.what());
    }
  }
  // If something went wrong when reading the Pulse Width Query set the local stored configuration
  if(currentConfiguration.size() != actionParser.getDegreeOfFreedoms().size())
  {
    return getLocalCurrentConfiguration();
  }
  return currentConfiguration;
}

std::vector<al5d_low_level_driver::DoFConfiguration> Driver::getLocalCurrentConfiguration() const
{
  std::vector<al5d_low_level_driver::DoFConfiguration> currentConfiguration;

  for(auto const& doF : actionParser.getDegreeOfFreedoms())
  {
    al5d_low_level_driver::DoFConfiguration currentConfig;
    currentConfig.channel = doF.getChannel();
    currentConfig.angle   = actionParser.toAngle(doF.getChannel(), doF.getCurrentPos());
    currentConfiguration.push_back(currentConfig);
  }
  return currentConfiguration;
}

void Driver::publishFeedback(const std::string& result)
{
  if(result == ".")
  {
    feedback.moving = false;
  }
  else if(result == "+")
  {
    feedback.moving = true;
  }
  else
  {
    ROS_DEBUG("CORRUPT DATA WAS %s", result.c_str());
    throw std::runtime_error("CORRUPT DATA WHEN READING FROM SERIAL DEVICE SSC32U.");
  }
  feedback.interimConfiguration = this->getCurrentConfiguration();
  as.publishFeedback(feedback);
}

void Driver::notifyActionHandler()
{
  std::lock_guard<std::mutex> lk(m);

  continueAllowed = true;
  cv.notify_one();
}

}// namespace RobotControl
}// namespace Al5dLowLevelDriver