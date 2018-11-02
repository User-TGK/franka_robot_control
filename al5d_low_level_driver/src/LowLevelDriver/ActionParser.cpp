#include <sstream>
#include <stdexcept>

#include <ActionParser.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{
ActionParser::ActionParser(const std::vector<DegreeOfFreedom>& aDegreeOfFreedoms, unsigned short aNumberOfDoFs)
    : degreeOfFreedoms(aDegreeOfFreedoms), numberOfDoFs(aNumberOfDoFs)
{
  assert(degreeOfFreedoms.size() == numberOfDoFs);
}

ActionParser::~ActionParser()
{
}

std::string
ActionParser::configurationToSSC32UCommandString(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal) const
{
  if(goal->configuration.size() == 0)
  {
    throw std::invalid_argument("THE RECEIVED CONFIGURATION DOES NOT CONTAIN ANY CONFIGURED ANGLES.");
  }

  std::stringstream command;
  unsigned long goalSize = goal->configuration.size();
  for(std::size_t i = 0; i < goalSize; ++i)
  {
    try
    {
      unsigned long convertedPulseWidth = toPulseWidth(goal->configuration[i].channel, goal->configuration[i].angle);

      command << "#" << std::to_string(goal->configuration[i].channel) << "P" << convertedPulseWidth;
      if(goal->configuration[i].speed != 0.0)
      {
        command << "S" << touSPerSecond(goal->configuration[i].channel, goal->configuration[i].speed);
      }
    }
    catch(const std::invalid_argument& a)
    {
      ROS_DEBUG("INVALID ARGUMENT: %s", a.what());
    }
  }
  if(goal->time != 0.0)
  {
    command << "T" << goal->time;
  }

  command << "\r";
  return command.str();
}

void ActionParser::setTargetPositions(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal)
{
  unsigned long goalSize = goal->configuration.size();

  for(std::size_t i = 0; i < goalSize; ++i)
  {
    unsigned long convertedPulseWidth = toPulseWidth(goal->configuration[i].channel, goal->configuration[i].angle);
    try
    {
      bool success = false;
      for(auto& doF : degreeOfFreedoms)
      {
        if(doF.getChannel() == goal->configuration[i].channel)
        {
          doF.setTargetPos(convertedPulseWidth);
          success = true;
          break;
        }
      }
      if(!success)
      {
        throw std::invalid_argument("DOF FROM GOAL NOT CONFIGURED IN AL5D DRIVER.");
      }
    }
    catch(const std::invalid_argument& a)
    {
      ROS_DEBUG("%s", a.what());
    }
  }
}

const std::vector<DegreeOfFreedom>& ActionParser::getDegreeOfFreedoms() const
{
  return this->degreeOfFreedoms;
}

unsigned long long
ActionParser::getLastFinishedTime(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal) const
{
  /** The slowest servo attached to the LynxMotion can move 180 degrees in 0.84 seconds */
  /** Source: Attachments: hs755hb.pdf */
  double maxMovingTime = 840;
  unsigned long long lastFinishedTime;
  short maxDegree             = 90;
  short minDegree             = -90;
  unsigned long topDifference = 0;

  for(const auto& doFConfiguration : goal->configuration)
  {
    unsigned long maxDifference = std::abs(maxDegree - doFConfiguration.angle);
    if(std::abs(doFConfiguration.angle - minDegree) > maxDifference)
    {
      maxDifference = std::abs(doFConfiguration.angle - minDegree);
    }
    if(maxDifference > topDifference)
    {
      topDifference = maxDifference;
    }
  }
  lastFinishedTime = maxMovingTime / 180 * topDifference;
  if(goal->time != 0)
  {
    if(lastFinishedTime > goal->time)
    {
      return lastFinishedTime;
    }
    return goal->time;
  }
  return lastFinishedTime;
}

void ActionParser::updateCurrentPositions(const std::map<unsigned short, unsigned long>& newPositions)
{
  for(const auto& newPosition : newPositions)
  {
    bool success = false;
    for(auto& doF : degreeOfFreedoms)
    {
      if(doF.getChannel() == newPosition.first)
      {
        doF.setCurrentPos(newPosition.second);
        success = true;
        break;
      }
    }
    if(!success)
    {
      throw std::invalid_argument("CAN'T UPDATE CURRENT POSITION FOR AL5D-DOF.");
    }
  }
}

const DegreeOfFreedom& ActionParser::getDoF(unsigned short channel) const
{
  auto it = std::find_if(degreeOfFreedoms.begin(), degreeOfFreedoms.end(), [&channel](const DegreeOfFreedom& doF) {
    return doF.getChannel() == channel;
  });

  if(it == degreeOfFreedoms.end())
  {
    throw std::invalid_argument("THE AL5D ROBOT DOES NOT CONTAIN A DEGREE OF FREEDOM ON CHANNEL " +
                                std::to_string(channel) + ".");
  }
  return *it;
}

unsigned long ActionParser::toPulseWidth(unsigned short channel, double angle) const
{
  return getDoF(channel).pulseWidthFromAngle(angle);
}

double ActionParser::toAngle(unsigned short channel, unsigned long pulseWidth) const
{
  return getDoF(channel).angleFromPulseWidth(pulseWidth);
}

unsigned short ActionParser::touSPerSecond(unsigned short channel, double speed) const
{
  return getDoF(channel).getConvertedSpeedFromDegreesPerSecond(speed);
}

}// namespace RobotControl
}// namespace Al5dLowLevelDriver