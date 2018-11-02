#include <ros/ros.h>

#include <DegreeOfFreedom.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{
DegreeOfFreedom::DegreeOfFreedom(unsigned short aChannel,
                                 unsigned long aMinPulseWidth,
                                 unsigned long aMaxPulseWidth,
                                 double aMinAngle,
                                 double aMaxAngle,
                                 double aMaxSpeedPerSecond,
                                 unsigned long aCurrentPos,
                                 unsigned long aTargetPos)
    : channel(aChannel),
      minPulseWidth(aMinPulseWidth),
      maxPulseWidth(aMaxPulseWidth),
      minAngle(aMinAngle),
      maxAngle(aMaxAngle),
      maxSpeedPerSecond(aMaxSpeedPerSecond),
      currentPos(aCurrentPos),
      targetPos(aTargetPos)
{
}

unsigned long DegreeOfFreedom::pulseWidthFromAngle(double angle) const
{
  unsigned long result = ((angle - minAngle) * (maxPulseWidth - minPulseWidth) / (maxAngle - minAngle) + minPulseWidth);

  if(result > maxPulseWidth)
  {
    return maxPulseWidth;
  }
  else if(result < minPulseWidth)
  {
    return minPulseWidth;
  }
  return result;
}

double DegreeOfFreedom::angleFromPulseWidth(unsigned long pulseWidth) const
{
  if(pulseWidth < minPulseWidth)
  {
    pulseWidth = minPulseWidth;
  }
  else if(pulseWidth > maxPulseWidth)
  {
    pulseWidth = maxPulseWidth;
  }
  return ((pulseWidth - minPulseWidth) * (maxAngle - minAngle) / (maxPulseWidth - minPulseWidth) + minAngle);
}

unsigned long DegreeOfFreedom::getConvertedSpeedFromDegreesPerSecond(double speed) const
{
  unsigned short speedFactor = maxSpeed / minSpeed;
  double result              = ((speed - (maxSpeedPerSecond / speedFactor)) * (maxSpeed - minSpeed) /
                       (maxSpeedPerSecond - (maxSpeedPerSecond / speedFactor)) +
                   minSpeed);

  if(result > maxSpeed)
  {
    return maxSpeed;
  }
  if(result < minSpeed)
  {
    ROS_DEBUG("THE MINIMUM DEGREES PER SECOND FOR THE DOF ON CHANNEL %s IS %s.",
              std::to_string(channel).c_str(),
              std::to_string(maxSpeedPerSecond / speedFactor).c_str());
    return minSpeed;
  }
  return result;
}

unsigned short DegreeOfFreedom::getChannel() const
{
  return this->channel;
}

unsigned long DegreeOfFreedom::getCurrentPos() const
{
  return this->currentPos;
}

unsigned long DegreeOfFreedom::getTargetPos() const
{
  return this->targetPos;
}

void DegreeOfFreedom::setCurrentPos(unsigned long aCurrentPos)
{
  this->currentPos = aCurrentPos;
}

void DegreeOfFreedom::setTargetPos(unsigned long aTargetPos)
{
  this->targetPos = aTargetPos;
}

}// namespace RobotControl
}// namespace Al5dLowLevelDriver