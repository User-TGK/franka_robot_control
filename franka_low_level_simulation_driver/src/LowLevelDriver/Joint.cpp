#include <cmath>

#include <Joint.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{
Joint::Joint(const double aQMax, const double aQMin, const double aQMaxSoft, const double aQMinSoft)
    : QMax(aQMax), QMin(aQMin), QMaxSoft(aQMaxSoft), QMinSoft(aQMinSoft), softMode(true)
{
}

Joint::~Joint()
{
}

double Joint::getRadians(const double degrees) const
{
  auto radians = (degrees * M_PI) / 180;
  return validateAngle(radians);
}

double Joint::getDegrees(const double radians)
{
  return (radians * 180) / M_PI;
}

double Joint::validateAngle(const double angle) const
{
  if(softMode)
  {
    if(angle > QMinSoft && angle < QMaxSoft)
    {
      return angle;
    }
    return (angle > QMaxSoft ? QMaxSoft : QMinSoft);
  }
  if(angle > QMin && angle < QMax)
  {
    return angle;
  }
  return (angle > QMax ? QMax : QMin);
}

}// namespace RobotControl
}// nameSpace FrankaLowLevelDriver