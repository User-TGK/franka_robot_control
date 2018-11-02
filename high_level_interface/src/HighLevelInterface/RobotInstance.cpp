#include <RobotInstance.hpp>

namespace RobotControl
{
namespace HighLevelInterface
{
RobotInstance::RobotInstance(const std::string& aName, const RobotType aRobotType)
    : as(aName, false), currentState(RobotState::Active), robotType(aRobotType)
{
}

RobotInstance::~RobotInstance()
{
}

void RobotInstance::setCurrentState(const RobotState& aCurrentState)
{
  this->currentState = aCurrentState;
}

RobotState RobotInstance::getCurrentState() const
{
  return this->currentState;
}

RobotType RobotInstance::getType() const
{
  return this->robotType;
}

PreProgrammedPosition RobotInstance::preProgrammedPositionFromString(const std::string& str) const
{
  auto found = PositionTypeMap.find(str);
  if(found == PositionTypeMap.end())
  {
    throw std::invalid_argument("The string " + str + " is not a supported preprogrammed position.");
  }
  return found->second;
}

std::string RobotInstance::strFromPreprogrammedPosition(const PreProgrammedPosition& preProgrammedPosition) const
{
  for (const auto& pair: PositionTypeMap)
  {
    if (pair.second == preProgrammedPosition)
    {
      return pair.first;
    }
  }
  throw std::invalid_argument("No string representation found for unsupported preprogrammed position.");
}

}// namespace RobotControl
}// namespace HighLevelInterface