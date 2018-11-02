#ifndef ACTION_PARSER_HPP_
#define ACTION_PARSER_HPP_

#include <string>

#include <ros/ros.h>
#include <al5d_low_level_driver/Al5dConfigurationAction.h>

#include <DegreeOfFreedom.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

class ActionParser
{
public:
    /**
     * @brief Construct a new Action Parser object
     * @author Ties Klappe
     * 
     * @param aDegreeOfFreedoms the DoF's attached to the SSC32U
     * @param aNumberOfDoFs the number of the DoF's specifically for the AL5D
     */
    ActionParser(const std::vector<DegreeOfFreedom>& aDegreeOfFreedoms, unsigned short aNumberOfDoFs);

    /**
     * @brief Destroy the Action Parser object
     * @author Ties Klappe
     */
    ~ActionParser();

    /**
     * @brief function that tries to parse an Al5dConfiguration goal to a SSC32U command string
     * @author Ties Klappe
     * 
     * @param goal the Al5dConfiguration goal to be parsed
     * @return std::string the SSC32U command in string form
     * @exception throws an exception if no DoFConfiguration objects were set in the configuration list
     */
    std::string configurationToSSC32UCommandString(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal) const;

    /**
     * @brief Get the Last Finished Time object
     * @author Ties Klappe
     * 
     * @param goal the goal will determine the last finished time in milliseconds
     * @return unsigned long long the calculated last finished time in milliseconds
     */
    unsigned long long getLastFinishedTime(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal) const;

    /**
     * @brief Get the Degree Of Freedoms object
     * @author Ties Klappe
     * 
     * @return std::vector<DegreeOfFreedom> the vector with all the intialized DoF's
     */
    const std::vector<DegreeOfFreedom>& getDegreeOfFreedoms() const;

    /**
     * @brief function that converts a specified pulseWidth for a specified DoF to an angle in degrees
     * @author Ties Klappe
     * 
     * @param channel the channel the DoF is attached to
     * @param pulseWidth the specified pulseWidth to be converted
     * @return double the angle in degrees
     */
    double toAngle(unsigned short channel, unsigned long pulseWidth) const;

    /**
     * @brief Set the Target Positions for each DoF based on an incoming goal
     * @author Ties Klappe
     * 
     * @param goal the goal that contains the target configuration
     */
    void setTargetPositions(const al5d_low_level_driver::Al5dConfigurationGoalConstPtr goal);

    /**
     * @brief update the current positions of each DoF
     * @author Ties Klappe
     * 
     * @param newPositions the new positions all doFs are at 
     */
    void updateCurrentPositions(const std::map<unsigned short, unsigned long>& newPositions);

private:
    /**
     * @brief Get the DoF object
     * @author Ties Klappe
     * 
     * @param channel the channel the DoF is attached to
     * @return DegreeOfFreedom the DoF object
     * @exception throws an std::runtime_error exception if there is no DoF attached to the specified channel
     */
    const DegreeOfFreedom& getDoF(unsigned short channel) const;

    /**
     * @brief function that converts a specified angle in degrees for a specified DoF to a pulseWidth
     * @author Ties Klappe
     * 
     * @param channel the channel the DoF is attached to
     * @param angle the specified angle in degrees to be converted
     * @return unsigned long the pulseWidth
     */
    unsigned long toPulseWidth(unsigned short channel, double angle) const;

    /**
     * @brief function that converts a speed in degrees per second to a speed in uS per second
     * @author Ties Klappe
     * 
     * @param channel channel the channel the DoF is attached to
     * @param speed the speed in degrees per second to be converted
     * @return unsigned short the speed in uS per second
     */
    unsigned short touSPerSecond(unsigned short channel, double speed) const;

    std::vector<DegreeOfFreedom> degreeOfFreedoms; ///< The attached degreesOfFreedom to the SSC32U serial device
    unsigned short numberOfDoFs; ///< The number of DoF's specifically for the AL5D robot
};

} // namespace RobotControl
} // namespace Al5dLowLevelDriver

#endif //ACTION_PARSER_HPP_