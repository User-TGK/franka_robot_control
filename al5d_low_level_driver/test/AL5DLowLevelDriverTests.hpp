#ifndef AL5D_LOW_LEVEL_DRIVER_TESTS_
#define AL5D_LOW_LEVEL_DRIVER_TESTS_

#include <gtest/gtest.h>
#include <Driver.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

/**
 * @brief function to set an example set of DoF's (used in the following unittests)
 * @author Ties Klappe
 * 
 * @return std::vector<RobotControl::Al5dLowLevelDriver::DegreeOfFreedom> the example DoF set
 */
std::vector<RobotControl::Al5dLowLevelDriver::DegreeOfFreedom> setDoFs()
{
    std::vector<RobotControl::Al5dLowLevelDriver::DegreeOfFreedom> degreeOfFreedoms;
    
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(0, 500, 2500, -90, 90, 272.73, 0, 0));   // AL5D's - Base servo
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(1, 500, 2500, -90, 90, 315.79, 0, 0));   // AL5D's - Turret servo
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(2, 500, 2500, -90, 90, 214.29, 0, 0));   // AL5D's - Upperarm servo
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(3, 500, 2500, -90, 90, 250.00, 0, 0));   // AL5D's - Forearm servo
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(4, 500, 2500, -90, 90, 285.71, 0, 0));   // AL5D's - Gripper
    degreeOfFreedoms.push_back(RobotControl::Al5dLowLevelDriver::DegreeOfFreedom(5, 500, 2500, -90, 90, 285.71, 0, 0));   // AL5D's - Wrist servo

    return degreeOfFreedoms;
}

/**
 * @brief Unittests for the DegreeOfFreedom class that tests if a pulse width is correctly converted
 * from an angle (in degrees) and its alternative flows: if an angle is above the maximum angle
 * it will return the maximum pulse width (Alternative flowA) and if an angle is under the minimum angle it will
 * return the minimum pulse width (Alternative flow B)
 * @author Ties Klappe
 */
TEST (DegreeOfFreedomSuite, PulseWidthFromAngleTest)
{
    DegreeOfFreedom doF(0, 500, 2500, -90, 90, 272.73, 0, 0);
    ASSERT_EQ(doF.pulseWidthFromAngle(0), 1500);

    ActionParser actionParser(setDoFs(), 6);
    ASSERT_EQ(actionParser.toAngle(0, 1396), -9.36);

    ASSERT_EQ(actionParser.toAngle(0, 300), -90); // Alternative flow A
    ASSERT_EQ(actionParser.toAngle(0, 3000), 90); // Alternative flow B
}

/**
 * @brief Unittests for the DegreeOfFreedom class that tests if an angle is correctly converted from
 * a pulse width and its alternative flows: if a pulse width is above the maximum pulse width
 * it will return the maximum angle (in degrees) (Alternative flow A) and if a pulse width is under the minimum
 * pulse width it will return the minimum angle (Alternative flow B)
 * @author Ties Klappe
 */
TEST (DegreeOfFreedomSuite, AngleFromPulseWidthTest)
{
    DegreeOfFreedom doF(0, 500, 2500, -90, 90, 272.73, 0, 0);

    ASSERT_EQ(doF.angleFromPulseWidth(1500), 0); 
    ASSERT_EQ(doF.angleFromPulseWidth(100), -90); // Alternative flow A
    ASSERT_EQ(doF.angleFromPulseWidth(2600), 90); // Alternative flow B
}

/**
 * @brief Unittests for the convertion of a speed in rad/second to its mapped pulse with
 * @author Ties Klappe
 * 
 */
TEST (DegreeOfFreedomSuite, PulseWidthSpeedFromDegreesPerSecondTest)
{
    DegreeOfFreedom doF(0, 500, 2500, -90, 90, 272.73, 0, 0);
    ASSERT_EQ(doF.getConvertedSpeedFromDegreesPerSecond(100), 916);
    ASSERT_EQ(doF.getConvertedSpeedFromDegreesPerSecond(10), 500);
    ASSERT_EQ(doF.getConvertedSpeedFromDegreesPerSecond(400), 2500);
}

/**
 * @brief Unittests for the SSC32U action parser class: it will check if an Al5dConfigurationGoal
 * is correctly parsed into a SSC32U Command string and if the last possible finished time is correctly calculated
 * @author Ties Klappe
 * 
 */
TEST (SSC32UActionParserSuite, SSC32UCommandTest)
{
    ActionParser actionParser(setDoFs(), 6);

    al5d_low_level_driver::Al5dConfigurationGoal initGoal;
    al5d_low_level_driver::DoFConfiguration configurationMessage;

    initGoal.time = 0;
    configurationMessage.channel = 0;
    configurationMessage.angle = 50;
    configurationMessage.speed = 55;
    initGoal.configuration.push_back(configurationMessage);

    auto goal = boost::make_shared<al5d_low_level_driver::Al5dConfigurationGoal>(initGoal);

    ASSERT_EQ(actionParser.configurationToSSC32UCommandString(goal), "#0P2055S504\r");
    ASSERT_EQ(actionParser.getLastFinishedTime(goal), 653);
}

/**
 * @brief Unittest for the actionparser class: it will check if local stored configuration is correctly
 * updated and used.
 * @author Ties Klappe
 * 
 */
TEST (SSC32UActionParserSuite, LocalConfigurationTest)
{
    ActionParser actionParser(setDoFs(), 6);
    al5d_low_level_driver::Al5dConfigurationGoal targetGoal;
    al5d_low_level_driver::DoFConfiguration configurationMessage;

    targetGoal.time = 0;
    configurationMessage.channel = 0;
    configurationMessage.angle = 50;
    configurationMessage.speed = 55;
    targetGoal.configuration.push_back(configurationMessage);

    auto goal = boost::make_shared<al5d_low_level_driver::Al5dConfigurationGoal>(targetGoal);
    actionParser.setTargetPositions(goal);


    std::map<unsigned short, unsigned long> currentPositions;

    for (const auto& doF: actionParser.getDegreeOfFreedoms())
    {
        currentPositions.insert(std::pair<unsigned short, unsigned long>(doF.getChannel(), doF.getTargetPos()));
    }

    actionParser.updateCurrentPositions(currentPositions);
    ASSERT_EQ(actionParser.toAngle(0, 500), -90);
}

}
}

#endif //AL5D_LOW_LEVEL_DRIVER_TESTS_