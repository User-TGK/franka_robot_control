#include <cmath>

#include <gtest/gtest.h>

#include <Driver.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{

TEST (FrankaConversionSuite, RadiansConversionTest)
{
    Joint joint(2.9671, -2.9671, 2.8973, -2.8973);
    ASSERT_EQ(joint.getRadians(90), 0.5 * M_PI);
    ASSERT_EQ(joint.getRadians(900), 2.8973);
    ASSERT_EQ(joint.getRadians(-900), -2.8973);
}

TEST (FrankaConversionSuite, DegreesConversionTest)
{
    Joint joint(2.9671, -2.9671, 2.8973, -2.8973);
    ASSERT_NEAR(joint.getDegrees(-2.8973), -166.003, 0.001); 
}

} //namespace RobotControl
} //namespace FrankaLowLevelDriver