#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>

#include <high_level_interface/EmergencyStop.h>
#include <franka_low_level_driver/FrankaConfigurationAction.h>
#include <franka_low_level_driver/EmergencyStopFranka.h>
#include <franka_low_level_driver/UnlockFranka.h>
#include <franka_low_level_driver/GripperFranka.h>

#include <MotionGenerator.hpp>
#include <Joint.hpp>
#include <safety/ThreatPublisher.hpp>

namespace RobotControl
{
namespace FrankaLowLevelDriver
{

/**
 * @brief The Franka Driver class is basically a conversion class between the ArmSistant High Level Robot Interface
 * and the LibFranka. The Driver class provides interfaces that will make the functionality of the LibFranka
 * available (only the functionality that is used by the ArmSistant system).
 * @author Ties Klappe
 */
class Driver
{
public:
    /**
     * @brief Construct a new Driver object
     * 
     * @param nodeName the name of the configuration server-node (topic to publish configuration goals to)
     * @param hostName the host-name of the Franka robot
     * @exception throws a franka exception (after a while) if no connection could be established
     * @author Ties Klappe
     */
    Driver(const std::string& name, const std::string& hostName);

    /**
     * @brief Destroy the Driver object
     * @author Ties Klappe
     */
    ~Driver();

private:
    Safety::ThreatPublisher threatPublisher;

    /**
     * @brief function that will parse a Franka Configuration goal to a configuration in radians
     * if there are joints that are not set, they will be set to their initial position
     * 
     * @param goal the goal to be parsed
     * @return std::array<double, 7> the configuration in radians
     * @author Ties Klappe
     */
    std::array<double, 7> toConfiguration(const franka_low_level_driver::FrankaConfigurationGoalConstPtr goal);

    /**
     * @brief Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
     * Values are based on the Franka example
     * @param robot Robot instance to set behavior on.
     * @author Ties Klappe
     */
    void setDefaultBehavior(franka::Robot& robot);

    /**
     * @brief callback that will execute the Franka stop function
     * 
     * @param req the request of the emergency stop
     * @param res the result of the emergency stop
     * @return true if the emergency stop was successfully executed
     * @return false if the emergency stop was not successfully executed
     * @exception throws a franka runtime exception if the stop command could not be executed
     * @author Ties Klappe
     */
    bool executeEmergencyStop(franka_low_level_driver::EmergencyStopFranka::Request& req, franka_low_level_driver::EmergencyStopFranka::Response& res);

    /**
     * @brief function that will try to unlock the franka robot
     * 
     * @param req the request of the unlock service
     * @param res the response to the unlock request
     * @return true if it was possible to start the service
     * @return false if it was not possible to start the service
     * @exception throws a franka runtime exception if it was not possible to unlock the robot
     * @author Ties Klappe
     */
    bool unlockRobot(franka_low_level_driver::UnlockFranka::Request& req, franka_low_level_driver::UnlockFranka::Response& res);

    /**
     * @brief function that will control the franka gripper
     * 
     * @param req the request (see .srv file for description)
     * @param res the response (see .srv file for description)
     * @return true if the connection with the gripper could be established
     * @return false if the connection with the gripper could not be established
     * @author Ties Klappe
     */
    bool controlGripper(franka_low_level_driver::GripperFranka::Request& req, franka_low_level_driver::GripperFranka::Response& res);

    /**
     * @brief callback function for the Franka Configuration Action. The function will be called when a 
     * new Franka -configuration goal is received
     * @exception throws a franka runtime exception if the configuration could not be reached
     * @author Ties Klappe
     */
    void executeConfigurationActionGoal();

    /**
     * @brief Get the Current Configuration of the Franka arm
     * 
     * @return std::vector<franka_low_level_driver::DoFConfiguration> the current configuration
     * @author Ties Klappe
     */
    std::vector<franka_low_level_driver::DoFConfiguration> getCurrentConfiguration();
    /**
     * @brief set the hard and soft limits for each Joint on the Franka.
     * The values are based on the limits described on the 
     * <a href="https://frankaemika.github.io/docs/control_parameters.html">FCI documentation page.</a>
     */
    const std::array<Joint, 7> joints =
    {{
        Joint(2.9671, -2.9671, 2.8973, -2.8973),
        Joint(1.8326, -1.8326, 1.7628, -1.7628),
        Joint(2.9671, -2.9671, 2.8973, -2.8973),
        Joint(0.0873, -3.1416, 0.0175, -3.0718),
        Joint(2.9671, -2.9671, 2.8973, -2.8973),
        Joint(3.8223, -0.0873, 3.7525, -0.0175),
        Joint(2.9671, -2.9671, 2.8973, -2.8973)
    }};
    franka::Robot robot; ///< The LibFranka Robot instance
    franka::Gripper gripper; ///< THe LibFranka Gripper instance
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<franka_low_level_driver::FrankaConfigurationAction> as; ///< The RoS actionserver
    franka_low_level_driver::FrankaConfigurationFeedback feedback; ///< The feedback object (updated before each time feedback is published)
    franka_low_level_driver::FrankaConfigurationResult result; ///< The result of the action to be published
    ros::ServiceServer emergencyService; ///< The ROS service server that handles incoming emergency requests
    ros::ServiceServer unlockService;
    ros::ServiceServer gripperService;
    high_level_interface::EmergencyStop hLEmergencyStop;
};

} //namespace RobotControl
} //nameSpace FrankaLowLevelDriver

#endif //DRIVER_HPP_