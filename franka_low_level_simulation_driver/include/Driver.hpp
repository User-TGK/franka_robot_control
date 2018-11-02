#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <high_level_interface/EmergencyStop.h>
#include <franka_low_level_driver/FrankaConfigurationAction.h>
#include <franka_low_level_driver/EmergencyStopFranka.h>
#include <franka_low_level_driver/UnlockFranka.h>
#include <franka_low_level_driver/GripperFranka.h>

#include <MotionGenerator.hpp>
#include <Joint.hpp>
#include <SerialCommunicationHandler.hpp>
#include <ActionParser.hpp>

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
     * @param serialPortPath the Serial path of the simulated Franka robot
     * @exception throws a franka exception (after a while) if no connection could be established
     * @author Ties Klappe
     */
    Driver(const std::string& name, const std::vector<DegreeOfFreedom>& aDegreeOfFreedoms,const std::string& serialPortPath);

    /**
     * @brief Destroy the Driver object
     * @author Ties Klappe
     */
    ~Driver();

private:
    /**
     * @brief function that will parse a Franka Configuration goal to a configuration in radians
     * 
     * @param goal the goal to be parsed
     * @return std::array<double, 7> the configuration in radians
     * @author Ties Klappe
     */
    std::array<double, 7> toConfiguration(const franka_low_level_driver::FrankaConfigurationGoalConstPtr goal) const;

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
     * @brief function that polls for the result of an action on a seperate thread.
     * The function checks for the SSC32U to respond with a '.' on a movement query
     * or if the function is running longer than the last finish time.
     * This function also publishes action feedback
     * @author Ties Klappe
     * 
     * @param lastFinishedTime the maximum duration the action may take before it should be finished
     * @param readMargin the number of times feedback is published: SHOULD BE AT LEAST 2 TO PREVENT UNDEFINED BEHAVIOUR
     */
    void pollForActionResult(unsigned long long lastFinishedTime, unsigned long readMargin = 2);

    /**
     * @brief function that handles the result of an action and publishes it
     * @author Ties Klappe
     */
    void handleActionResult();

    /**
     * @brief Function that notifies the action handler when an actions result must be published
     * @author Ties Klappe
     */
    void notifyActionHandler();

    /**
     * @brief function that updates the feedback and publishes it
     * @author Ties Klappe
     * 
     * @param result the result from the SSC32U movement query: + (moving) or . (finished). else
     * the string data is corrupted.
     */
    void publishFeedback(const std::string& result);

    /**
     * @brief Function that reads feedback from the SerialCommunicationHandler instance
     * Function should be called on a seperate thread
     * @author Ties Klappe
     */
    void asynchReadFeedback();

    /**
     * @brief Get the Current Configuration of the Franka arm
     * 
     * @return std::vector<franka_low_level_driver::DoFConfiguration> the current configuration
     * @author Ties Klappe
     */
    std::vector<franka_low_level_driver::DoFConfiguration> getCurrentConfiguration();


    /**
     * @brief If something went wrong reading the current configuration from the robot use the local
     * stored latest configuration to notify the High Level Robot Interface of the robot its current position
     * 
     * @return std::vector<al5d_low_level_driver::DoFConfiguration> the current local configuration
     */
    std::vector<franka_low_level_driver::DoFConfiguration> getLocalCurrentConfiguration() const;
    
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
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<franka_low_level_driver::FrankaConfigurationAction> as; ///< The RoS actionserver
    franka_low_level_driver::FrankaConfigurationFeedback feedback; ///< The feedback object (updated before each time feedback is published)
    franka_low_level_driver::FrankaConfigurationResult result; ///< The result of the action to be published
    ros::ServiceServer emergencyService; ///< The ROS service server that handles incoming emergency requests
    ros::ServiceServer unlockService;
    ros::ServiceServer gripperService;
    high_level_interface::EmergencyStop hLEmergencyStop;
    SerialCommunicationHandler serialCommunicationHandler; ///< Communication handler that handles all serial communication
    ActionParser actionParser; ///< Actionparser that parses incoming messages to raw SSC32U string-messages
    std::thread resultPollThread; ///< Thread that polls when the AL5D has reached its position
    std::thread feedbackThread; ///< Thread that asynchronously reads the result
    std::mutex m; ///< Mutex to lock variable continueAllowed accessed by multiple threads
    std::mutex feedbackMutex; ///< Mutex to lock the readFeedback
    std::condition_variable cv; ///< ondition variable that waits untill the continueAllowed boolean is set to true
    std::atomic<bool> emergencyStopActivated; ///< Atomic boolean that is true when during the last action an emergencystop request was called
    bool continueAllowed; ///< Boolean that tells when the action can continue to publishing the result
    std::atomic<bool> timerExpired; ///< Atomic boolean that knows when the result timer is expired
    std::string readFeedback; ///< The feedback that was read
    std::string queryMovementStatus; ///< SSC32U command to check if the robot is moving
    std::string emergencyStopCommand; ///< SSC32U command to immediately stop the robot
};

} //namespace RobotControl
} //nameSpace FrankaLowLevelDriver

#endif //DRIVER_HPP_