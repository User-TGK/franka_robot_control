#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <al5d_low_level_driver/EmergencyStopAl5d.h>
#include <actionlib/server/simple_action_server.h>

#include <ActionParser.hpp>
#include <SerialCommunicationHandler.hpp>

namespace RobotControl
{
namespace Al5dLowLevelDriver
{

class Driver
{
public:
    /**
     * @brief Construct a new Driver object
     * @author Ties Klappe
     * 
     * @param name the name of the action server/driver
     * @param aDegreeOfFreedoms the DoF's attached to the SSC32U
     * @param serialPortPath the path to the serial port the SSC32U is attached to
     */
    Driver(const std::string& name, const std::vector<DegreeOfFreedom>& aDegreeOfFreedoms, const std::string& serialPortPath);
    
    /**
     * @brief Destroy the Driver object
     * @author Ties Klappe
     */
    ~Driver();

    /**
     * @brief Callback function that is called when an emergency stop request is received
     * 
     * @param req the request of the service
     * @param res the response send after the request is received
     * @return true if the request was successfull
     * @return false if the request was not successfull
     */
    bool executeEmergencyStop(al5d_low_level_driver::EmergencyStopAl5d::Request &req, al5d_low_level_driver::EmergencyStopAl5d::Response &res);

private:
    /**
     * @brief Callback function that is called when a new Al5dConfigurationGoal is received
     * @author Ties Klappe
     */
    void executeConfigurationActionGoal();

    /**
     * @brief function that gets the current robot configuration (current position) by using
     * query pulse width commands and reading the results
     * @author Ties Klappe
     * 
     * @return std::vector<al5d_low_level_driver::DoFConfiguration> the current configuration
     */
    std::vector<al5d_low_level_driver::DoFConfiguration> getCurrentConfiguration();

    /**
     * @brief If something went wrong reading the current configuration from the robot use the local
     * stored latest configuration to notify the High Level Robot Interface of the robot its current position
     * 
     * @return std::vector<al5d_low_level_driver::DoFConfiguration> the current local configuration
     */
    std::vector<al5d_low_level_driver::DoFConfiguration> getLocalCurrentConfiguration() const;
    /**
     * @brief function that updates the feedback and publishes it
     * @author Ties Klappe
     * 
     * @param result the result from the SSC32U movement query: + (moving) or . (finished). else
     * the string data is corrupted.
     */
    void publishFeedback(const std::string& result);

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
     * @brief Function that reads feedback from the SerialCommunicationHandler instance
     * Function should be called on a seperate thread
     * @author Ties Klappe
     */
    void asynchReadFeedback();
 
    ros::NodeHandle nh; ///< The RoS nodehandler instance
    actionlib::SimpleActionServer<al5d_low_level_driver::Al5dConfigurationAction> as; ///< The RoS actionserver
    al5d_low_level_driver::Al5dConfigurationFeedback feedback; ///< The feedback object (updated before each time feedback is published)
    al5d_low_level_driver::Al5dConfigurationResult result; ///< The result of the action to be published
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
    ros::ServiceServer emergencyService; ///< The ROS service server that handles incoming emergency requests
    std::string readFeedback; ///< The feedback that was read
    std::string queryMovementStatus; ///< SSC32U command to check if the robot is moving
    std::string emergencyStopCommand; ///< SSC32U command to immediately stop the robot
};

} // namespace RobotControl
} // namespace Al5dLowLevelDriver   

#endif //DRIVER_HPP_