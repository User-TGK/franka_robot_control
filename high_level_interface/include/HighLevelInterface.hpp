#ifndef HIGH_LEVEL_INTERFACE_HPP_
#define HIGH_LEVEL_INTERFACE_HPP_

#include <memory>

#include <ros/ros.h>

#include <high_level_interface/UpdateManual.h>
#include <franka_low_level_driver/UnlockFranka.h>
#include <RobotInstance.hpp>

namespace RobotControl
{
namespace HighLevelInterface
{

class HighLevelInterface
{
public:
    /**
     * @brief Construct a new High Level Interface object
     * 
     * @param configTopicName the action topic -name configuration goals will be send to
     * @param robotType the robot type of the attached robot
     * @param emergencyStopName the topic the emergency service will be available on
     * @author Ties Klappe
     */
    HighLevelInterface(const std::string& configTopicName, RobotType robotType, const std::string& emergencyStopName);
    
    /**
     * @brief Destroy the High Level Interface object
     * @author Ties Klappe
     */
    virtual ~HighLevelInterface();
private:
    /**
     * @brief callback function that will move a robot to a preprogrammed position
     * 
     * @param request the request that contains to which preprogrammed position the robot should move
     * @param response the response that will tell if the robot started moving to the requested position
     * @return true if the system was able to start moving to the position
     * @return false if the system couldn't call the functionality that will start the robot
     * @author Ties Klappe
     */
    bool preProgrammedPositionCB(high_level_interface::PositionRequest& request,
                                    high_level_interface::PositionResponse& response);

    /**
     * @brief callback function for incoming emergency -stop requests
     * 
     * @param request the request of the emergency stop service
     * @param response the response of the emergency stop service
     * @return true if the emergency stop was successfully executed 
     * @return false if the emergency stop was not successfully executed
     * @author Ties Klappe
     * @author Joris van Zeeland
     */
    bool handleEmergencyStopCB(high_level_interface::EmergencyStopRequest& request, 
                                    high_level_interface::EmergencyStopResponse& response);
    
    /**
     * @brief callback function that will unlock the robot interface after an emergency stop was called
     * 
     * @param request the request of the unlock service
     * @param response the response of the unlock service
     * @return true if the system was able to execute the request
     * @return false if the system wasn't able to execute the request
     * @author Ties Klappe
     */
    bool handleUnlockCB(high_level_interface::UnlockRequest& request,
                                    high_level_interface::UnlockResponse& response);
    
    /**
     * @brief callback function that will be called when a gripper control request is received
     * 
     * @param request the request of the grippers new state (see .srv file for description)
     * @param response the response to the gripper request (see .srv file for description)
     * @return true if the connection with one of the drivers could be stablished
     * @return false if no connection with one of the drivers could be established
     * @author Ties Klappe
     */
    bool handleGripper(high_level_interface::GripperRequest& request,
                                    high_level_interface::GripperResponse& response);
    
    /**
     * @brief callback function that will update the manual position to the current position
     * 
     * @param request the request of the new manualpos (see .srv file for description)
     * @param response the response
     * @return true if the service could be called
     * @author Ties Klappe
     */
    bool updateManual(high_level_interface::UpdateManualRequest& request,
                        high_level_interface::UpdateManualResponse& response);
    /**
     * @brief Set the Robot object with the factory pattern
     * 
     * @param configTopicName the topic the actionserver should listen to for new configuration actions
     * @param robotType the type of the attached robot
     * @author Ties Klappe
     * @author Joris van Zeeland
     */
    void setRobot(const std::string& configTopicName, RobotType robotType);

    ros::NodeHandle nh; ///< The nodehandle instance
    std::vector<std::shared_ptr<RobotInstance>> robotInstances; ///< Smart pointers to the attached robots
    ros::ServiceServer emergencyStopHandler; ///< The emergency stop service -handler
    ros::ServiceServer preprogrammedPositionHandler; ///< The preprogrammed position service -handler
    ros::ServiceServer unlockEmergencyStopHandler; ///< The emergencystop unlock handler
    ros::ServiceServer gripperHandler;
    ros::ServiceServer updateManualHandler;
    franka_low_level_driver::UnlockFranka frankaUnlockMsg;
};

} //namespace RobotControl
} //namespace HighLevelInterface

#endif //HIGH_LEVEL_INTERFACE_HPP_