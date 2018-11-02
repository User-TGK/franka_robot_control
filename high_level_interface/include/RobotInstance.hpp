#ifndef ROBOT_INSTANCE_HPP_
#define ROBOT_INSTANCE_HPP_

#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <high_level_interface/EmergencyStop.h>
#include <high_level_interface/Unlock.h>
#include <high_level_interface/Position.h>
#include <high_level_interface/Gripper.h>
#include <high_level_interface/ConfigurationAction.h>
#include <al5d_low_level_driver/EmergencyStopAl5d.h>
#include <al5d_low_level_driver/Al5dConfigurationAction.h>
#include <franka_low_level_driver/FrankaConfigurationAction.h>
#include <franka_low_level_driver/EmergencyStopFranka.h>
#include <franka_low_level_driver/GripperFranka.h>

namespace RobotControl
{
namespace HighLevelInterface
{

/**
 * @brief Enum class that contains all robot types
 * @author Ties Klappe
 * @author Joris van Zeeland
 */
enum class RobotType
{
    Al5d,
    Franka,
    FrankaSimulation,
    FrankaCombined
};

/**
 * @brief the supported preprogrammed positions
 * @author Ties Klappe
 */
enum class PreProgrammedPosition
{
    Park,
    Straight,
    Manual
};

/**
 * @brief the states the robot interface can be in
 * @author Ties Klappe
 */
enum class RobotState
{
    Active,
    Stopping,
    Moving,
    Locked
};

class RobotInstance
{
public:
    /**
     * @brief Construct a new Robot Instance object
     * 
     * @param aName the name the configuration actionserver of the robot instance should listen to
     * @author Ties Klappe
     */
    explicit RobotInstance(const std::string& aName, const RobotType aRobotType);

    /**
     * @brief Destroy the Robot Instance object
     * @author Ties Klappe
     */
    virtual ~RobotInstance();

    /**
     * @brief pure virtual function that is the callback function for incoming configuration goals
     * @author Ties Klappe
     */
    virtual void executeConfigurationAction() = 0;

    /**
     * @brief pure virtual function that will be called when the preprogrammed position's service is called
     * 
     * @param request the request that contains the position to move to
     * @param response the response that will tell if the functionality was called
     * @return true if a connection could be established with the robot
     * @return false if no connection could be established with the robot
     * @author Ties Klappe
     */
    virtual bool toPreprogrammedPosition(high_level_interface::PositionRequest& request,
                                    high_level_interface::PositionResponse& response) = 0;

    /**
     * @brief pure virtual function that will handle an incoming emergency stop request by passing
     * 
     * @param request the request of the emergency stop service
     * @param response the response to the emergency stop service request
     * @return true if the emergency stop service request was successfully executed
     * @return false if the emergency stop service request was not successfully executed
     * @author Ties Klappe
     */
    virtual bool handleEmergencyStopRequest(high_level_interface::EmergencyStopRequest& request, 
                                    high_level_interface::EmergencyStopResponse& response) = 0;

    /**
     * @brief pure virtual function that will be used to control the gripper
     * 
     * @param request the request of the gripper (open or close)
     * @param response the response (see .srv file for description)
     * @return true if the connection could be established
     * @return false if the connection could not be established
     * @author Ties Klappe
     */
    virtual bool controlGripper(high_level_interface::GripperRequest& request,
                                high_level_interface::GripperResponse& response) = 0;

    /**
     * @brief pure virtual function to update the manual position
     * 
     * @return std::vector<high_level_interface::DoFConfiguration> the new manual configuration
     * @author Ties Klappe
     */
    virtual std::vector<high_level_interface::DoFConfiguration> updateManualPosition(bool update) = 0;
    /**
     * @brief Set the Current State object
     * 
     * @param aCurrentState the state to move the robot interface to
     * @author Ties Klappe
     */
    void setCurrentState(const RobotState& aCurrentState);

    /**
     * @brief Get the Current State object
     * 
     * @return RobotState the current state
     * @author Ties Klappe
     */
    RobotState getCurrentState() const;

    RobotType getType() const;
protected:
    /**
     * @brief function that will convert a string from the Position service to a preprogrammedposition object
     * 
     * @param str the std::string to be converted
     * @return PreProgrammedPosition the preprogrammed position object
     * @exception std::invalid_argument if the string does not have a preprogrammed position representation
     * @author Ties Klappe
     */
    PreProgrammedPosition preProgrammedPositionFromString(const std::string& str) const;

    /**
     * @brief function that will return the string representation of a preprogrammed position object
     * 
     * @param preProgrammedPosition the preprogrammed position instance
     * @return std::string the string representation of the preprogrammed position
     * @exception std::invalid_argument if the preprogrammed position does not have a string representation
     * @author Ties Klappe
     */
    std::string strFromPreprogrammedPosition(const PreProgrammedPosition& preProgrammedPosition) const;

    /**
     * @brief unordered map that contains the string representation of each preprogrammed position.
     * Each object in the PreProgrammedPosition object should contain a string representation
     */
    const std::unordered_map<std::string, PreProgrammedPosition> PositionTypeMap =
    {
        {"PARK", PreProgrammedPosition::Park},
        {"STRAIGHT", PreProgrammedPosition::Straight},
        {"MANUAL", PreProgrammedPosition::Manual}
    };

    ros::NodeHandle nh;
    ros::ServiceClient emergencyStopClient;
    actionlib::SimpleActionServer<high_level_interface::ConfigurationAction> as;
    high_level_interface::ConfigurationFeedback configurationFeedback;
    high_level_interface::ConfigurationResult configurationResult;
    RobotState currentState;
    RobotType robotType;
};

} //namespace RobotControl
} //namespace HighLevelInterface

#endif //ROBOT_INSTANCE_HPP_