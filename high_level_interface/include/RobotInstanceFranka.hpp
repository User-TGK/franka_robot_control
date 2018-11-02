#ifndef ROBOT_INSTANCE_FRANKA_HPP_
#define ROBOT_INSTANCE_FRANKA_HPP_

#include <RobotInstance.hpp>

typedef actionlib::SimpleActionClient<franka_low_level_driver::FrankaConfigurationAction> FrankaConfigClient;

namespace RobotControl
{
namespace HighLevelInterface
{

class RobotInstanceFranka: public RobotInstance
{
public:
    /**
     * @brief Construct a new Robot Instance Franka object
     * 
     * @param name the topic the action server should listen to for incoming goals
     * @author Ties Klappe
     */
    explicit RobotInstanceFranka(const std::string& name);

    /**
     * @brief Destroy the Robot Instance Franka object
     * @author Ties Klappe
     */
    virtual ~RobotInstanceFranka();
private:

    /**
     * @brief function that will update the preprogrammed positions to the config file
     * @author Ties Klappe
     */
    void updatePreprogrammedPositions();

    /**
     * @brief function that will parse the preprogrammed positions in the config file
     * 
     * @return true if the config file was successfully parsed
     * @author Ties Klappe
     */
    bool parsePreprogrammedPositions();
    /**
     * @brief overriding the update manual function in RobotInstance
     * Function will update the current manual configuration to the current position
     * 
     * @return std::vector<high_level_interface::DoFConfiguration> the new manual position
     * @author Ties Klappe
     */
    std::vector<high_level_interface::DoFConfiguration> updateManualPosition(bool update);
    /**
     * @brief overriding the executeConfigurationAction function in RobotInstance
     * used as callback function for incoming configuration goals when the Franka is attached
     * @author Ties Klappe
     * @author Joris van Zeeland
     */
    void executeConfigurationAction();

    /**
     * @brief callback function for an incoming emergency stop request when the Franka is attached
     * 
     * @param request the request of the emergency stop service
     * @param response the response to the emergency stop request
     * @return true if the emergency stop was executed successfully 
     * @return false if the emergency stop was not executed successfully
     * @author Ties Klappe
     * @author Joris van Zeeland
     */
    bool handleEmergencyStopRequest(high_level_interface::EmergencyStopRequest& request, 
                                high_level_interface::EmergencyStopResponse& response);
    
    /**
     * @brief callback function for the gripper interface that overrides controlGripper in RobotInstance
     * 
     * @param request the request (see .srv file for description)
     * @param response the response (see .srv file for description)
     * @return true if the connection could be established
     * @return false if the connection could not be established
     * @author Ties Klappe
     */
    bool controlGripper(high_level_interface::GripperRequest& request,
                            high_level_interface::GripperResponse& response);
    /**
     * @brief overriding the toPreProgrammedPosition function in RobotInstance
     * function will be called when a preprogrammed position is requested and the Franka is attached
     * 
     * @param request the request that contains the position to move to
     * @param response the response that will tell if the functionality was called
     * @return true if a connection could be established with the robot
     * @return false if no connection could be established with the robot
     * @author Ties Klappe
     */
    bool toPreprogrammedPosition(high_level_interface::PositionRequest& request,
                                    high_level_interface::PositionResponse& response);
    /**
     * @brief function that will copy an franka driver configuration object to a high level configuration goal
     * 
     * @param goal the goal that will get origin's values assigned
     * @param origin the origin of the values the goal will get assigned
     * @author Ties Klappe
     */
    void copyDoFConfiguration(high_level_interface::DoFConfiguration& goal, const franka_low_level_driver::DoFConfiguration& origin);
    
    /**
     * @brief function that will copy a high level configuration object to a a franka configuration goal
     * 
     * @param goal the goal that will get origin's values assigned
     * @param origin the origin of the values the goal will get assigned
     * @author Ties Klappe
     */
    void copyDoFConfiguration(franka_low_level_driver::DoFConfiguration& goal, const high_level_interface::DoFConfiguration& origin);

    /**
     * @brief callback function that will be called when the franka configuration action is finished
     * (and the configuration action's result can be published)
     * 
     * @param state the state of the franka configuration goal when its finished
     * @param result the result that the franka configuration action published
     * @author Ties Klappe
     */
    void configurationResultCB(const actionlib::SimpleClientGoalState& state, const franka_low_level_driver::FrankaConfigurationResultConstPtr& result);
    
    /**
     * @brief callback function that will be called when the franka configuration publishes feedback
     * 
     * @param feedback the feedback that the franka configuration action published
     * @author Ties Klappe
     */
    void configurationFeedbackCB(const franka_low_level_driver::FrankaConfigurationFeedbackConstPtr& feedback);

    franka_low_level_driver::EmergencyStopFranka emergencyStopMsg;
    franka_low_level_driver::GripperFranka gripperMsg;
    FrankaConfigClient configurationClient;
    ros::ServiceClient gripperClient;
    std::map<PreProgrammedPosition, std::vector<franka_low_level_driver::DoFConfiguration>> preProgrammedConfigurations;
};

} //namespace RobotControl
} //namespace HighLevelInterface

#endif //ROBOT_INSTANCE_FRANKA_HPP_