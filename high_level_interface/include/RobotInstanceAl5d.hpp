#ifndef ROBOT_INSTANCE_AL5D_HPP_
#define ROBOT_INSTANCE_AL5D_HPP_

#include <RobotInstance.hpp>

typedef actionlib::SimpleActionClient<al5d_low_level_driver::Al5dConfigurationAction> Al5dConfigClient;

namespace RobotControl
{
namespace HighLevelInterface
{

class RobotInstanceAl5d: public RobotInstance
{
public:
    /**
     * @brief Construct a new Robot Instance Al5d object
     * 
     * @param name the name of the topic the actionserver should listen to for new goals
     * @author Ties Klappe
     */
    explicit RobotInstanceAl5d(const std::string& name);

    /**
     * @brief Destroy the Robot Instance Al 5d object
     * @author Ties Klappe
     */
    virtual ~RobotInstanceAl5d();

    /**
     * @brief overriding the executeConfigurationAction function in RobotInstance
     * used as callback function for incoming configuration goals when the AL5D is attached
     * @author Ties Klappe
     * @author Joris van Zeeland
     */
    void executeConfigurationAction();
private:
    /**
     * @brief overriding the updateManualPosition function in RobotInstance
     * function will be called when the manual position is updated
     * 
     * @return std::vector<high_level_interface::DoFConfiguration> the new manual position
     * @author Ties Klappe
     */
    std::vector<high_level_interface::DoFConfiguration> updateManualPosition(bool update);
    /**
     * @brief overriding the toPreProgrammedPosition function in RobotInstance
     * function will be called when a preprogrammed position is requested and the AL5D is attached
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
     * @brief callback function for an incoming emergency stop request when the AL5D is attached
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
     * @brief function that will copy an al5d driver configuration object to a high level configuration goal
     * 
     * @param goal the goal that will get origin's values assigned
     * @param origin the origin of the values the goal will get assigned
     * @author Ties Klappe
     */
    void copyDoFConfiguration(high_level_interface::DoFConfiguration& goal, const al5d_low_level_driver::DoFConfiguration& origin);
    
    /**
     * @brief function that will copy a high level configuration object to a an al5d configuration goal
     * 
     * @param goal the goal that will get origin's values assigned
     * @param origin the origin of the values the goal will get assigned
     * @author Ties Klappe
     */
    void copyDoFConfiguration(al5d_low_level_driver::DoFConfiguration& goal, const high_level_interface::DoFConfiguration& origin);

    /**
     * @brief callback function that will be called when the al5d configuration action is finished
     * (and the configuration action's result can be published)
     * 
     * @param state the state of the al5d configuration goal when its finished
     * @param result the result that the al5d configuration action published
     * @author Ties Klappe
     */
    void configurationResultCB(const actionlib::SimpleClientGoalState& state, const al5d_low_level_driver::Al5dConfigurationResultConstPtr& result);
    
    /**
     * @brief callback function that will be called when the al5d configuration publishes feedback
     * 
     * @param feedback the feedback that the al5d configuration action published
     * @author Ties Klappe
     */
    void configurationFeedbackCB(const al5d_low_level_driver::Al5dConfigurationFeedbackConstPtr& feedback);
    
    al5d_low_level_driver::EmergencyStopAl5d emergencyStopMsg;
    Al5dConfigClient configurationClient;
    const unsigned long nrOfJoints = 6;
    std::vector<al5d_low_level_driver::DoFConfiguration> manualConfiguration;
};

} //namespace RobotControl
} //namespace HighLevelInterface

#endif //ROBOT_INSTANCE_AL5D_HPP_