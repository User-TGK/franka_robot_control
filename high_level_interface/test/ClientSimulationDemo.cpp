#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <high_level_interface/ConfigurationAction.h>
#include <high_level_interface/EmergencyStop.h>
#include <high_level_interface/Unlock.h>
#include <high_level_interface/Position.h>

/**
 * @brief this function will test all actions and services provided by the HLI.
 * 
 * @pre start the driver of either the AL5D or the Franka with the simulation attached.
 * @param argc def.
 * @param argv def.
 * @return int def.
 * @author Ties Klappe
 */
int main (int argc, char** argv)
{
    ros::init(argc, argv, "HighLevelClientTest");

    ros::NodeHandle nh;

    ros::ServiceClient parkPositionClient = nh.serviceClient<high_level_interface::Position>("robot_position");
    high_level_interface::Position positionService;
    positionService.request.position = "PARK";

    if(parkPositionClient.call(positionService))
    {
        ROS_INFO("SUCCESSFULLY MOVING TO PARK POSITION.");
    }

    ros::Duration(2.0).sleep();

    ros::ServiceClient emergencyStopServiceClient = nh.serviceClient<high_level_interface::EmergencyStop>("emergency_stop");
    
    high_level_interface::EmergencyStop stopService;
    stopService.request.stop = true;

    actionlib::SimpleActionClient<high_level_interface::ConfigurationAction> ac("high_level_interface", true);
    ac.waitForServer();

    high_level_interface::ConfigurationGoal goal;
    high_level_interface::DoFConfiguration configurationMessage;
    goal.speedFactor = 0.3;
    configurationMessage.channel = 0;
    configurationMessage.angle = -15.936809;
    //configurationMessage.speed = 0;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 1;
    configurationMessage.angle = -4.186951;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 2;
    configurationMessage.angle = 8.800835   ;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 3;
    configurationMessage.angle = -118.619221;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 4;
    configurationMessage.angle = -0.991262;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 5;
    configurationMessage.angle = 118.797055;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 6;
    configurationMessage.angle = 45;
    //configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    ROS_INFO("Random pose.");

    ac.sendGoal(goal);
    ros::Duration(3.5).sleep();
    if(emergencyStopServiceClient.call(stopService))
    {
        ROS_INFO("EMERGENCY STOP EXECUTED.");
    }
    ros::Duration(1.0).sleep();

    ros::ServiceClient unlockClient = nh.serviceClient<high_level_interface::Unlock>("unlock_robot_driver");
    high_level_interface::Unlock unlockService;
    unlockService.request.unlock = true;
    
    if(unlockClient.call(unlockService))
    {
        ROS_INFO("SUCCESSFULLY UNLOCKED.");
    }

     positionService.request.position = "STRAIGHT";

    if(parkPositionClient.call(positionService))
    {
        ROS_INFO("SUCCESSFULLY MOVING TO STRAIGHT POSITION.");
    }

    ac.waitForResult(ros::Duration(5.0));
    return 0;
}