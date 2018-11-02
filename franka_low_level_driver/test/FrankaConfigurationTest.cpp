#include <actionlib/client/simple_action_client.h>
#include <Driver.hpp>

typedef actionlib::SimpleActionClient<franka_low_level_driver::FrankaConfigurationAction> Client;

/**
 * @brief test program that creates a test HLI client to test the Franka driver (This is an example)
 * @author Ties Klappe
 * 
 * @param argc -
 * @param argv -
 * @return int successfull executed (0) or not (other than 0)
 */
int main(int argc, char **argv) 
{
    const std::string nodeName = "/FrankaDriver";
    ros::init(argc, argv, "FrankaDriverIntegrationTestclient");

    Client client(nodeName, true);
    client.waitForServer();
    ROS_INFO("Server available");

    /** Setting the FrankaConfiguration message */
    franka_low_level_driver::FrankaConfigurationGoal goal;
    franka_low_level_driver::DoFConfiguration configurationMessage;

    goal.speedFactor = 0.3;

    configurationMessage.channel = 0;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 1;
    configurationMessage.angle = -45;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 2;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 3;
    configurationMessage.angle = -165;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 4;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 5;
    configurationMessage.angle = 100;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 6;
    configurationMessage.angle = 45;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);

    // Sending the goal
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(6));

    auto result = client.getResult();

    for (const auto& doFConfig: result->resultConfiguration)
    {
        ROS_DEBUG("CHANNEL: %s, ANGLE: %s", std::to_string(doFConfig.channel).c_str(), std::to_string(doFConfig.angle).c_str());
    }
    ROS_INFO("FINISHED");
    return 0;
}