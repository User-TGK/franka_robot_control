#include <actionlib/client/simple_action_client.h>
#include <Driver.hpp>

typedef actionlib::SimpleActionClient<al5d_low_level_driver::Al5dConfigurationAction> Client;

/**
 * @brief test program that creates a test HLI client to test the AL5D driver (This is an example)
 * @author Ties Klappe
 * 
 * @param argc -
 * @param argv -
 * @return int successfull executed (0) or not (other than 0)
 */
int main(int argc, char **argv) 
{
    const std::string nodeName = "AL5DDriver";
    ros::init(argc, argv, "AL5DDriverIntegrationTestclient");

    Client client(nodeName, true);
    client.waitForServer();

    /** Setting the Al5dConfiguration message */
    al5d_low_level_driver::Al5dConfigurationGoal goal;
    al5d_low_level_driver::DoFConfiguration configurationMessage;
    configurationMessage.channel = 0;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 1;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 2;
    configurationMessage.angle = 100;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 3;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 4;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);
    configurationMessage.channel = 5;
    configurationMessage.angle = 0;
    configurationMessage.speed = 65;
    goal.configuration.push_back(configurationMessage);

    // Sending the goal
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(6000));

    auto result = client.getResult();

    for (const auto& doFConfig: result->resultConfiguration)
    {
        ROS_DEBUG("CHANNEL: %s, ANGLE: %s", std::to_string(doFConfig.channel).c_str(), std::to_string(doFConfig.angle).c_str());
    }
    return 0;
}