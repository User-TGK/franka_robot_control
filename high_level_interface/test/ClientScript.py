#! /usr/bin/python
from __future__ import print_function

## Test script to easily test RobotControl without having to build
## Author: Ties Klappe
## Usage: (in a sourced terminal) run: 
## ** rosrun high_level_interface ClientScript.py <DOF0Config> <DOF1Config> <DOF2Config> <DOF3Config> <DOF4Config> <DOF5Config> <DOF6Config>

from high_level_interface.msg import DoFConfiguration
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import high_level_interface.msg

def high_level_client(argv):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('high_level_interface', high_level_interface.msg.ConfigurationAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    config = list()
    
    for i in range (0, 7):
        dof_config = DoFConfiguration()
        dof_config.channel = i
        dof_config.angle = float(argv[i+1])
        config.append(dof_config)

    print("Goal:")
    for goal_config in config:
        print ("Goal for DOF ", goal_config.channel, "is:", goal_config.angle)
    goal = high_level_interface.msg.ConfigurationGoal(configuration = config, speedFactor = 0.3)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    if len(sys.argv) != 8:
        print("len", len(sys.argv))
        print("Usage: rosrun high_level_interface ClientScript.py <DOF 0 Config> <DOF 1 Config> <DOF 2 Config> <DOF 3 Config> <DOF 4 Config> <DOF 5 Config> <DOF 6 Config>")
        sys.exit()
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('high_level_interface_client_py')
        result = high_level_client(sys.argv).resultConfiguration
        print("\nResult: ")
        for dof_config in result:
            print("DoF ", dof_config.channel, ": ", dof_config.angle, " degrees")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)