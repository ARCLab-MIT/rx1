#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates

def link_state_callback(data):
    link_name = "rx1::right_forearm_roll_link"  # Replace with your robot's link name
    try:
        index = data.name.index(link_name)
        position = data.pose[index].position
        orientation = data.pose[index].orientation
        print(f"Position: {position}, Orientation: {orientation}")
    except ValueError:
        rospy.logwarn("Link not found in /gazebo/link_states")

def get_link_position():
    rospy.init_node('get_link_position', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_state_callback)
    rospy.spin()

if __name__ == '__main__':
    get_link_position()
