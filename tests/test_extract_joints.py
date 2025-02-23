#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

JOINTS_OF_INTEREST = [
    "right_shoul_base2shoul_joint",
    "right_shoul2shoul_rot_joint",
    "right_arm2armrot_joint",
    "right_armrot2elbow_joint",
    "right_forearm2forearmrot_joint",
    "right_forearmrot2forearm_pitch_joint",
    "right_forearm_pitch2forearm_roll_joint"
]

def joint_state_callback(msg):
    for joint_name in JOINTS_OF_INTEREST:
        if joint_name in msg.name:
            # Find the array index for our joint of interest
            idx = msg.name.index(joint_name)
            
            # Retrieve its position
            pos = msg.position[idx]
            
            # Print or do something with the position
            rospy.loginfo("Joint '%s' position: %f", joint_name, pos)

def main():
    rospy.init_node("single_joint_position_listener", anonymous=True)
    
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    
    # Keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    main()
