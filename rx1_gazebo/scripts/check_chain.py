#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from collections import defaultdict
import numpy as np

class JointStateAnalyzer:
    def __init__(self):
        # Initialize controller groups based on the controller_manager info
        self.controller_groups = {
            'right_arm': [
                'right_arm2armrot_joint',
                'right_armrot2elbow_joint',
                'right_forearm2forearmrot_joint',
                'right_forearm_pitch2forearm_roll_joint',
                'right_forearmrot2forearm_pitch_joint',
                'right_shoul2shoul_rot_joint',
                'right_shoul_base2shoul_joint'
            ],
            'left_arm': [
                'left_arm2armrot_joint',
                'left_armrot2elbow_joint',
                'left_forearm2forearmrot_joint',
                'left_forearm_pitch2forearm_roll_joint',
                'left_forearmrot2forearm_pitch_joint',
                'left_shoul2shoul_rot_joint',
                'left_shoul_base2shoul_joint'
            ],
            'torso': [
                'base2torso_yaw_joint',
                'torso_pitch2roll_joint',
                'torso_yaw2pitch_joint'
            ],
            'head': [
                'head_base2neck_yaw_joint',
                'neck_pitch2head_depth_cam_mount_joint',
                'neck_yaw2pitch_joint'
            ]
        }

        # Data storage for analysis
        self.joint_data = defaultdict(list)
        self.start_time = None

        # Initialize ROS node and subscriber
        rospy.init_node('joint_state_analyzer', anonymous=True)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp

        # Store joint data
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.joint_data[name].append({
                'time': (msg.header.stamp - self.start_time).to_sec(),
                'position': pos,
                'velocity': vel,
                'effort': eff
            })

    def analyze_group(self, group_name):
        """Analyze joint data for a specific controller group"""
        if not self.joint_data:
            return

        print(f"\nAnalysis for {group_name} group:")
        print("-" * 50)

        for joint in self.controller_groups[group_name]:
            if joint not in self.joint_data:
                continue

            data = self.joint_data[joint]
            positions = [d['position'] for d in data]
            velocities = [d['velocity'] for d in data]
            efforts = [d['effort'] for d in data]

            print(f"\nJoint: {joint}")
            print(f"Position range: {min(positions):.4f} to {max(positions):.4f}")
            print(f"Velocity range: {min(velocities):.4f} to {max(velocities):.4f}")
            print(f"Effort range: {min(efforts):.4f} to {max(efforts):.4f}")

    def run_analysis(self):
        """Run analysis for all controller groups"""
        rate = rospy.Rate(1)  # 1 Hz
        
        # Collect data for 5 seconds
        rospy.sleep(5.0)

        # Analyze each group
        for group in self.controller_groups:
            self.analyze_group(group)

if __name__ == '__main__':
    try:
        analyzer = JointStateAnalyzer()
        analyzer.run_analysis()
    except rospy.ROSInterruptException:
        pass