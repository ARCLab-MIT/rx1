#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys
from collections import deque
import numpy as np
from threading import Lock

class JointStateController:
    def __init__(self):
        rospy.init_node('joint_state_controller', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        
        # Optimize MoveIt settings for real-time control
        self.group.set_planning_time(0.05)  # Minimal planning time
        self.group.set_num_planning_attempts(1)  # Single planning attempt
        self.group.set_goal_position_tolerance(0.01)  # 1cm tolerance
        self.group.set_goal_orientation_tolerance(0.1)  # Larger orientation tolerance
        self.group.set_max_velocity_scaling_factor(0.3)  # 30% of max speed
        self.group.set_max_acceleration_scaling_factor(0.3)  # 30% of max acceleration
        
        # Buffer for pose interpolation
        self.pose_buffer = deque(maxlen=3)  # Keep last 3 poses for interpolation
        self.buffer_lock = Lock()
        self.last_execution_time = rospy.Time.now()
        
        # Control rate
        self.control_rate = 30.0  # Hz
        self.rate = rospy.Rate(self.control_rate)
        
        # Move to home position
        rospy.loginfo("Moving to home position...")
        self.group.set_named_target("home")
        self.group.go(wait=False)
        self.group.stop()
        self.group.clear_pose_targets()
        
        # Subscribe to target pose
        self.pose_sub = rospy.Subscriber(
            '/right_gripper_pose',
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )
        
        # Start control loop in a separate thread
        rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_loop)
        
        rospy.loginfo("Joint State Controller initialized and ready for poses")

    def interpolate_poses(self, start_pose, end_pose, factor):
        """Linear interpolation between two poses"""
        interp_pose = PoseStamped()
        interp_pose.header = end_pose.header
        
        # Interpolate position
        interp_pose.pose.position.x = (1 - factor) * start_pose.pose.position.x + factor * end_pose.pose.position.x
        interp_pose.pose.position.y = (1 - factor) * start_pose.pose.position.y + factor * end_pose.pose.position.y
        interp_pose.pose.position.z = (1 - factor) * start_pose.pose.position.z + factor * end_pose.pose.position.z
        
        # Keep the latest orientation (for simplicity and stability)
        interp_pose.pose.orientation = end_pose.pose.orientation
        
        return interp_pose

    def pose_callback(self, msg):
        """Callback for new pose targets"""
        with self.buffer_lock:
            self.pose_buffer.append(msg)

    def control_loop(self, event):
        """Main control loop for executing poses"""
        try:
            with self.buffer_lock:
                if not self.pose_buffer:
                    return
                
                target_pose = self.pose_buffer[-1]  # Get latest pose
                self.pose_buffer.clear()  # Clear buffer after getting pose
            
            # Set the pose target without planning
            self.group.set_pose_target(target_pose.pose)
            
            # Execute the motion
            self.group.go(wait=False)  # Non-blocking execution
            self.group.clear_pose_targets()
            
        except Exception as e:
            rospy.logerr(f"Error in control loop: {str(e)}")
            self.group.stop()
            self.group.clear_pose_targets()

if __name__ == '__main__':
    try:
        controller = JointStateController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass