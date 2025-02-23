#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import sys

class MoveItIKPublisher:
    def __init__(self):
        rospy.init_node('moveit_ik_publisher', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        
        # Increase planning time and tolerance for better success rate
        self.group.set_planning_time(0.5)
        self.group.set_goal_position_tolerance(0.05)
        self.group.set_goal_orientation_tolerance(0.05)
        
        # Subscribe to pose commands
        self.pose_sub = rospy.Subscriber(
            '/right_gripper_pose',
            PoseStamped,
            self.pose_callback
        )
        
        rospy.loginfo("MoveItIKPublisher initialized and waiting for poses...")

    def pose_callback(self, msg):
        try:
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo("Received new pose target")
            rospy.loginfo(f"Position: x={msg.pose.position.x:.3f}, "
                         f"y={msg.pose.position.y:.3f}, "
                         f"z={msg.pose.position.z:.3f}")
            
            # Set and execute pose target directly using the approach from test_ik_to_arm
            self.group.set_pose_target(msg)
            plan_success = self.group.go(wait=True)  # Plans and executes
            self.group.stop()  # Ensures no residual movement
            self.group.clear_pose_targets()  # Cleans up
            
            if plan_success:
                rospy.loginfo("Successfully moved to target pose")
            else:
                rospy.logwarn("Failed to move to target pose")
                
        except Exception as e:
            rospy.logerr(f"Error in pose callback: {str(e)}")

if __name__ == '__main__':
    try:
        ik_publisher = MoveItIKPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass