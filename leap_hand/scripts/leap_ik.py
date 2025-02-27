#!/usr/bin/env python3

import rospy
import moveit_commander
import tf
from moveit_commander.move_group import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import JointState
import sys

class IKSolver:
    def __init__(self):
        # Initialize moveit_commander and tf
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ik_solver', anonymous=True)
        self.tf_listener = tf.TransformListener()

        # Initialize both robot and move group
        self.robot = moveit_commander.RobotCommander()
        self.group = MoveGroupCommander("right_thumb")
        
        # Get the joint names from the group
        self.joint_names = self.group.get_active_joints()
        # rospy.loginfo(f"MoveIt joint names: {self.joint_names}")
        
        # Subscribe to joint states
        self.current_joints = None
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        
        # Wait for first joint state message
        rospy.loginfo("Waiting for joint states...")
        while self.current_joints is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Set the reference frame for planning
        self.group.set_pose_reference_frame("right_palm_lower")
        
        # Get frame information
        self.planning_frame = self.group.get_planning_frame()
        self.end_effector_link = self.group.get_end_effector_link()
        self.reference_frame = self.group.get_pose_reference_frame()
        
        rospy.loginfo(f"Planning frame: {self.planning_frame}")
        rospy.loginfo(f"End effector link: {self.end_effector_link}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        rospy.loginfo(f"Robot base frame: {self.robot.get_root_link()}")

    def joint_callback(self, msg):
        """Callback for joint states"""
        # Get thumb joint indices
        thumb_indices = [i for i, name in enumerate(msg.name) 
                        if name.startswith('right_thumb_joint')]
        
        if thumb_indices:
            self.current_joints = [msg.position[i] for i in thumb_indices]
            # rospy.loginfo(f"Current thumb joint names: {[msg.name[i] for i in thumb_indices]}")
            # rospy.loginfo(f"Current thumb joint values: {self.current_joints}")

    def get_current_fingertip_pose(self):
        """Get current fingertip pose from tf"""
        try:
            self.tf_listener.waitForTransform('/right_palm_lower', '/right_thumb_fingertip', 
                                           rospy.Time(0), rospy.Duration(1.0))
            
            (trans, rot) = self.tf_listener.lookupTransform('/right_palm_lower', 
                                                          '/right_thumb_fingertip', 
                                                          rospy.Time(0))
            
            euler = euler_from_quaternion(rot)
            # rospy.loginfo(f"Current fingertip position from tf: {trans}")
            # rospy.loginfo(f"Current fingertip orientation from tf: {euler}")
            return trans, euler

        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None

    def get_ik_solution(self, position, orientation):
        """
        Get IK solution for a given pose using MoveGroupCommander
        Args:
            position: [x, y, z] relative to right_palm_lower
            orientation: [roll, pitch, yaw] in radians relative to right_palm_lower
        Returns:
            list: Joint angles if solution found, None otherwise
        """
        # First, set the start state to current
        self.group.set_start_state_to_current_state()
        
        # Create PoseStamped message
        pose_target = PoseStamped()
        pose_target.header.frame_id = "right_palm_lower"
        pose_target.header.stamp = rospy.Time.now()

        # Set position
        pose_target.pose.position.x = position[0]
        pose_target.pose.position.y = position[1]
        pose_target.pose.position.z = position[2]

        # Convert euler to quaternion and set orientation
        quat = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3]

        try:
            # Set the pose target
            self.group.set_pose_target(pose_target)
            
            # Get the current joint values
            # rospy.loginfo(f"MoveIt current joint values: {self.group.get_current_joint_values()}")
            # rospy.loginfo(f"Actual current joint values: {self.current_joints}")
            
            # Compute IK
            plan = self.group.plan()
            success = plan[0]
            
            if success:
                solution = self.group.get_current_joint_values()
                return solution
            else:
                rospy.logwarn("Planning failed")
                return None

        finally:
            self.group.clear_pose_targets()

if __name__ == '__main__':
    try:
        solver = IKSolver()
        
        # Get current fingertip pose
        result = solver.get_current_fingertip_pose()
        if result:
            position, orientation = result
            # Try to find IK solution for current pose
            position = position + [0.01, 0.01, 0.01]
            solution = solver.get_ik_solution(position, orientation)
            if solution:
                rospy.loginfo(f"IK solution found: {solution}")
            else:
                rospy.logwarn("No IK solution found")
        
    except rospy.ROSInterruptException:
        pass