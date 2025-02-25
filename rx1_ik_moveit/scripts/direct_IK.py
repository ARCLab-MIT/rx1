#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import sys

class DirectIKPublisher:
    def __init__(self):
        rospy.init_node('direct_ik_publisher', anonymous=True)
        
        # Subscribe to joint states first
        self.current_joint_states = None
        self.joint_states_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self.joint_states_callback,
            queue_size=1
        )
        
        # Wait for robot state with better error handling
        rospy.loginfo("Waiting for /robot/joint_states...")
        try:
            msg = rospy.wait_for_message("/robot/joint_states", JointState, timeout=10.0)
            rospy.loginfo(f"Received initial joint states: {msg.name}")
            self.current_joint_states = msg
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get joint states: {e}")
            rospy.logerr("Available topics:")
            for topic, msg_type in rospy.get_published_topics():
                rospy.logerr(f"  {topic}: {msg_type}")
            raise  # Re-raise the exception to stop initialization
        
        # Initialize MoveIt for IK only (no planning)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        
        # Disable all planning-related parameters
        self.group.set_planner_id("")
        self.group.set_planning_time(0)
        self.group.set_num_planning_attempts(1)
        self.group.allow_looking(False)
        self.group.allow_replanning(False)
        
        # Subscribe to pose commands
        self.pose_sub = rospy.Subscriber(
            '/right_gripper_pose',
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )
        
        # Publisher for joint trajectory
        self.joint_traj_pub = rospy.Publisher(
            '/right_arm_controller/command',
            JointTrajectory,
            queue_size=1
        )
        
        # Get joint names from the group
        self.joint_names = self.group.get_active_joints()
        rospy.loginfo(f"Controlling joints: {self.joint_names}")
        
        rospy.loginfo("DirectIKPublisher initialized")

    def joint_states_callback(self, msg):
        """Store the current joint states"""
        self.current_joint_states = msg
        # Debug joint states occasionally
        if rospy.Time.now().to_sec() % 5 < 0.1:  # Log every ~5 seconds
            rospy.loginfo(f"Current joint positions: {msg.position}")

    def pose_callback(self, msg):
        try:
            if self.current_joint_states is None:
                rospy.logwarn("No joint states received yet")
                return

            # Get IK solution directly
            joint_goal = self.group.get_current_joint_values()
            success = self.group.set_pose_target(msg)
            
            if success:
                # Create trajectory message
                traj = JointTrajectory()
                traj.joint_names = self.joint_names
                
                point = JointTrajectoryPoint()
                point.positions = joint_goal
                point.time_from_start = rospy.Duration(0.1)  # 100ms execution
                
                traj.points.append(point)
                
                # Publish trajectory
                self.joint_traj_pub.publish(traj)
                rospy.logdebug(f"Published trajectory for joints: {self.joint_names}")
            else:
                rospy.logwarn("IK solution not found")
                
        except Exception as e:
            rospy.logerr(f"Error in pose callback: {str(e)}")

if __name__ == '__main__':
    try:
        ik_publisher = DirectIKPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass