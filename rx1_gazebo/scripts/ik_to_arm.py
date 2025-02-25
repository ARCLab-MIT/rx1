#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import numpy as np

class IKToArmController:
    def __init__(self):
        rospy.init_node('ik_to_arm_controller', anonymous=True)

        # These must match your URDF + what your IK node publishes
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]

        # Create publishers
        self.right_arm_pub = rospy.Publisher(
            '/right_arm_position_controller/command',
            JointTrajectory,
            queue_size=1
        )
        
        # Subscribe to the IK solver output
        self.joint_state_sub = rospy.Subscriber(
            '/right_arm_joint_states',  # This matches your IK node's output
            JointState,
            self.joint_state_callback,
            queue_size=1
        )

        # Store joint states for trajectory generation
        self.current_joint_positions = None
        self.target_joint_positions = None
        
        # Trajectory parameters
        self.num_points = 5  # Number of points in trajectory
        self.total_time = 0.1  # Total time for trajectory execution
        
        # Performance monitoring
        self.timing_pub = rospy.Publisher('/controller_timing_metrics', String, queue_size=10)
        self.timing_stats = {
            'count': 0,
            'total_time': 0,
            'min_time': float('inf'),
            'max_time': 0
        }
        
        rospy.loginfo("IKToArmController: Ready. Waiting for joint states from IK solver")

    def joint_state_callback(self, msg):
        """Handle joint states from IK solver and generate trajectory"""
        try:
            start_time = rospy.Time.now()

            # Verify we have all needed joints
            if not all(j in msg.name for j in self.right_arm_joints):
                rospy.logwarn("Not all right-arm joints present in IK solution")
                return

            # Get target joint positions
            target_positions = [msg.position[msg.name.index(jn)] for jn in self.right_arm_joints]

            # If this is our first message, initialize current positions
            if self.current_joint_positions is None:
                self.current_joint_positions = target_positions
                return

            # Generate trajectory
            traj = JointTrajectory()
            traj.joint_names = self.right_arm_joints

            # Create trajectory points
            for i in range(self.num_points):
                point = JointTrajectoryPoint()
                fraction = i / (self.num_points - 1)
                
                # Linear interpolation between current and target positions
                point.positions = [
                    current + fraction * (target - current)
                    for current, target in zip(self.current_joint_positions, target_positions)
                ]
                
                # Set timing
                point.time_from_start = rospy.Duration(fraction * self.total_time)
                
                # Add velocities (optional)
                if i > 0 and i < self.num_points - 1:
                    point.velocities = [0.0] * len(self.right_arm_joints)
                
                traj.points.append(point)

            # Update current positions
            self.current_joint_positions = target_positions

            # Publish trajectory
            self.right_arm_pub.publish(traj)

            # Update performance metrics
            execution_time = (rospy.Time.now() - start_time).to_sec()
            self.timing_stats['count'] += 1
            self.timing_stats['total_time'] += execution_time
            self.timing_stats['min_time'] = min(self.timing_stats['min_time'], execution_time)
            self.timing_stats['max_time'] = max(self.timing_stats['max_time'], execution_time)

            # Log slow executions
            if execution_time > 0.05:  # More than 50ms is considered slow
                rospy.logwarn(f"Slow trajectory generation: {execution_time:.3f}s")

            # Publish metrics periodically
            if self.timing_stats['count'] % 100 == 0:
                avg_time = self.timing_stats['total_time'] / self.timing_stats['count']
                metrics_msg = (
                    f"Controller Metrics:\n"
                    f"Last execution: {execution_time:.3f}s\n"
                    f"Average: {avg_time:.3f}s\n"
                    f"Min: {self.timing_stats['min_time']:.3f}s\n"
                    f"Max: {self.timing_stats['max_time']:.3f}s\n"
                    f"Count: {self.timing_stats['count']}"
                )
                self.timing_pub.publish(metrics_msg)

        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {str(e)}")

    def spin(self):
        """Keep the node alive."""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = IKToArmController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
