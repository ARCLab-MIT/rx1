#!/usr/bin/env python3

import rospy
import tkinter as tk
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

class RightArmController:
    def __init__(self):
        rospy.init_node('test_right_arm_controller', anonymous=True)

        # Action client for the right arm controller
        self.client = actionlib.SimpleActionClient(
            '/right_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        # Wait for the action server to start
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server found!")

        # Joint names for the right arm (matching your URDF)
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]

        # Dictionary to store joint positions
        self.joint_positions = {joint: 0.0 for joint in self.right_arm_joints}

        # Initialize the GUI
        self.init_gui()

    def init_gui(self):
        """Initialize the Tkinter GUI with sliders for each joint."""
        self.root = tk.Tk()
        self.root.title("Right Arm Joint Controller")

        # Add joint limit labels
        limits = {
            'right_shoul_base2shoul_joint': (-2.5132, 2.5132),
            'right_shoul2shoul_rot_joint': (-1.57075, 0.50264),
            'right_arm2armrot_joint': (-1.57075, 1.57075),
            'right_armrot2elbow_joint': (-2.19905, 0),
            'right_forearm2forearmrot_joint': (-1.57075, 1.57075),
            'right_forearmrot2forearm_pitch_joint': (-0.785375, 0.785375),
            'right_forearm_pitch2forearm_roll_joint': (-0.785375, 0.785375)
        }

        row = 0
        for joint_name in self.right_arm_joints:
            # Joint name label
            label = tk.Label(self.root, text=joint_name)
            label.grid(row=row, column=0)

            # Slider with joint limits
            min_val, max_val = limits[joint_name]
            slider = tk.Scale(
                self.root,
                from_=min_val,
                to=max_val,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=300,
                command=lambda val, j=joint_name: self.update_joint_position(j, val)
            )
            slider.grid(row=row, column=1)
            
            # Limits label
            limits_label = tk.Label(self.root, text=f"[{min_val:.2f}, {max_val:.2f}]")
            limits_label.grid(row=row, column=2)
            
            row += 1

        # Add send command button
        send_button = tk.Button(
            self.root,
            text="Send Joint Commands",
            command=self.send_joint_commands
        )
        send_button.grid(row=row, column=0, columnspan=3, pady=10)

        # Add a reset button
        reset_button = tk.Button(
            self.root,
            text="Reset to Zero",
            command=self.reset_joints
        )
        reset_button.grid(row=row+1, column=0, columnspan=3, pady=5)

    def update_joint_position(self, joint_name, value):
        """Update joint position when slider moves."""
        self.joint_positions[joint_name] = float(value)

    def create_trajectory_goal(self):
        """Create a FollowJointTrajectoryGoal message from current slider positions."""
        goal = FollowJointTrajectoryGoal()
        
        # Create the trajectory
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "base_link"
        traj.joint_names = self.right_arm_joints

        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[joint] for joint in self.right_arm_joints]
        point.velocities = [0.0] * len(self.right_arm_joints)
        point.accelerations = [0.0] * len(self.right_arm_joints)
        point.time_from_start = rospy.Duration(1.0)  # 1 second movement

        traj.points.append(point)
        goal.trajectory = traj
        
        return goal

    def send_joint_commands(self):
        """Send the joint commands to the controller."""
        try:
            goal = self.create_trajectory_goal()
            self.client.send_goal(goal)
            rospy.loginfo("Joint commands sent to right arm controller")
            
            # Optional: Wait for the movement to complete
            self.client.wait_for_result(rospy.Duration(5.0))
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Movement completed successfully")
            else:
                rospy.logwarn("Movement did not complete in time")
                
        except Exception as e:
            rospy.logerr(f"Failed to send joint commands: {e}")

    def reset_joints(self):
        """Reset all joints to zero position."""
        for joint in self.right_arm_joints:
            self.joint_positions[joint] = 0.0
        self.send_joint_commands()
        rospy.loginfo("Reset all joints to zero")

    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()

if __name__ == '__main__':
    try:
        controller = RightArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass