#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class IKTester:
    def __init__(self):
        rospy.init_node('ik_test_node', anonymous=True)
        
        # Define the right arm joint names (should match your URDF)
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]
        
        # Store last received joint values
        self.last_joints = None
        self.received_solution = False
        self.last_pose = None
        
        # Debugging
        self.debug = rospy.get_param('~debug', False)
        
        # Subscriber for joint states (to receive IK solutions)
        self.joint_sub = rospy.Subscriber('/right_arm_joint_states', JointState, self.joint_callback)
        
        # Subscribe to right gripper pose
        self.pose_sub = rospy.Subscriber('/right_gripper_pose', Pose, self.pose_callback)
        
        rospy.loginfo("[IK TEST] Node started. Waiting for poses from /right_gripper_pose...")
        
    def pose_callback(self, msg):
        """Handle incoming right gripper poses"""
        self.last_pose = msg
        self.received_solution = False  # Reset flag
        
        rospy.loginfo("\n[IK TEST] Received new target pose:")
        rospy.loginfo(f"Position: x={msg.position.x:.3f}, y={msg.position.y:.3f}, z={msg.position.z:.3f}")
        rospy.loginfo(f"Orientation: x={msg.orientation.x:.3f}, y={msg.orientation.y:.3f}, "
                      f"z={msg.orientation.z:.3f}, w={msg.orientation.w:.3f}")

    def joint_callback(self, msg):
        """Handle joint state updates (IK solutions)"""
        if self.last_pose is None or self.received_solution:
            return  # Ignore if no pose received yet or solution already checked
            
        right_arm_positions = []
        try:
            for joint_name in self.right_arm_joints:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    right_arm_positions.append(msg.position[index])
                else:
                    rospy.logwarn(f"[IK TEST] Joint {joint_name} not found in joint state message")
                    return
            
            # If we have a new solution, print it
            if not self.received_solution:
                rospy.loginfo("\n[IK TEST] Potential IK solution found:")
                for name, pos in zip(self.right_arm_joints, right_arm_positions):
                    rospy.loginfo(f"{name}: {pos:.4f}")
            
            # Compare with last joint state if available
            if self.last_joints is not None:
                joint_changes = [abs(new - old) for new, old in zip(right_arm_positions, self.last_joints)]
                rospy.loginfo("\n[IK TEST] Joint Changes:")
                for name, change in zip(self.right_arm_joints, joint_changes):
                    rospy.loginfo(f"{name} changed by {change:.4f}")

            self.last_joints = right_arm_positions
            self.received_solution = True
            
        except Exception as e:
            rospy.logwarn(f"[IK TEST] Error processing joint states: {e}")

def main():
    tester = IKTester()
    rate = rospy.Rate(10)  # 10 Hz
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[IK TEST] Shutting down IK tester node")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
