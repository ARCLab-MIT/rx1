#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class IKToArmController:
    def __init__(self):
        rospy.init_node('ik_to_arm_controller', anonymous=True)

        # These must match your URDF + what your IK node publishes for the "right arm"
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]

        # Create a publisher to send joint trajectories to the right arm position controller
        self.right_arm_pub = rospy.Publisher(
            '/right_arm_position_controller/command',
            JointTrajectory,
            queue_size=10
        )

        # Subscribe to the IK solver output topic for the right arm
        # (This must match where your IK node is actually publishing the solved angles.)
        self.right_arm_ik_sub = rospy.Subscriber(
            '/right_arm_joint_states',  
            JointState,
            self.right_arm_ik_callback
        )

        rospy.loginfo("IKToArmController: Ready. Subscribing to /right_arm_joint_states.")

    def right_arm_ik_callback(self, msg):
        """
        Whenever new IK joint angles arrive on /right_arm_joint_states,
        convert them into a JointTrajectory and publish to the controller.
        """
        # We expect exactly the 7 right-arm joints we care about
        if not all(j in msg.name for j in self.right_arm_joints):
            rospy.logwarn("IKToArmController: Not all right-arm joints present. Ignoring this msg.")
            return

        # Build a JointTrajectory for the 7 right-arm joints
        traj = JointTrajectory()
        traj.joint_names = self.right_arm_joints

        point = JointTrajectoryPoint()
        # Extract positions in the same order as self.right_arm_joints
        point.positions = []
        for jn in self.right_arm_joints:
            idx = msg.name.index(jn)
            point.positions.append(msg.position[idx])

        # We can choose a small time_from_start so the controller knows how fast to move
        point.time_from_start = rospy.Duration(1.0)  # 1 second to move

        traj.points.append(point)

        # Publish the trajectory
        self.right_arm_pub.publish(traj)
        # rospy.loginfo("IKToArmController: Published new trajectory for right arm.")

    def spin(self):
        """Keep the node alive."""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = IKToArmController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
