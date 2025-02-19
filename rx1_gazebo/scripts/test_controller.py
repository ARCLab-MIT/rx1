#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def test_moveit_ik():
    # Initialize
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_test', anonymous=True)
    
    # Setup move group
    arm_group = moveit_commander.MoveGroupCommander("right_arm")
    
    # Print useful info
    print("Reference frame:", arm_group.get_planning_frame())
    print("End effector link:", arm_group.get_end_effector_link())
    print("Current pose:", arm_group.get_current_pose())
    
    # Set target pose
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.003
    pose_target.position.y = -0.218
    pose_target.position.z = 0.283
    pose_target.orientation.x = -0.023
    pose_target.orientation.y = 0.703
    pose_target.orientation.z = -0.028
    pose_target.orientation.w = 0.709
    
    arm_group.set_pose_target(pose_target)
    
    # Plan and execute
    plan = arm_group.go(wait=True)
    
    # Cleanup
    arm_group.stop()
    arm_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_moveit_ik()
    except rospy.ROSInterruptException:
        pass