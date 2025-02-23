#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys

def test_ik_poses():
    rospy.init_node('test_ik_poses', anonymous=True)
    
    # Initialize MoveIt just to get current pose
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    
    # Create publisher for poses
    pose_pub = rospy.Publisher('/right_gripper_pose', PoseStamped, queue_size=1)
    
    # Get current pose and workspace details
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Current pose: \n{current_pose}")
    
    # Increase planning time and tolerance
    group.set_planning_time(10.0)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.1)
    
    rate = rospy.Rate(0.2)  # 5 seconds between poses

    # Test poses - larger movements to see clear motion
    test_poses = [
        # Current position (baseline)
        {'pos': [current_pose.position.x, 
                 current_pose.position.y, 
                 current_pose.position.z],
         'orient': [current_pose.orientation.x,
                   current_pose.orientation.y,
                   current_pose.orientation.z,
                   current_pose.orientation.w]},
        
        # Move 5cm in X (forward)
        {'pos': [current_pose.position.x + 0.05, 
                 current_pose.position.y, 
                 current_pose.position.z],
         'orient': [current_pose.orientation.x,
                   current_pose.orientation.y,
                   current_pose.orientation.z,
                   current_pose.orientation.w]},
        
        # Move 5cm in Y (side)
        {'pos': [current_pose.position.x, 
                 current_pose.position.y - 0.05, 
                 current_pose.position.z],
         'orient': [current_pose.orientation.x,
                   current_pose.orientation.y,
                   current_pose.orientation.z,
                   current_pose.orientation.w]},
        
        # Move 5cm in Z (up)
        {'pos': [current_pose.position.x, 
                 current_pose.position.y, 
                 current_pose.position.z + 0.05],
         'orient': [current_pose.orientation.x,
                   current_pose.orientation.y,
                   current_pose.orientation.z,
                   current_pose.orientation.w]},
    ]

    current_pose_idx = 0
    while not rospy.is_shutdown():
        # Get current test pose
        test_pose = test_poses[current_pose_idx]
        
        # Create pose target
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        target_pose.pose.position.x = test_pose['pos'][0]
        target_pose.pose.position.y = test_pose['pos'][1]
        target_pose.pose.position.z = test_pose['pos'][2]
        
        # Set orientation
        target_pose.pose.orientation.x = test_pose['orient'][0]
        target_pose.pose.orientation.y = test_pose['orient'][1]
        target_pose.pose.orientation.z = test_pose['orient'][2]
        target_pose.pose.orientation.w = test_pose['orient'][3]

        rospy.loginfo("\n" + "="*50)
        rospy.loginfo(f"Publishing pose {current_pose_idx + 1}/{len(test_poses)}")
        rospy.loginfo(f"Position: x={target_pose.pose.position.x:.3f}, y={target_pose.pose.position.y:.3f}, z={target_pose.pose.position.z:.3f}")
        
        # Publish the pose instead of executing it
        pose_pub.publish(target_pose)
        
        # Move to next pose
        current_pose_idx = (current_pose_idx + 1) % len(test_poses)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        test_ik_poses()
    except rospy.ROSInterruptException:
        pass