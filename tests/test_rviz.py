#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

def publish_test_poses():
    rospy.init_node('test_poses', anonymous=True)
    pose_pub = rospy.Publisher('/right_gripper_pose', Pose, queue_size=1)
    
    # Wait for publisher to be ready
    rospy.sleep(0.1)
    
    # Create base pose from the given position
    base_pose = Pose()
    # Position
    base_pose.position.x = 0.269751
    base_pose.position.y = -0.314831
    base_pose.position.z = -0.316261
    # Orientation
    base_pose.orientation.x = 0.003374
    base_pose.orientation.y = -0.707466
    base_pose.orientation.z = 0.027737
    base_pose.orientation.w = 0.706195
    
    # Test movements relative to base pose
    test_poses = [
        {'x': 0.0, 'y': 0.0, 'z': 0.0},      # Base position
        {'x': 0.05, 'y': 0.0, 'z': 0.0},     # Forward 5cm
        {'x': 0.0, 'y': 0.05, 'z': 0.0},     # Right 5cm
        {'x': 0.0, 'y': 0.0, 'z': 0.05},     # Up 5cm
        {'x': -0.05, 'y': 0.0, 'z': 0.0},    # Back 5cm
        {'x': 0.0, 'y': -0.05, 'z': 0.0},    # Left 5cm
        {'x': 0.0, 'y': 0.0, 'z': -0.05},    # Down 5cm
    ]
    
    rate = rospy.Rate(10)  # 2 seconds between poses
    
    while not rospy.is_shutdown():
        for i, offset in enumerate(test_poses):
            target_pose = Pose()
            # Apply offset to base position
            target_pose.position.x = base_pose.position.x + offset['x']
            target_pose.position.y = base_pose.position.y + offset['y']
            target_pose.position.z = base_pose.position.z + offset['z']
            # Keep the same orientation
            target_pose.orientation = base_pose.orientation
            
            rospy.loginfo(f"\nPublishing pose {i+1}:")
            rospy.loginfo(f"Position: x={target_pose.position.x:.3f}, "
                         f"y={target_pose.position.y:.3f}, "
                         f"z={target_pose.position.z:.3f}")
            
            pose_pub.publish(target_pose)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_poses()
    except rospy.ROSInterruptException:
        pass