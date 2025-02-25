#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys

def test_ik_poses():
    rospy.init_node('test_ik_poses', anonymous=True)
    
    # Initialize MoveIt for getting current pose
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    
    # Create publisher for poses
    pose_pub = rospy.Publisher('/right_gripper_pose', PoseStamped, queue_size=1)
    
    # Wait for publisher to be ready
    rospy.sleep(1.0)
    
    # Move to home position first
    rospy.loginfo("Moving to home position...")
    group.set_named_target("home")
    plan_success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    if plan_success:
        rospy.loginfo("Successfully moved to home position")
    else:
        rospy.logwarn("Failed to move to home position")
        return
    
    # Get current pose
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Current pose: \n{current_pose}")
    
    # Configuration
    STEP_SIZE = 0.05  # 2cm steps
    RATE = 5  # Hz (5 seconds between movements)
    rate = rospy.Rate(RATE)
    start_time = rospy.Time.now()
    duration = rospy.Duration(10)  # Run for 10 seconds

    # More extensive movement sequence
    movements = [
        # Forward and back
        {'axis': 'x', 'distance': 0.010},   # Move 10cm forward
        {'axis': 'x', 'distance': -0.010},  # Move back
        
        # Left and right
        {'axis': 'y', 'distance': 0.010},   # Move 10cm right
        {'axis': 'y', 'distance': -0.010},  # Move left
        
        # Up and down
        {'axis': 'z', 'distance': 0.010},   # Move 10cm up
        {'axis': 'z', 'distance': -0.010},  # Move down
        
        # Diagonal movements
        {'axis': 'x', 'distance': 0.07},   # Forward
        {'axis': 'y', 'distance': 0.07},   # Right
        {'axis': 'x', 'distance': -0.07},  # Back
        {'axis': 'y', 'distance': -0.07},  # Left
        
        # Square movement
        {'axis': 'x', 'distance': 0.05},   # Forward
        {'axis': 'y', 'distance': 0.05},   # Right
        {'axis': 'x', 'distance': -0.05},  # Back
        {'axis': 'y', 'distance': -0.05},  # Left
        
        # Vertical square
        {'axis': 'z', 'distance': 0.05},   # Up
        {'axis': 'x', 'distance': 0.05},   # Forward
        {'axis': 'z', 'distance': -0.05},  # Down
        {'axis': 'x', 'distance': -0.05},  # Back
    ]

    rospy.loginfo("Starting incremental movements...")
    current_movement = 0
    
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < duration:
        movement = movements[current_movement % len(movements)]
        axis = movement['axis']
        total_distance = movement['distance']
        num_steps = int(abs(total_distance / STEP_SIZE))
        direction = 1 if total_distance > 0 else -1
        
        rospy.loginfo(f"\nStarting {abs(total_distance*100):.1f}cm movement in {axis} direction")
        
        for step in range(num_steps):
            if (rospy.Time.now() - start_time) >= duration:
                break
                
            # Get current pose for relative movement
            current_pose = group.get_current_pose()
            
            # Create new target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "world"
            target_pose.header.stamp = rospy.Time.now()
            
            # Copy current pose
            target_pose.pose = current_pose.pose
            
            # Increment the specified axis
            if axis == 'x':
                target_pose.pose.position.x += direction * STEP_SIZE
            elif axis == 'y':
                target_pose.pose.position.y += direction * STEP_SIZE
            elif axis == 'z':
                target_pose.pose.position.z += direction * STEP_SIZE
            
            # Log the movement
            rospy.loginfo(f"Step {step + 1}/{num_steps} along {axis}")
            rospy.loginfo(f"Target position: x={target_pose.pose.position.x:.3f}, "
                         f"y={target_pose.pose.position.y:.3f}, "
                         f"z={target_pose.pose.position.z:.3f}")
            
            # Publish the pose
            pose_pub.publish(target_pose)
            
            # Wait for movement to complete
            rate.sleep()
            
        current_movement += 1
    
    rospy.loginfo("Movement sequence completed! Returning to home position...")
    
    # Return to home position
    group.set_named_target("home")
    plan_success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    if plan_success:
        rospy.loginfo("Successfully returned to home position")
    else:
        rospy.logwarn("Failed to return to home position")

if __name__ == '__main__':
    try:
        test_ik_poses()
    except rospy.ROSInterruptException:
        pass