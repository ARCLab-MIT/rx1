#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import euler_from_quaternion

class FingerPositionTracker:
    def __init__(self):
        rospy.init_node('finger_position_tracker')
        self.tf_listener = tf.TransformListener()

    def get_fingertip_pose(self):
        """
        Get the position and orientation of the right thumb fingertip in right_palm_lower frame
        Returns:
            tuple: (position [x,y,z], orientation [roll,pitch,yaw]) or None if transform not available
        """
        try:
            # Wait for transform to be available
            self.tf_listener.waitForTransform('/right_palm_lower', '/right_thumb_fingertip', 
                                           rospy.Time(0), rospy.Duration(1.0))
            
            # Get both translation and rotation
            (trans, rot) = self.tf_listener.lookupTransform('/right_palm_lower', 
                                                          '/right_thumb_fingertip', 
                                                          rospy.Time(0))
            
            # Convert quaternion to euler angles (in radians)
            euler = euler_from_quaternion(rot)
            
            return trans, euler

        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            result = self.get_fingertip_pose()
            if result:
                position, orientation = result
                rospy.loginfo(f"Thumb position [x,y,z]: {position}")
                rospy.loginfo(f"Thumb orientation [roll,pitch,yaw]: {orientation}")
            rate.sleep()

if __name__ == '__main__':
    try:
        tracker = FingerPositionTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass