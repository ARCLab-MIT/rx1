#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import zmq
from gazebo_msgs.msg import LinkStates

class ROSZMQBridge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ros_zmq_bridge', anonymous=True)
        
        # ZMQ setup for publishing original pose
        self.context = zmq.Context()
        self.pub_socket = self.context.socket(zmq.PUB)
        
        # ZMQ setup for receiving transformed pose
        self.sub_socket = self.context.socket(zmq.SUB)
        
        # Get parameters
        self.host = rospy.get_param('~host', '10.29.190.204')
        self.pub_port = rospy.get_param('~pub_port', 5555)
        self.sub_port = rospy.get_param('~sub_port', 9118)
        
        # Bind/connect sockets
        self.pub_socket.bind(f"tcp://{self.host}:{self.pub_port}")
        self.sub_socket.connect(f"tcp://{self.host}:{self.sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Subscribe to Gazebo link states
        self.link_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_state_callback)
        self.link_name = "rx1::right_forearm_roll_link"
        
        # Publisher for transformed poses to `/right_gripper_pose`
        self.right_gripper_pub = rospy.Publisher('/right_gripper_pose', Pose, queue_size=10)
        
        rospy.loginfo(f"ZMQ Bridge initialized - Publishing on {self.host}:{self.pub_port}")
        rospy.loginfo(f"Subscribing to transformed poses on {self.host}:{self.sub_port}")

    def link_state_callback(self, data):
        try:
            index = data.name.index(self.link_name)
            pose = data.pose[index]
            
            # Create a dictionary with position and orientation
            ee_state = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            
            # Send the data through ZMQ
            self.pub_socket.send_pyobj(ee_state)
            
            # Try to receive the transformed pose
            try:
                transformed_data = self.sub_socket.recv_pyobj(flags=zmq.NOBLOCK)
                rospy.loginfo("Received transformed pose:")
                rospy.loginfo(str(transformed_data))
                
                # Extract the pose data from the nested structure
                pose_data = transformed_data['data']
                
                # Create and publish a Pose message
                pose_msg = Pose()
                pose_msg.position.x = pose_data['position']['x']
                pose_msg.position.y = pose_data['position']['y']
                pose_msg.position.z = pose_data['position']['z']
                
                pose_msg.orientation.x = pose_data['orientation']['x']
                pose_msg.orientation.y = pose_data['orientation']['y']
                pose_msg.orientation.z = pose_data['orientation']['z']
                pose_msg.orientation.w = pose_data['orientation']['w']
                
                # Publish to `/right_gripper_pose`
                self.right_gripper_pub.publish(pose_msg)
                rospy.loginfo("Published transformed pose to `/right_gripper_pose`")

            except zmq.Again:
                pass  # No message available
            except Exception as e:
                rospy.logerr(f"Error receiving transformed pose: {e}")
            
        except ValueError:
            rospy.logwarn_throttle(1, f"Link {self.link_name} not found in /gazebo/link_states")
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

    def run(self):
        rospy.spin()

    def __del__(self):
        """Cleanup ZMQ context"""
        if hasattr(self, 'pub_socket'):
            self.pub_socket.close()
        if hasattr(self, 'sub_socket'):
            self.sub_socket.close()
        if hasattr(self, 'context'):
            self.context.term()

if __name__ == "__main__":
    try:
        bridge = ROSZMQBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
