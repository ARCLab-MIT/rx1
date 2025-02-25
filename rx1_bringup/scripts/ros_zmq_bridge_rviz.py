#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose  # Changed from PoseStamped to Pose
from sensor_msgs.msg import JointState
import zmq
import pickle
import threading
from scipy.spatial.transform import Rotation as R
import cv2
import base64
import numpy as np
import blosc as bl
import tf2_ros

class ROSZMQBridge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ros_zmq_bridge', anonymous=True)
        
        # Get parameters
        self.host = rospy.get_param('~host', '10.29.190.204')
        self.right_ee_port_pub = rospy.get_param('~right_ee_port_pub', 5555)
        self.right_joint_state_port_pub = rospy.get_param('~right_joint_state_port_pub', 5556)
        self.right_ee_port_sub = rospy.get_param('~right_ee_port_sub', 9118)
        
        # Initialize ZMQ publishers and subscribers
        self.right_ee_pub = ZMQKeypointPublisher(self.host, self.right_ee_port_pub) # Send current pose
        self.right_arm_joint_states_pub = ZMQKeypointPublisher(self.host, self.right_joint_state_port_pub) # Send joint states
        self.right_ee_sub = ZMQKeypointSubscriber(self.host, self.right_ee_port_sub, "pose") # Receive transformed pose
        
        # Subscribe to right arm joint states instead of all joint states
        self.joint_sub = rospy.Subscriber('/right_arm_joint_states', JointState, self.joint_state_callback)
        
        # Right arm joints
        self.right_arm_joints = [
            "right_shoul_base2shoul_joint",
            "right_shoul2shoul_rot_joint",
            "right_arm2armrot_joint",
            "right_armrot2elbow_joint",
            "right_forearm2forearmrot_joint",
            "right_forearmrot2forearm_pitch_joint",
            "right_forearm_pitch2forearm_roll_joint"
        ]
        
        # Publishers
        self.right_gripper_pub = rospy.Publisher('/right_gripper_pose', Pose, queue_size=10)
        
        # Setup TF2 listener for getting link positions
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo(f"ZMQ Bridge initialized - Publishing on {self.host}:{self.right_ee_port_pub} & {self.right_joint_state_port_pub}")
        rospy.loginfo(f"Subscribing to transformed poses on {self.host}:{self.right_ee_port_sub}")
        rospy.loginfo("Subscribing to right arm joint states from /right_arm_joint_states")

    def joint_state_callback(self, data):
        try:
            # Process joint states - only extract positions
            joint_array = []
            for joint_name in self.right_arm_joints:
                if joint_name in data.name:
                    idx = data.name.index(joint_name)
                    # Add position only
                    joint_array.append(data.position[idx])
            
            if joint_array:  # Only publish if we have data
                self.right_arm_joint_states_pub.pub_keypoints(joint_array, "joint_state")
                
                try:
                    trans = self.tf_buffer.lookup_transform('head_base_link', 'right_forearm_roll_link', rospy.Time())
                    
                    # Extract position and orientation
                    position = [
                        trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z
                    ]
                    
                    # Convert quaternion to rotation vector for ZMQ transmission
                    quat = [
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ]
                    r = R.from_quat(quat)
                    rotvec = r.as_rotvec()
                    
                    # Combine position and rotation vector
                    ee_pose = position + rotvec.tolist()
                    
                    # Publish to ZMQ
                    self.right_ee_pub.pub_keypoints(ee_pose, "ee_pose")
                    
                    # Try to receive the transformed pose
                    try:
                        transformed_data = self.right_ee_sub.recv_keypoints(flags=zmq.NOBLOCK)
                        
                        # Handle dictionary format (what we're actually receiving)
                        if transformed_data is not None and isinstance(transformed_data, dict):
                            if 'position' in transformed_data and 'orientation' in transformed_data:
                                # Create and publish a Pose message
                                pose_msg = Pose()
                                
                                # Set position
                                pose_msg.position.x = transformed_data['position'][0]
                                pose_msg.position.y = transformed_data['position'][1]
                                pose_msg.position.z = transformed_data['position'][2]
                                
                                # Set orientation
                                pose_msg.orientation.x = transformed_data['orientation'][0]
                                pose_msg.orientation.y = transformed_data['orientation'][1]
                                pose_msg.orientation.z = transformed_data['orientation'][2]
                                pose_msg.orientation.w = transformed_data['orientation'][3]
                                
                                # Publish the pose
                                self.right_gripper_pub.publish(pose_msg)
                                # rospy.loginfo("Published dictionary pose to /right_gripper_pose")
                        
                        # Handle array format (original expected format)
                        elif transformed_data is not None and len(transformed_data) >= 6:
                            # Convert to ROS message
                            position = transformed_data[0:3]
                            rotation = transformed_data[3:6]
                            r_back = R.from_rotvec(rotation)
                            quat_back = r_back.as_quat()
                            
                            # Create and publish a Pose message
                            pose_msg = Pose()
                            
                            pose_msg.position.x = position[0]
                            pose_msg.position.y = position[1]
                            pose_msg.position.z = position[2]
                            pose_msg.orientation.x = quat_back[0]
                            pose_msg.orientation.y = quat_back[1]
                            pose_msg.orientation.z = quat_back[2]
                            pose_msg.orientation.w = quat_back[3]
                            
                            self.right_gripper_pub.publish(pose_msg)
                            rospy.loginfo("Published array pose to /right_gripper_pose")
                    
                    except Exception as e:
                        rospy.logerr(f"Error receiving transformed pose: {e}")
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF Error: {e}")
                    
        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {e}")

    def __del__(self):
        """Cleanup ZMQ publishers and subscribers"""
        self.right_ee_pub.stop()
        self.right_arm_joint_states_pub.stop()
        self.right_ee_sub.stop()

    def run(self):
        """Keep the node running"""
        rospy.spin()

# ZMQ Sockets
def create_push_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_pull_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_response_socket(host, port):
    content = zmq.Context()
    socket = content.socket(zmq.REP)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_request_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://{}:{}'.format(host, port))
    return socket

# Pub/Sub classes for Keypoints
class ZMQKeypointPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def pub_keypoints(self, keypoint_array, topic_name):
        """
        Process the keypoints into a byte stream and input them in this function
        """
        buffer = pickle.dumps(keypoint_array, protocol = -1)
        self.socket.send(bytes('{} '.format(topic_name), 'utf-8') + buffer)

    def stop(self):
        print('Closing the publisher socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQKeypointSubscriber(threading.Thread):
    def __init__(self, host, port, topic):
        self._host, self._port, self._topic = host, port, topic
        self._init_subscriber()

        # Topic chars to remove
        self.strip_value = bytes("{} ".format(self._topic), 'utf-8')

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self._topic, 'utf-8'))

    def recv_keypoints(self, flags=None):
        if flags is None:
            raw_data = self.socket.recv()
            raw_array = raw_data.lstrip(self.strip_value)
            return pickle.loads(raw_array)
        else: # For possible usage of no blocking zmq subscriber
            try:
                raw_data = self.socket.recv(flags)
                raw_array = raw_data.lstrip(self.strip_value)
                return pickle.loads(raw_array)
            except zmq.Again:
                # print('zmq again error')
                return None
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

# Pub/Sub classes for storing data from Realsense Cameras
class ZMQCameraPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        print('tcp://{}:{}'.format(self._host, self._port))
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))


    def pub_intrinsics(self, array):
        self.socket.send(b"intrinsics " + pickle.dumps(array, protocol = -1))

    def pub_rgb_image(self, rgb_image, timestamp):
        _, buffer = cv2.imencode('.jpg', rgb_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = dict(
            timestamp = timestamp,
            rgb_image = base64.b64encode(buffer)
        )
        self.socket.send(b"rgb_image " + pickle.dumps(data, protocol = -1))

    def pub_depth_image(self, depth_image, timestamp):
        compressed_depth = bl.pack_array(depth_image, cname = 'zstd', clevel = 1, shuffle = bl.NOSHUFFLE)
        data = dict(
            timestamp = timestamp,
            depth_image = compressed_depth
        )
        self.socket.send(b"depth_image " + pickle.dumps(data, protocol = -1))

    def stop(self):
        print('Closing the publisher socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQCameraSubscriber(threading.Thread):
    def __init__(self, host, port, topic_type):
        self._host, self._port, self._topic_type = host, port, topic_type
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        print('tcp://{}:{}'.format(self._host, self._port))
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))

        if self._topic_type == 'Intrinsics':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"intrinsics")
        elif self._topic_type == 'RGB':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")
        elif self._topic_type == 'Depth':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"depth_image")

    def recv_intrinsics(self):
        raw_data = self.socket.recv()
        raw_array = raw_data.lstrip(b"intrinsics ")
        return pickle.loads(raw_array)

    def recv_rgb_image(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
        encoded_data = np.fromstring(base64.b64decode(data['rgb_image']), np.uint8)
        return cv2.imdecode(encoded_data, 1), data['timestamp']
        
    def recv_depth_image(self):
        raw_data = self.socket.recv()
        striped_data = raw_data.lstrip(b"depth_image ")
        data = pickle.loads(striped_data)
        depth_image = bl.unpack_array(data['depth_image'])
        return np.array(depth_image, dtype = np.int16), data['timestamp']
        
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

# Publisher for image visualizers
class ZMQCompressedImageTransmitter(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_push_socket()
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def _init_push_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def send_image(self, rgb_image):
        _, buffer = cv2.imencode('.jpg', rgb_image, [int(cv2.IMWRITE_WEBP_QUALITY), 10])
        self.socket.send(np.array(buffer).tobytes())

    def stop(self):
        print('Closing the publisher in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQCompressedImageReciever(threading.Thread):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_pull_socket()
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.subscribe("")

    def _init_pull_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))

    def recv_image(self):
        raw_data = self.socket.recv()
        encoded_data = np.fromstring(raw_data, np.uint8)
        decoded_frame = cv2.imdecode(encoded_data, 1)
        return decoded_frame
        
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQButtonFeedbackSubscriber(threading.Thread):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_pull_socket()
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.subscribe("")

    def _init_pull_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))


    def recv_keypoints(self):
        raw_data = self.socket.recv()
        return pickle.loads(raw_data)
    
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

if __name__ == "__main__":
    try:
        bridge = ROSZMQBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
