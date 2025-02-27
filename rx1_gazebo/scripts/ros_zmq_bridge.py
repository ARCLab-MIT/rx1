#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import moveit_commander
import sys
import zmq
import cv2
import base64
import numpy as np
import pickle
import blosc as bl
import threading
from scipy.spatial.transform import Rotation as R

class ROSZMQBridge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ros_zmq_bridge', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        
        # Get parameters
        self.host = rospy.get_param('~host', '10.29.190.204')
        self.ee_pose_port = rospy.get_param('~ee_pose_port', 5555)
        self.joint_state_port = rospy.get_param('~joint_state_port', 5556)
        self.sub_port = rospy.get_param('~sub_port', 9118)
        
        # Initialize ZMQ publishers and subscribers
        self.ee_pose_pub = ZMQKeypointPublisher(self.host, self.ee_pose_port)
        self.joint_state_pub = ZMQKeypointPublisher(self.host, self.joint_state_port)
        self.transformed_pose_sub = ZMQKeypointSubscriber(self.host, self.sub_port, "pose")
        
        # Subscribe to joint states
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
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
        
        # End effector link name
        self.ee_link = "right_forearm_roll_link"
        
        # Publishers
        self.right_gripper_pub = rospy.Publisher('/right_gripper_pose', PoseStamped, queue_size=10)
        
        rospy.loginfo(f"ZMQ Bridge initialized - Publishing on {self.host}:{self.ee_pose_port} & {self.joint_state_port}")
        rospy.loginfo(f"Subscribing to transformed poses on {self.host}:{self.sub_port}")

    def joint_state_callback(self, data):
        try:
            # Process joint states
            joint_array = []
            for joint_name in self.right_arm_joints:
                if joint_name in data.name:
                    idx = data.name.index(joint_name)
                    joint_array.extend([
                        data.position[idx],
                        data.velocity[idx],
                        data.effort[idx] if len(data.effort) > idx else 0.0
                    ])
            
            if joint_array:  # Only publish if we have data
                self.joint_state_pub.pub_keypoints(joint_array, "joint_state")
                
                # Get end-effector pose using MoveIt
                current_pose = self.group.get_current_pose(self.ee_link)
                
                # Convert quaternion to axis-angle
                quat = [
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w
                ]
                r = R.from_quat(quat)
                rotvec = r.as_rotvec()
                
                # Create array with [x, y, z, rx, ry, rz]
                ee_array = [
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z,
                    rotvec[0],
                    rotvec[1],
                    rotvec[2]
                ]
                
                # Send the position and axis-angle through ZMQ
                self.ee_pose_pub.pub_keypoints(ee_array, "ee_pose")
                
                # Try to receive the transformed pose
                try:
                    transformed_data = self.transformed_pose_sub.recv_keypoints(flags=zmq.NOBLOCK)
                    if transformed_data is not None and len(transformed_data) >= 6:
                        # Convert back to quaternion for ROS message
                        position = transformed_data[0:3]
                        rotation = transformed_data[3:6]
                        
                        r_back = R.from_rotvec(rotation)
                        quat_back = r_back.as_quat()
                        
                        # Create and publish a PoseStamped message
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = "world"
                        pose_msg.header.stamp = rospy.Time.now()
                        
                        pose_msg.pose.position.x = position[0]
                        pose_msg.pose.position.y = position[1]
                        pose_msg.pose.position.z = position[2]
                        pose_msg.pose.orientation.x = quat_back[0]
                        pose_msg.pose.orientation.y = quat_back[1]
                        pose_msg.pose.orientation.z = quat_back[2]
                        pose_msg.pose.orientation.w = quat_back[3]
                        
                        self.right_gripper_pub.publish(pose_msg)
                
                except Exception as e:
                    rospy.logerr(f"Error receiving transformed pose: {e}")
                    
        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {e}")

    def __del__(self):
        """Cleanup ZMQ publishers and subscribers"""
        self.ee_pose_pub.stop()
        self.joint_state_pub.stop()
        self.transformed_pose_sub.stop()
        moveit_commander.roscpp_shutdown()

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
