#!/usr/bin/env python3

import rospy
import tkinter as tk
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

class GripperPosePublisher:
    def __init__(self):
        rospy.init_node('test_gripper_pose_publisher', anonymous=True)
        
        # Publisher for the gripper pose
        self.pose_pub = rospy.Publisher('/right_gripper_pose', Pose, queue_size=10)
        
        # Initialize pose values
        self.pose = {
            'x': 0.5,    # Forward/back
            'y': -0.2,   # Left/right
            'z': 0.5,    # Up/down
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # Initialize GUI
        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Gripper Pose Publisher")
        
        # Position sliders
        tk.Label(self.root, text="Position (meters)").grid(row=0, column=0, columnspan=2)
        
        self.create_slider('x', 1, -1.0, 1.0, "Forward/Back")
        self.create_slider('y', 2, -1.0, 1.0, "Left/Right")
        self.create_slider('z', 3, 0.0, 1.5, "Up/Down")
        
        # Orientation sliders (in degrees for intuition)
        tk.Label(self.root, text="Orientation (degrees)").grid(row=4, column=0, columnspan=2)
        
        self.create_slider('roll', 5, -180, 180, "Roll")
        self.create_slider('pitch', 6, -180, 180, "Pitch")
        self.create_slider('yaw', 7, -180, 180, "Yaw")
        
        # Add publish rate slider
        tk.Label(self.root, text="Publish Rate (Hz)").grid(row=8, column=0)
        self.rate_slider = tk.Scale(self.root, from_=1, to=50, orient=tk.HORIZONTAL)
        self.rate_slider.set(10)  # Default to 10 Hz
        self.rate_slider.grid(row=8, column=1)
        
        # Add start/stop button
        self.running = False
        self.toggle_button = tk.Button(self.root, text="Start Publishing", command=self.toggle_publishing)
        self.toggle_button.grid(row=9, column=0, columnspan=2)
        
        # Add status label
        self.status_label = tk.Label(self.root, text="Status: Stopped")
        self.status_label.grid(row=10, column=0, columnspan=2)

    def create_slider(self, name, row, min_val, max_val, label_text):
        tk.Label(self.root, text=label_text).grid(row=row, column=0)
        slider = tk.Scale(
            self.root,
            from_=min_val,
            to=max_val,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            command=lambda val, n=name: self.update_pose(n, val)
        )
        slider.set(self.pose[name])
        slider.grid(row=row, column=1)

    def update_pose(self, name, value):
        self.pose[name] = float(value)

    def publish_pose(self):
        if not self.running:
            return
            
        pose_msg = Pose()
        
        # Set position
        pose_msg.position.x = self.pose['x']
        pose_msg.position.y = self.pose['y']
        pose_msg.position.z = self.pose['z']
        
        # Convert degrees to radians for orientation
        roll_rad = math.radians(self.pose['roll'])
        pitch_rad = math.radians(self.pose['pitch'])
        yaw_rad = math.radians(self.pose['yaw'])
        
        # Convert Euler angles to quaternion
        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
        
        # Publish the pose
        self.pose_pub.publish(pose_msg)
        
        # Schedule next publication
        rate = self.rate_slider.get()
        self.root.after(int(1000/rate), self.publish_pose)

    def toggle_publishing(self):
        self.running = not self.running
        if self.running:
            self.toggle_button.config(text="Stop Publishing")
            self.status_label.config(text="Status: Running")
            self.publish_pose()
        else:
            self.toggle_button.config(text="Start Publishing")
            self.status_label.config(text="Status: Stopped")

    def run(self):
        # Start the GUI event loop
        self.root.mainloop()

if __name__ == '__main__':
    try:
        publisher = GripperPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass