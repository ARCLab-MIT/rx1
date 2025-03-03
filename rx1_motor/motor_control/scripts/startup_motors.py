#!/usr/bin/env python

import sys
import os
import yaml
from typing import List, Dict
import time

# Add the required directories to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
motor_control_dir = os.path.dirname(current_dir)  # Get motor_control directory
sys.path.insert(0, motor_control_dir)  # Add motor_control directory to path first

if current_dir not in sys.path:
    sys.path.append(current_dir)

from scripts.write import WriteMotors
from scripts.read import ReadMotors
from scripts.wheel import STServoWheel, control_loop
from stservo_sdk.protocol_packet_handler import COMM_SUCCESS

class StartupMotors:
    def __init__(self, config_path: str = None):
        """Initialize the startup sequence handler."""
        if config_path is None:
            # Use absolute path to configs/motors.yaml
            config_path = os.path.join(motor_control_dir, '..', 'configs', 'motors.yaml')
        
        self.right_elbow_id = 14
        self.left_elbow_id = 24

        # Load configuration
        self.config = self._load_config(config_path)
        
        # Initialize motor controllers
        self.writer = WriteMotors(device_name=self.config['servo_port'])
        self.reader = ReadMotors(device_name=self.config['servo_port'])
        
        # Position tolerance for verification
        self.position_tolerance = 5  # Acceptable difference between target and actual position

        self.wheel = STServoWheel(servo_id=14, moving_acc=15)

    def _load_config(self, config_path: str) -> Dict:
        """Load motor configuration from YAML file."""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise Exception(f"Failed to load config file: {e}")

    def initialize_right_arm(self) -> bool:
        """Home right arm motors to center position (2048), excluding elbow."""
        print("Initializing right arm...")
        
        # Get all right arm IDs except elbow
        right_arm_ids = [id for id in self.writer.right_arm_ids if id != self.right_elbow_id]
        
        # All non-elbow motors to 2048
        center_positions = [2048] * len(right_arm_ids)
        print(f"Moving right arm (except elbow) to center positions: {center_positions}")
        
        # Move non-elbow motors to center
        success = self.writer.write_arm_positions(
            right_arm_ids,
            center_positions
        )

        if not success:
            print("Failed to send center positions to right arm motors")
            return False

        time.sleep(2)  # Wait for motors to reach position
        return True
        
    def initialize_left_arm(self) -> bool:
        """Home left arm motors to center position (2048), excluding elbow."""
        print("Initializing left arm...")
        
        # Get all left arm IDs except elbow
        left_arm_ids = [id for id in self.writer.left_arm_ids if id != self.left_elbow_id]
        
        # All non-elbow motors to 2048
        center_positions = [2048] * len(left_arm_ids)
        print(f"Moving left arm (except elbow) to center positions: {center_positions}")
        
        # Move non-elbow motors to center
        success = self.writer.write_arm_positions(
            left_arm_ids,
            center_positions
        )
        
        if not success:
            print("Failed to send center positions to left arm motors")
            return False
        
        time.sleep(2)  # Wait for motors to reach position
        return True

    def initialize_all_motors(self) -> bool:
        """Initialize all motors to their starting positions."""
        right_success = self.initialize_right_arm()
        left_success = self.initialize_left_arm()

        # Call the standalone control_loop function
        # control_loop()

        return right_success and left_success

    def close(self):
        """Clean up resources."""
        self.writer.close()
        self.reader.close()

def main():
    """Example usage of the StartupMotors class."""
    startup = None
    try:
        startup = StartupMotors()
        success = startup.initialize_all_motors()
        
        if success:
            print("All motors successfully initialized")
        else:
            print("Failed to initialize all motors")
            
    except Exception as e:
        print(f"Error during startup: {e}")
    finally:
        if startup is not None:
            startup.close()

if __name__ == "__main__":
    main()
