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

from write import WriteMotors
from read import ReadMotors
from wheel import STServoWheel  # If you need wheel mode, otherwise can remove
from stservo_sdk.protocol_packet_handler import COMM_SUCCESS

class StartupMotors:
    """
    Example that shows how to incorporate a 1:3 gear ratio for elbow joints.
    Even though we still command "center = 2048" for all joints, we define
    raw <-> gear angle conversion so you can easily adapt the code for a smaller
    elbow range in the future.
    """

    def __init__(self, config_path: str = None):
        """Initialize the startup sequence handler."""
        if config_path is None:
            # Use absolute path to configs/motors.yaml
            config_path = os.path.join(motor_control_dir, '..', 'configs', 'motors.yaml')
        
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Initialize motor controllers
        self.writer = WriteMotors(device_name=self.config['servo_port'])
        self.reader = ReadMotors(device_name=self.config['servo_port'])
        
        # Position tolerance for verification
        self.position_tolerance = 10  # Acceptable difference in raw ticks

        # IDs for elbow joints
        self.right_elbow_id = 14
        self.left_elbow_id  = 24

        # Define gear ratio for elbows: servo rotates 3x for 1x at the elbow gear
        # => gearAngle = servoAngle / 3
        # We'll treat raw=2048 as gearAngle=0°, so offset around 2048
        self.raw_center = 2048
        self.deg_per_raw = 360.0 / 4096.0  # servo's raw -> degrees if no gear
        self.gear_ratio = 1.0 / 3.0        # elbow moves 1 deg if servo moves 3 deg

    def _load_config(self, config_path: str) -> Dict:
        """Load motor configuration from YAML file."""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise Exception(f"Failed to load config file: {e}")

    def close(self):
        """Clean up resources."""
        self.writer.close()
        self.reader.close()

    # ------------------------------------------------------------------------
    # Gear Ratio Conversion Functions
    # ------------------------------------------------------------------------
    def raw_to_gear_angle(self, raw: int) -> float:
        """
        Convert servo raw [0..4095] to elbow gear angle (degrees).
        For a 1:3 ratio, gearAngle = servoAngle/3.
        Here we treat raw=2048 => servoAngle=0 => gearAngle=0.
        """
        servo_angle = (raw - self.raw_center) * self.deg_per_raw  # servo angle in range ~ -180..+180
        gear_angle  = servo_angle * self.gear_ratio               # elbow sees 1/3 that angle
        return gear_angle

    def gear_angle_to_raw(self, gear_angle: float) -> int:
        """
        Convert elbow gear angle (degrees) back to servo raw [0..4095].
        """
        servo_angle = gear_angle / self.gear_ratio
        raw = int(round(servo_angle / self.deg_per_raw + self.raw_center))
        # Wrap or clamp if needed
        if raw < 0:    raw += 4096
        if raw > 4095: raw -= 4096
        return max(0, min(4095, raw))

    def clamp_gear_angle(self, angle: float, min_deg: float = -60, max_deg: float = 60) -> float:
        """
        If your elbow physically only allows ~120°,
        you can clamp it here to avoid commanding beyond safe range.
        """
        return max(min_deg, min(max_deg, angle))

    # ------------------------------------------------------------------------
    # Homing Routines
    # ------------------------------------------------------------------------
    def initialize_right_arm(self) -> bool:
        """
        Home right arm motors to "center" (raw=2048).
        For the elbow (ID=14), we are effectively sending gearAngle=0 => 2048.
        For other joints, also 2048.
        """
        print("Initializing right arm...")

        # We make a center_positions array of raw=2048 for all joints
        right_ids = self.writer.right_arm_ids

        # If you ever wanted to do a special gear angle for the elbow, you could do:
        # elbow_raw = self.gear_angle_to_raw(0.0)  # i.e. 0 deg at the elbow
        # For now, we keep it simple: 2048 for all
        center_positions = [2048]*len(right_ids)

        # Move all motors to "2048" in servo mode
        print(f"Moving right arm to center positions: {center_positions}")
        success = self.writer.write_arm_positions(right_ids, center_positions)
        if not success:
            print("Failed to send center positions to right arm motors")
            return False

        time.sleep(1)
        print("Right arm initialized")
        return True

    def initialize_left_arm(self) -> bool:
        """
        Home left arm motors to center (raw=2048).
        Same logic as the right. For the left elbow (ID=24),
        that's gearAngle=0 => raw=2048.
        """
        print("Initializing left arm...")

        left_ids = self.writer.left_arm_ids
        center_positions = [2048]*len(left_ids)

        print(f"Moving left arm to center positions: {center_positions}")
        success = self.writer.write_arm_positions(left_ids, center_positions)
        if not success:
            print("Failed to send center positions to left arm motors")
            return False

        time.sleep(1)
        print("Left arm initialized")
        return True

    def initialize_all_motors(self) -> bool:
        """
        Initialize all motors to their "center" positions.
        """
        right_success = self.initialize_right_arm()
        left_success  = self.initialize_left_arm()
        return right_success and left_success

def main():
    """Example usage of the StartupMotors class."""
    startup = None
    try:
        startup = StartupMotors()
        success = startup.initialize_all_motors()
        
        if success:
            print("All motors successfully initialized (gear ratio logic included for elbows).")
        else:
            print("Failed to initialize all motors")
            
    except Exception as e:
        print(f"Error during startup: {e}")
    finally:
        if startup is not None:
            startup.close()

if __name__ == "__main__":
    main()
