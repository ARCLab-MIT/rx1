#!/usr/bin/env python

import sys
import os
from typing import List, Tuple

# Get the absolute path to the stservo_sdk directory
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sdk_path = os.path.join(parent_dir, 'stservo_sdk')

# Add both the SDK path and its parent directory to sys.path
sys.path.insert(0, sdk_path)
sys.path.insert(0, parent_dir)

try:
    from stservo_sdk.port_handler import PortHandler
    from stservo_sdk.protocol_packet_handler import COMM_SUCCESS
    from stservo_sdk.sts import sts
except ImportError as e:
    print(f"Failed to import stservo_sdk: {e}")
    print(f"Current sys.path: {sys.path}")
    sys.exit(1)

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class ReadMotors:
    def __init__(self, device_name: str = '/dev/ttyACM0', baudrate: int = 1000000):
        # Motor IDs configuration
        self.right_arm_ids = [i for i in range(1, 19)]
        self.left_arm_ids = [i for i in range(21, 28)]
        
        # Position to angle conversion constants
        self.ZERO_POSITION = 2048  # Position value for 0 degrees
        self.POSITIONS_PER_ROTATION = 4096  # Total positions per 360 degrees
        
        # Communication setup
        self.device_name = device_name
        self.baudrate = baudrate
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = sts(self.port_handler)
        
        # Initialize connection
        self._initialize_connection()

    def _initialize_connection(self) -> None:
        """Initialize the connection with the motor controller."""
        # Open port
        if not self.port_handler.openPort():
            raise ConnectionError("Failed to open the port")

        # Set port baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError("Failed to change the baudrate")
            
        print("Successfully connected to motor controller")

    def position_to_angle(self, position: int) -> float:
        """
        Convert a position value to its corresponding angle in degrees.
        Args:
            position: Raw position value from the motor
        Returns:
            float: Angle in degrees (-180 to 180 range)
        """
        if position is None:
            return None
            
        # Calculate relative position from zero point
        relative_pos = position - self.ZERO_POSITION
        
        # Convert to degrees (360 degrees / 4096 positions = 0.087890625 degrees per position)
        angle = (relative_pos * 360.0) / self.POSITIONS_PER_ROTATION
        
        # Normalize to -180 to 180 range
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
            
        return round(angle, 2)

    def read_arm_positions(self, ids: List[int]) -> List[int]:
        """Read positions for a given list of motor IDs."""
        positions = []
        for id in ids:
            pos, spd, comm_result, error = self.packet_handler.ReadPosSpeed(id)
            if comm_result != COMM_SUCCESS:
                print(f"Communication error for ID {id}: {self.packet_handler.getTxRxResult(comm_result)}")
                positions.append(None)
            elif error != 0:
                print(f"Packet error for ID {id}: {self.packet_handler.getRxPacketError(error)}")
                positions.append(None)
            else:
                positions.append(pos)
        return positions

    def read_arm_angles(self, ids: List[int]) -> List[float]:
        """Read angles for a given list of motor IDs."""
        positions = self.read_arm_positions(ids)
        return [self.position_to_angle(pos) for pos in positions]

    def read_all_positions(self) -> Tuple[List[int], List[int]]:
        """Read positions for both arms and return them as separate arrays."""
        right_positions = self.read_arm_positions(self.right_arm_ids)
        left_positions = self.read_arm_positions(self.left_arm_ids)
        return right_positions, left_positions

    def read_all_angles(self) -> Tuple[List[float], List[float]]:
        """Read angles for both arms and return them as separate arrays."""
        right_positions, left_positions = self.read_all_positions()
        right_angles = [self.position_to_angle(pos) for pos in right_positions]
        left_angles = [self.position_to_angle(pos) for pos in left_positions]
        return right_angles, left_angles

    def close(self):
        """Close the port connection."""
        self.port_handler.closePort()

def main():
    """Example usage of the ReadMotors class."""
    try:
        # Initialize the reader
        reader = ReadMotors()
        
        while True:
            print("\nPress any key to read positions (ESC to quit)")
            if getch() == chr(0x1b):
                break
                
            # Read positions and angles
            right_pos, left_pos = reader.read_all_positions()
            right_angles, left_angles = reader.read_all_angles()
            
            # Display results
            print("\nRight Arm Positions:", right_pos)
            print("Right Arm Angles (degrees):", right_angles)
            print("Left Arm Positions:", left_pos)
            print("Left Arm Angles (degrees):", left_angles)

            # Print right elbow joint angle:
            # print("Right Elbow Joint Angle (degrees):", right_angles[3])
            # Print left elbow joint angle:
            # print("Left Elbow Joint Angle (degrees):", left_angles[3])
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        reader.close()

if __name__ == "__main__":
    main()
