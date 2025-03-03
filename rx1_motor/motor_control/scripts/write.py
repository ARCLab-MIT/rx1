#!/usr/bin/env python

import sys
import os
from typing import Tuple

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

# Add the path to stservo_sdk
sys.path.append(os.path.join(os.path.dirname(__file__), '../stservo_sdk'))
from stservo_sdk import * 

class WriteMotors:
    def __init__(self, device_name: str = '/dev/ttyACM0', baudrate: int = 1000000):
        # Motor IDs configuration
        self.right_arm_ids = [i for i in range(11, 18)]
        self.left_arm_ids = [i for i in range(21, 28)]
        
        # Communication setup
        self.device_name = device_name
        self.baudrate = baudrate
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = sts(self.port_handler)
        
        # Default movement parameters
        self.moving_speed = 600
        self.moving_acc = 50
        
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

    def write_motor_position(self, motor_id: int, position: int) -> Tuple[int, int]:
        """
        Write position to a specific motor.
        
        Args:
            motor_id: ID of the motor to control
            position: Target position (0-4095)
            
        Returns:
            Tuple of (comm_result, error)
        """
        comm_result, error = self.packet_handler.WritePosEx(
            motor_id, 
            position, 
            self.moving_speed, 
            self.moving_acc
        )
        
        if comm_result != COMM_SUCCESS:
            print(f"Communication error for ID {motor_id}: {self.packet_handler.getTxRxResult(comm_result)}")
        if error != 0:
            print(f"Packet error for ID {motor_id}: {self.packet_handler.getRxPacketError(error)}")
            
        return comm_result, error

    def write_arm_positions(self, ids: list, positions: list) -> bool:
        """
        Write positions to a list of motors.
        
        Args:
            ids: List of motor IDs
            positions: List of target positions
            
        Returns:
            bool: True if all writes were successful
        """
        if len(ids) != len(positions):
            raise ValueError("Number of IDs must match number of positions")
            
        success = True
        for motor_id, position in zip(ids, positions):
            comm_result, error = self.write_motor_position(motor_id, position)
            if comm_result != COMM_SUCCESS or error != 0:
                success = False
                
        return success

    def set_movement_params(self, speed: int = None, acceleration: int = None) -> None:
        """Update movement parameters."""
        if speed is not None:
            self.moving_speed = speed
        if acceleration is not None:
            self.moving_acc = acceleration

    def close(self):
        """Close the port connection."""
        self.port_handler.closePort()

def main():
    """Example usage of the WriteMotors class."""
    try:
        # Initialize the writer
        writer = WriteMotors()
        
        # Example position for testing
        test_position = 1000
        
        while True:
            print(f"\nPress any key to move motors to position {test_position} (ESC to quit)")
            if getch() == chr(0x1b):
                break
                
            # Example: Move first motor of right arm
            comm_result, error = writer.write_motor_position(writer.right_arm_ids[0], test_position)
            
            if comm_result == COMM_SUCCESS and error == 0:
                print(f"Successfully moved motor to position {test_position}")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        writer.close()

if __name__ == "__main__":
    main()
