#!/usr/bin/env python

import sys
import os
import select
import termios
import tty
import time
from typing import List, Tuple

# Adjust paths to stservo_sdk
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sdk_path = os.path.join(parent_dir, 'stservo_sdk')

sys.path.insert(0, sdk_path)
sys.path.insert(0, parent_dir)

try:
    from stservo_sdk.port_handler import PortHandler
    from stservo_sdk.protocol_packet_handler import COMM_SUCCESS
    from stservo_sdk.sts import sts
    from scripts.read import ReadMotors
except ImportError as e:
    print(f"Failed to import stservo_sdk: {e}")
    sys.exit(1)

class STServoWheel:
    """
    A class to control STServo in wheel mode, with arrow-key logic.
    """

    def __init__(self, 
                 servo_id: int = 14,
                 moving_acc: int = 30,
                 device_name: str = '/dev/ttyACM0',
                 baudrate: int = 1000000,
                 debug: bool = False):
        """
        Args:
            servo_id: ID of the servo to control
            moving_acc: Acceleration value
            device_name: e.g. '/dev/ttyACM0'
            baudrate: e.g. 1000000
            debug: Flag to enable/disable debug prints
        """
        self.servo_id = servo_id
        self.moving_acc = moving_acc
        self.baudrate = baudrate
        self.device_name = device_name
        self.debug = debug
        
        # Create port + packet handlers
        self.port_handler = PortHandler(self.device_name)
        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open the port")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError("Failed to set baudrate")
        print("Port initialized successfully")

        self.packet_handler = sts(self.port_handler)

    def close(self):
        """Close the port."""
        self.port_handler.closePort()

    def set_wheel_mode(self) -> Tuple[int,int]:
        """Put this servo into wheel (continuous) mode."""
        return self.packet_handler.WheelMode(self.servo_id)

    def set_speed(self, speed: int) -> Tuple[int,int]:
        """
        Write a speed in wheel mode:
            speed > 0 => one direction
            speed < 0 => opposite direction
            speed=0 => stop
        """
        direction_str = "(stop)" if speed == 0 else "(+speed)" if speed > 0 else "(-speed)"
        if self.debug:
            print(f"[Servo {self.servo_id}] set_speed({speed}) {direction_str}")
        return self.packet_handler.WriteSpec(self.servo_id, speed, self.moving_acc)

    def set_position_mode(self) -> Tuple[int,int]:
        """Put this servo into position control mode."""
        return self.packet_handler.write1ByteTxRx(self.servo_id, 33, 0)  # 33 is STS_MODE

def parse_arrow_key(seq: str) -> str:
    """
    Given a 3-char sequence like '\x1b[A' etc., return 'UP','DOWN','LEFT','RIGHT' or 'ESC'/None.
    If it's just single chars, handle that too.
    """
    # Typical arrow key sequences in a Linux terminal:
    #  - Up:    \x1b [ A
    #  - Down:  \x1b [ B
    #  - Right: \x1b [ C
    #  - Left:  \x1b [ D
    #  - Esc alone: \x1b
    if seq == '\x1b':
        return 'ESC'
    if seq.startswith('\x1b['):
        if len(seq) == 3:
            if seq[2] == 'A': return 'UP'
            if seq[2] == 'B': return 'DOWN'
            if seq[2] == 'C': return 'RIGHT'
            if seq[2] == 'D': return 'LEFT'
    # If something else, return None or 'CHAR'
    return None

def nonblocking_get_arrow_key() -> str:
    """
    Reads up to 3 bytes from stdin if available.
    Returns 'UP','DOWN','LEFT','RIGHT','ESC', or None if no data.
    """
    # Use select to see if there's data
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    if r:
        # We have at least one byte
        # Typically arrow keys are 3 bytes. Let's read up to 3
        data = os.read(sys.stdin.fileno(), 3).decode(errors='ignore')
        return parse_arrow_key(data)
    else:
        return None

def control_loop():
    """
    Control right or left arm elbow with arrow keys, in a "hold-to-move" manner:
      - Start controlling right elbow = ID=14
      - Press LEFT arrow => switch to left elbow (ID=24)
      - Press RIGHT arrow => switch to right elbow (ID=14)
      
      Right elbow logic:
       - UP => speed=-400 (CW)
       - DOWN => speed=+400 (CCW)
      Left elbow logic:
       - UP => speed=+400 (CCW)
       - DOWN => speed=-400 (CW)
       
      If no key press is detected => speed=0 (stop).
      ESC => exit loop.
    """
    # We'll have 2 STServoWheel objects, or we can re-use one object by changing servo_id each time.
    # For simplicity, let's just create one and reassign servo_id.
    wheel = STServoWheel(servo_id=14, moving_acc=15)
    
    # Terminal setup: Make stdin raw
    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    current_elbow_id = 14  # 'right' by default
    # Put the current servo in wheel mode
    wheel.servo_id = current_elbow_id
    wheel.set_wheel_mode()

    print("\n--- Hold-to-Move Arrow Key Controls ---")
    print("UP / DOWN => move elbow. LEFT => switch to left elbow, RIGHT => switch to right elbow.")
    print()
    print("Press ESC to quit.\n")

    try:
        while True:
            # 1) Read if arrow key is pressed
            key = nonblocking_get_arrow_key()
            
            # 2) Decide speed
            speed = 0  # default if no arrow or if different key

            if key is not None:
                # We got some arrow key
                if key == 'ESC':
                    print("ESC pressed => exit.")
                    break
                elif key == 'LEFT':
                    # switch to left elbow
                    current_elbow_id = 24
                    wheel.servo_id = 24
                    wheel.set_wheel_mode()
                    print("Switched to LEFT elbow (24).")
                elif key == 'RIGHT':
                    # switch to right elbow
                    current_elbow_id = 14
                    wheel.servo_id = 14
                    wheel.set_wheel_mode()
                    print("Switched to RIGHT elbow (14).")
                elif key == 'UP':
                    # Move up => depends on which elbow
                    if current_elbow_id == 14:
                        # Right elbow => UP => CW => speed=-400
                        speed = -400
                    else:
                        # Left elbow => UP => CCW => speed=+400
                        speed = 400
                elif key == 'DOWN':
                    # Move down => depends on which elbow
                    if current_elbow_id == 14:
                        # Right elbow => DOWN => CCW => speed=+400
                        speed = 400
                    else:
                        # Left elbow => DOWN => CW => speed=-400
                        speed = -400

            # 3) Set speed
            if speed != 0:
                wheel.set_speed(speed)
            else:
                # If no arrow press => stop
                wheel.set_speed(0)

            # 4) short pause
            time.sleep(0.05)

    finally:
        # Cleanup
        wheel.set_speed(0)  # Stop the motor
        
        # Switch both motors back to position mode
        original_id = wheel.servo_id  # Store current ID
        
        # Switch right motor (ID 14) to position mode
        wheel.servo_id = 14
        wheel.set_position_mode()
        
        # Switch left motor (ID 24) to position mode
        wheel.servo_id = 24
        wheel.set_position_mode()
        
        # Restore original ID and close
        wheel.servo_id = original_id
        wheel.close()
        
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        print("Control loop ended. Motors returned to position mode.")

if __name__ == "__main__":
    control_loop()
