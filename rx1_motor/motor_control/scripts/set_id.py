#!/usr/bin/env python3
"""
set_id.py
A utility script to change the ID of SMS/STS series servo motors.

Usage:
    python set_id.py [options]

Options:
    --port PORT       Serial port to use (default: /dev/ttyACM0)
    --baudrate BAUD   Baudrate for serial communication (default: 1000000)
    --current-id ID   Current ID of the motor (default: 1)
    --new-id ID       New ID to set for the motor (required)
"""

import sys
import time
import argparse
import serial

class ServoMotor:
    """Class to interface with SMS/STS series servo motors"""
    
    # Command bytes
    HEADER = 0x55
    HEADER2 = 0x55
    CMD_WRITE = 0x03
    
    # Register addresses
    REG_ID = 0x05        # ID register address
    REG_LOCK = 0x37      # Lock register address (SMS_STS_LOCK)
    
    def __init__(self, port='/dev/ttyACM0', baudrate=1000000, timeout=0.1):
        """Initialize the servo motor communication"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        
    def connect(self):
        """Connect to the serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False
            
    def disconnect(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")
            
    def calculate_checksum(self, data):
        """Calculate the checksum for the packet"""
        return (~sum(data)) & 0xFF
        
    def unlock_eprom(self, motor_id):
        """Unlock the EPROM to allow ID changes
        
        Args:
            motor_id: Current ID of the motor
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            print("Serial port not open")
            return False
            
        # Construct the packet to unlock EPROM (write 0 to REG_LOCK)
        length = 4  # Length of the packet (not including headers and length byte)
        data = [motor_id, length, self.CMD_WRITE, self.REG_LOCK, 0]
        checksum = self.calculate_checksum(data)
        
        # Full packet with headers
        packet = [self.HEADER, self.HEADER2] + data + [checksum]
        
        try:
            self.serial.write(bytes(packet))
            self.serial.flush()
            print(f"Command sent to unlock EPROM for motor ID {motor_id}")
            
            # Wait for response
            time.sleep(0.1)
            
            # Check if there's a response
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                print(f"Received response: {' '.join([hex(b) for b in response])}")
            
            return True
        except Exception as e:
            print(f"Error sending unlock command: {e}")
            return False
        
    def write_id(self, current_id, new_id):
        """Change the ID of a servo motor
        
        Args:
            current_id: The current ID of the motor
            new_id: The new ID to set
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            print("Serial port not open")
            return False
            
        if not 0 <= new_id <= 253:
            print("New ID must be between 0 and 253")
            return False
            
        # Construct the packet
        length = 4  # Length of the packet (not including headers and length byte)
        data = [current_id, length, self.CMD_WRITE, self.REG_ID, new_id]
        checksum = self.calculate_checksum(data)
        
        # Full packet with headers
        packet = [self.HEADER, self.HEADER2] + data + [checksum]
        
        # Send the packet
        try:
            self.serial.write(bytes(packet))
            self.serial.flush()
            print(f"Command sent to change ID from {current_id} to {new_id}")
            
            # Wait for response
            time.sleep(0.1)
            
            # Check if there's a response
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                print(f"Received response: {' '.join([hex(b) for b in response])}")
            
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

def main():
    """Main function to parse arguments and change motor ID"""
    parser = argparse.ArgumentParser(description='Change the ID of a servo motor')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='Serial port to use')
    parser.add_argument('--baudrate', type=int, default=1000000,
                        help='Baudrate for serial communication')
    parser.add_argument('--current-id', type=int, default=1,
                        help='Current ID of the motor')
    parser.add_argument('--new-id', type=int, required=True,
                        help='New ID to set for the motor')
    
    args = parser.parse_args()
    
    # Create servo motor object
    servo = ServoMotor(port=args.port, baudrate=args.baudrate)
    
    # Connect to the servo
    if not servo.connect():
        sys.exit(1)
    
    try:
        # First unlock the EPROM
        if not servo.unlock_eprom(args.current_id):
            print("Failed to unlock EPROM")
            sys.exit(1)
            
        # Wait a moment for the unlock to take effect
        time.sleep(0.5)
        
        # Change the ID
        if servo.write_id(args.current_id, args.new_id):
            print(f"Successfully sent command to change motor ID from {args.current_id} to {args.new_id}")
            print("Please power cycle the motor for the change to take effect")
        else:
            print("Failed to change motor ID")
    finally:
        # Always disconnect
        servo.disconnect()

if __name__ == "__main__":
    main()