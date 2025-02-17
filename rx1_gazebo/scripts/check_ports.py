#!/usr/bin/env python3
import subprocess
import re

def check_gazebo_ports():
    """
    Check ports used by Gazebo processes in the container
    Returns: Dictionary with process name and port information
    """
    try:
        # Get all network connections
        cmd = "netstat -tulpn | grep gazebo"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode != 0:
            # Try alternative command if netstat is not available
            cmd = "ss -tulpn | grep gazebo"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        ports = {}
        for line in result.stdout.splitlines():
            # Extract port numbers and process info
            match = re.search(r':(\d+).*?(\d+/.*?)\s*$', line)
            if match:
                port, process = match.groups()
                ports[process] = port
                
        return ports
    
    except Exception as e:
        print(f"Error checking ports: {str(e)}")
        return None

if __name__ == "__main__":
    ports = check_gazebo_ports()
    if ports:
        print("\nGazebo Ports in Use:")
        for process, port in ports.items():
            print(f"Process: {process} - Port: {port}")
    else:
        print("No Gazebo processes found or error occurred")