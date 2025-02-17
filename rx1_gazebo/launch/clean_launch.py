#!/usr/bin/env python3
import subprocess
import time
import os
import signal

def install_tools():
    """Install required tools if they're missing"""
    try:
        # Install required packages
        subprocess.run(['apt-get', 'update'], stdout=subprocess.DEVNULL)
        subprocess.run(['apt-get', 'install', '-y', 'net-tools', 'lsof', 'psmisc', 'iproute2', 'apt-utils'], stdout=subprocess.DEVNULL)
    except:
        print("Warning: Could not install required tools")

def check_port(port):
    """Check if a port is in use"""
    try:
        # Try netstat first
        cmd = f"netstat -tuln | grep :{port}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.stdout.strip():
            return True
            
        # Try ss as backup
        cmd = f"ss -tuln | grep :{port}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return bool(result.stdout.strip())
    except:
        return False

def kill_port_users(port):
    """Kill any process using the specified port"""
    try:
        # More aggressive approach to find and kill processes
        commands = [
            # Try to kill using fuser
            f"fuser -k -n tcp {port}",
            # Try to find using netstat and kill
            f"netstat -tlpn | grep :{port} | awk '{{print $7}}' | cut -d'/' -f1 | xargs -r kill -9",
            # Last resort: kill all possible Gazebo/ROS processes
            "pkill -9 gzserver",
            "pkill -9 gzclient",
            "pkill -9 gazebo",
            "pkill -9 rosmaster",
            # Force close the port using TCP reset
            f"echo 1 > /proc/sys/net/ipv4/tcp_abort_on_overflow",
            # Try to reset the network stack for this port
            f"iptables -A INPUT -p tcp --dport {port} -j DROP",
            f"iptables -D INPUT -p tcp --dport {port} -j DROP",
            # Kill any remaining processes
            f"pkill -9 -f {port}"
        ]
        
        for cmd in commands:
            try:
                subprocess.run(cmd, shell=True, stderr=subprocess.DEVNULL)
            except:
                continue

    except:
        pass

def kill_process(process_name):
    """Kill process by name using pkill"""
    try:
        subprocess.run(['pkill', '-9', '-f', process_name], stderr=subprocess.DEVNULL)
    except:
        pass

def cleanup_gazebo():
    """Clean up any existing Gazebo processes and lock files"""
    # Install required tools first
    install_tools()
    
    # Kill processes using Gazebo ports
    gazebo_ports = [11345, 11346, 11347]
    for port in gazebo_ports:
        kill_port_users(port)
        if check_port(port):
            print(f"Warning: Port {port} is still in use, trying again...")
            time.sleep(2)
            kill_port_users(port)
    
    # Clean up lock files and temporary files
    cleanup_commands = [
        "rm -rf /tmp/gazebo*",
        "rm -rf ~/.gazebo",
        "rm -rf /tmp/.X11-unix/X*",
        "rm -f /tmp/.gazebo-server",
        "rm -f /tmp/.gazebo-client",
        "rm -rf ~/.ros/log/*",
        "rm -rf /tmp/rostmp/*",
        "rm -f ~/.ros/roscore-*",
        "rm -rf /tmp/.ros/*",
        "rm -rf ~/.ros/running/*",
        "rm -f /tmp/.X11-unix/X*",
        "rm -f /tmp/.X*",
        # Add network socket cleanup
        "rm -f /tmp/.X11-unix/X*",
        "rm -f /tmp/.X*",
        # Force close any remaining sockets
        "echo 1 > /proc/sys/net/ipv4/tcp_abort_on_overflow"
    ]
    
    for cmd in cleanup_commands:
        try:
            subprocess.run(cmd, shell=True, stderr=subprocess.DEVNULL)
        except:
            pass
    
    # Try to reset network stack
    try:
        subprocess.run("service network-manager restart", shell=True, stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Wait for processes to fully terminate
    time.sleep(5)
    
    # Final check of ports
    for port in gazebo_ports:
        if check_port(port):
            print(f"Error: Port {port} is still in use after cleanup")
            return False
    return True

if __name__ == "__main__":
    if cleanup_gazebo():
        # Set Gazebo ports through environment variables
        os.environ['GAZEBO_MASTER_URI'] = 'http://localhost:11345'
        os.environ['GAZEBO_MODEL_DATABASE_URI'] = 'http://localhost:11346'
        
        # Launch your ROS launch file
        launch_cmd = "roslaunch rx1_gazebo rx1_gazebo.launch"
        os.system(launch_cmd)
    else:
        print("Failed to clean up Gazebo processes and ports")