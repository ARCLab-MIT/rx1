#!/usr/bin/env python3
import zmq
import numpy as np
import time
from collections import defaultdict

def calculate_rate(timestamps, window=5.0):
    """Calculate message rate over the last window seconds"""
    current_time = time.time()
    # Keep only timestamps within window
    recent = [t for t in timestamps if (current_time - t) <= window]
    if len(recent) > 1:
        return len(recent) / window
    return 0

def main():
    # ZMQ setup
    context = zmq.Context()
    cartesian_sub = context.socket(zmq.SUB)
    joint_sub = context.socket(zmq.SUB)
    command_sub = context.socket(zmq.SUB)
    
    # Connect to publisher ports
    cartesian_sub.connect("tcp://localhost:8118")
    joint_sub.connect("tcp://localhost:8119")
    command_sub.connect("tcp://localhost:8120")
    
    # Subscribe to all messages
    cartesian_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    joint_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    command_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    
    print("Listening for ZMQ messages...")
    
    # Setup polling
    poller = zmq.Poller()
    poller.register(cartesian_sub, zmq.POLLIN)
    poller.register(joint_sub, zmq.POLLIN)
    poller.register(command_sub, zmq.POLLIN)
    
    # Message statistics
    msg_counts = defaultdict(int)
    msg_timestamps = defaultdict(list)
    last_stats_time = time.time()
    
    try:
        while True:
            socks = dict(poller.poll(timeout=100))  # 100ms timeout
            
            current_time = time.time()
            
            if cartesian_sub in socks:
                msg = cartesian_sub.recv_pyobj()
                msg_counts['cartesian'] += 1
                msg_timestamps['cartesian'].append(current_time)
                print("\nCartesian Message:")
                print(f"Position: {np.round(msg['position'], 3)}")
                print(f"Orientation (quat): {np.round(msg['orientation'], 3)}")
                print(f"Timestamp: {msg['timestamp']}")
                if 'frame' in msg:
                    print(f"Frame: {msg['frame']}")
                if 'endeff_homo' in msg:
                    print(f"Homogeneous Matrix:\n{np.round(np.array(msg['endeff_homo']), 3)}")
            
            if joint_sub in socks:
                msg = joint_sub.recv_pyobj()
                msg_counts['joint'] += 1
                msg_timestamps['joint'].append(current_time)
                print("\nJoint Message:")
                print(f"Names: {msg['joint_names']}")
                print(f"Positions: {np.round(msg['joint_positions'], 3)}")
                print(f"Timestamp: {msg['timestamp']}")
                print(f"Arm: {msg['arm']}")
            
            if command_sub in socks:
                msg = command_sub.recv_pyobj()
                msg_counts['command'] += 1
                msg_timestamps['command'].append(current_time)
                print("\nCommand Message:")
                print(msg)  # Print raw command message for debugging
            
            # Print statistics every second
            if current_time - last_stats_time >= 1.0:
                print("\n=== Statistics ===")
                for msg_type in msg_counts.keys():
                    rate = calculate_rate(msg_timestamps[msg_type])
                    print(f"{msg_type.capitalize()} Messages:")
                    print(f"  Count: {msg_counts[msg_type]}")
                    print(f"  Rate: {rate:.1f} Hz")
                print("================")
                last_stats_time = current_time
                
                # Cleanup old timestamps
                for timestamps in msg_timestamps.values():
                    while timestamps and (current_time - timestamps[0]) > 5.0:
                        timestamps.pop(0)
            
    except KeyboardInterrupt:
        print("\nClosing subscriber...")
    finally:
        cartesian_sub.close()
        joint_sub.close()
        command_sub.close()
        context.term()

if __name__ == "__main__":
    main() 