#!/usr/bin/env python3
import roslibpy
import time

def main():
    # Connect to rosbridge
    client = roslibpy.Ros(host='localhost', port=9090)
    
    # Define publisher
    talker = roslibpy.Topic(client, '/test_topic', 'std_msgs/String')
    
    try:
        # Connect to ROS
        client.run()
        print('Connected to ROS bridge')
        
        # Publish messages every second
        while client.is_connected:
            talker.publish(roslibpy.Message({'data': 'Hello from host machine!'}))
            print('Message published')
            time.sleep(1)
            
    except KeyboardInterrupt:
        client.terminate()
        
if __name__ == '__main__':
    main()
