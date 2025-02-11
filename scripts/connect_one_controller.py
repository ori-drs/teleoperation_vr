#!/usr/bin/env python3
'''
This script listens for UDP packets from the Quest 3 Headset right controller and publishes them as a ROS topic in one_controller.msg format.
This script interfaces with Teleoperation_v2
'''


import rospy
import socket
import re
import numpy as np
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Bool
from teleoperation_vr.msg import controller  # Replace with your actual package name

# UDP Configurations
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 7777

def extract_list(s):
    """Extracts a list of floats from a formatted string."""
    match = re.search(r'\[\s*([-?\d.e,\s]+)\s*\]', s)
    return [float(x) for x in match.group(1).split(',')] if match else None

def udp_listener():
    """Receives UDP data, parses it, and publishes as a ROS topic."""
    rospy.init_node("udp_to_ros_publisher", anonymous=True)
    
    # ROS Publisher
    pub = rospy.Publisher("vr_controller", controller, queue_size=10)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)  # Prevent blocking indefinitely

    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

    # Data storage (to track latest values)
    latest_data = {"position": None, "orientation": None, "gripper": 0, "base": [0,0]}

    try:
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)  # Receive UDP packet
                msg = data.decode().strip()
                print(f"Received: {msg}")

                # Parse incoming messages
                if msg.startswith("EE Position:"):
                    latest_data["position"] = extract_list(msg)
                elif msg.startswith("EE Orientation:"):
                    latest_data["orientation"] = extract_list(msg)
                elif msg.startswith("EE Gripper:"):
                    latest_data["gripper"] = bool(int(msg.split(":")[1].strip()))
                elif msg.startswith("Thumbstick:"):
                    latest_data["base"] = extract_list(msg)
                # print('Processed: ',latest_data)
                # Ensure we have all required data before publishing
                if None not in latest_data.values():
                    ros_msg = controller()
                    ros_msg.ee_pose.position.x, ros_msg.ee_pose.position.y, ros_msg.ee_pose.position.z = latest_data["position"]
                    ros_msg.ee_pose.orientation.x, ros_msg.ee_pose.orientation.y, ros_msg.ee_pose.orientation.z, ros_msg.ee_pose.orientation.w = latest_data["orientation"]
                    ros_msg.gripper = latest_data["gripper"]
                    ros_msg.base_position.x, ros_msg.base_position.y = latest_data["base"]
                    pub.publish(ros_msg)
                    # print(f"Published: {ros_msg}")

            except socket.timeout:
                continue  # Allow loop to continue if no data arrives

    except KeyboardInterrupt:
        print("Shutting down UDP listener.")
        sock.close()

if __name__ == "__main__":
    udp_listener()
