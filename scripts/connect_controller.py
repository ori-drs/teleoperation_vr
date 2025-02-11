#!/usr/bin/env python3
'''
This script listens for UDP packets from the Quest 3 Headset controller and publishes them as a ROS topic in controller.msg format.
This script interfaces with Teleoperation_v3
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
    left_controller = {"position": None, "orientation": None, "gripper": 0, "thumbstick": [0,0]}
    right_controller = {"position": None, "orientation": None, "gripper": 0, "thumbstick": [0,0]}


    try:
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)  # Receive UDP packet
                msg = data.decode().strip()
                print(f"Received: {msg}")

                # Parse incoming messages
                if msg.startswith("Left controller position:"):
                    left_controller["position"] = extract_list(msg)
                elif msg.startswith("Left controller orientation:"):
                    left_controller["orientation"] = extract_list(msg)
                elif msg.startswith("Left controller gripper:"):
                    left_controller["gripper"] = bool(int(msg.split(":")[1].strip()))
                elif msg.startswith("Left controller thumbstick:"):
                    left_controller["thumbstick"] = extract_list(msg)
                elif msg.startswith("Right controller position:"):
                    right_controller["position"] = extract_list(msg)
                elif msg.startswith("Right controller orientation:"):
                    right_controller["orientation"] = extract_list(msg)
                elif msg.startswith("Right controller gripper:"):
                    right_controller["gripper"] = bool(int(msg.split(":")[1].strip()))
                elif msg.startswith("Right controller thumbstick:"):
                    right_controller["thumbstick"] = extract_list(msg)

                if None not in left_controller.values() and None not in right_controller.values():
                    ros_msg = controller()

                    # Left controller handling
                    ros_msg.left_controller_pose.position.x, ros_msg.left_controller_pose.position.y, ros_msg.left_controller_pose.position.z = left_controller["position"]
                    ros_msg.left_controller_pose.orientation.x, ros_msg.left_controller_pose.orientation.y, ros_msg.left_controller_pose.orientation.z, ros_msg.left_controller_pose.orientation.w = left_controller["orientation"]
                    ros_msg.left_gripper = left_controller["gripper"]
                    ros_msg.left_thumbstick_position.x, ros_msg.left_thumbstick_position.y = left_controller["thumbstick"]

                    # Right controller handling
                    ros_msg.right_controller_pose.position.x, ros_msg.right_controller_pose.position.y, ros_msg.right_controller_pose.position.z = right_controller["position"]
                    ros_msg.right_controller_pose.orientation.x, ros_msg.right_controller_pose.orientation.y, ros_msg.right_controller_pose.orientation.z, ros_msg.right_controller_pose.orientation.w = right_controller["orientation"]
                    ros_msg.right_gripper = right_controller["gripper"]
                    ros_msg.right_thumbstick_position.x, ros_msg.right_thumbstick_position.y = right_controller["thumbstick"]

                    pub.publish(ros_msg)
                    # print(f"Published: {ros_msg}")

            except socket.timeout:
                continue  # Allow loop to continue if no data arrives

    except KeyboardInterrupt:
        print("Shutting down UDP listener.")
        sock.close()

if __name__ == "__main__":
    udp_listener()
