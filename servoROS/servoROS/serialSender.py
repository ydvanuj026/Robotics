#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import serial
import math
import time

ser = serial.Serial('/dev/ttyACM2', 115200)

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("serialSender")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 1)

    def pose_callback(self, msg: Pose):
        th = msg.theta + math.pi
        angle = int((th / (2 * math.pi)) * 180)
        if 0 <= angle <= 180:
            self.get_logger().info("Servo angle set to: " + str(angle))
            ser.write((str(angle) + "\n").encode())  # Send as string with
        
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
