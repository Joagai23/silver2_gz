#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import csv
import os
from datetime import datetime
import math

class EffortAndPoseLogger(Node):
    def __init__(self):
        super().__init__('effort_and_pose_logger')
        
        os.makedirs('data', exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"data/motion_and_effort_{timestamp_str}.csv"
        
        # Open CSV and write headers: time, coordinates, movement status, and joint efforts
        self.csv_file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'sec', 'nanosec', 'x', 'y', 'z', 'is_moving', 
            'coxa_mean_effort', 'femur_mean_effort', 'tibia_mean_effort'
        ])
        
        self.get_logger().info(f"Logging motion, pose & efforts to {self.filename}")
        
        # QoS profile to match Gazebo bridge reliably
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, '/silver2/pose', self.pose_callback, qos_profile
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos_profile
        )
        
        # Coordinate and tracking states
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.is_moving = False
        self.current_sec = 0
        self.current_nanosec = 0
        
        # Effort buffers
        self.latest_coxa = 0.0
        self.latest_femur = 0.0
        self.latest_tibia = 0.0

    def pose_callback(self, msg):
        # Update coordinates and timestamp from header
        self.current_sec = msg.header.stamp.sec
        self.current_nanosec = msg.header.stamp.nanosec
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Determine if the robot has started moving (threshold of 0.1mm displacement)
        if math.hypot(self.current_x - self.last_x, self.current_y - self.last_y) > 0.0001:
            self.is_moving = True
        else:
            self.is_moving = False
            
        self.last_x = self.current_x
        self.last_y = self.current_y
        
        # Write synchronized data row upon pose updates
        self.csv_writer.writerow([
            self.current_sec, self.current_nanosec,
            self.current_x, self.current_y, self.current_z,
            1 if self.is_moving else 0,
            self.latest_coxa, self.latest_femur, self.latest_tibia
        ])

    def joint_callback(self, msg):
        if not msg.effort:
            return
            
        coxa_efforts = []
        femur_efforts = []
        tibia_efforts = []
        
        # Categorize joints based on URDF naming structure (coxa, femur, tibia)
        for name, effort in zip(msg.name, msg.effort):
            if 'coxa' in name:
                coxa_efforts.append(effort)
            elif 'femur' in name:
                femur_efforts.append(effort)
            elif 'tibia' in name:
                tibia_efforts.append(effort)
                
        # Calculate mean values for each group (if data exists)
        self.latest_coxa = sum(coxa_efforts) / len(coxa_efforts) if coxa_efforts else 0.0
        self.latest_femur = sum(femur_efforts) / len(femur_efforts) if femur_efforts else 0.0
        self.latest_tibia = sum(tibia_efforts) / len(tibia_efforts) if tibia_efforts else 0.0

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("Effort CSV file closed successfully.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger = EffortAndPoseLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()

if __name__ == '__main__':
    main()