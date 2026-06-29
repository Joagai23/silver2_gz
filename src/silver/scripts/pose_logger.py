#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        
        # Create a 'data' directory in the current working directory if it doesn't exist
        os.makedirs('data', exist_ok=True)
        
        # Generate a unique filename using the current date and time
        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"data/silver_pose_{timestamp_str}.csv"
        
        # Open the CSV file and write the header
        self.csv_file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['sec', 'nanosec', 'x', 'y', 'z'])
        
        self.get_logger().info(f"Logging pose data to {self.filename}")
        
        # Subscribe to the bridged PoseStamped topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/silver2/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Extract seconds, nanoseconds, and position coordinates
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Write the row to the CSV file
        self.csv_writer.writerow([sec, nanosec, x, y, z])

    def destroy_node(self):
        # Ensure the file is safely closed when the node shuts down
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV file closed successfully.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger = PoseLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()

if __name__ == '__main__':
    main()