# Author: Enrico Mendez
# Date: 13 Febrero 2024
# Description: Node for image capture of the peppers

import csv
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time


class DataAdquisition(Node):
    def __init__(self):
        super().__init__('Image_capture')
        self.get_logger().info('Image_capture initialized')
        
        # Define constants
        pkg_path = self.get_pkg_path()
        file_name = '/data.csv'
        self.full_file_name = pkg_path + file_name
        self.bridge = CvBridge()
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.data_register)
        
        # Create variables
        self.id = self.check_id(self.full_file_name)

        # Create client
        

    def reader_csv(self, file_name):

        with open (file_name, 'r+') as my_file:
            text_read = csv.reader(my_file)
            data = list(text_read)
        return data

    def new_entry(self, file_name,content):
        with open(file_name, 'a') as my_file:
            writer = csv.writer(my_file)
            writer.writerow(content)

    def create_csv(self, file_name):
        with open (file_name, 'x') as my_file :
            writer = csv.writer(my_file)
            
            header = ['Id',' Original image path', 'Processed image path', 
                            'Area in image (px2)',' Area in image (cm2)',
                                'Weight (g)', 'Time stamp']
            writer.writerow(header)

    def check_id(self,file_name):
        try:
            content = self.reader_csv(file_name)
            current_id = content[-1][0]
            if current_id == 'Id': id = 1 
            else: id = int(current_id) + 1
        except:
            self.create_csv(file_name)
            id = 1
        self.get_logger().info("Id obtained: {}".format(id))
        return id

    def get_pkg_path(self,target='size_estimation'):
        # Get exc path
        pkg_path = get_package_share_directory(target)

        # Converting to list
        parts = pkg_path.split('/')

        # Directing to the src folder
        replace = 'install'
        idx = parts.index(replace)
        parts[idx] = 'src'
        parts.remove('share')

        # Converting back to string
        path = '/'.join(parts)

        return path
    
    def data_register(self):
        self.get_logger().info("Ready for capture")
        self.get_logger().info("Press enter to capture")
        keyboard_input = input()
        if keyboard_input == '' : self.get_logger().info("Proceding ...")
        print("Capture time:", time.strftime("%H:%M:%S", self.capture_time))
        print("Printing time:", time.strftime("%H:%M:%S", time.localtime(time.time())))
        return
        
            


def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = DataAdquisition()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()