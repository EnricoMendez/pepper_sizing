# Author: Enrico Mendez
# Date: 11 March 2024
# Description: node to recieve images from realsense d435i and publish them in new topics each 4 seconds
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class image_extractor(Node):
    def __init__(self):
        super().__init__('Image_extractor')
        self.get_logger().info('Image_extractor initialized')

        # Create variables
        self.color_image = Image()
        self.depth_image = Image()
        self.depth_info  = CameraInfo()
        
        # Define constants
        self.timer_period = 4
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Create publishers
        self.pub_color_image  = self.create_publisher(Image,'rgb_image',10)
        self.msg_color_image  = Image()
        self.pub_depth_image  = self.create_publisher(Image,'depth_image',10)
        self.msg_depth_image  = Image()
        self.pub_depth_info  = self.create_publisher(CameraInfo,'camera_info',10)
        self.msg_depth_info  = CameraInfo()
        
        # Create subscribers
        self.color_image_sub = self.create_subscription(Image,'/camera/color/image_raw', self.color_image_callback, 10)
        self.sub_depth_image = self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10) 
        self.sub_depth_info = self.create_subscription(CameraInfo,'/camera/aligned_depth_to_color/camera_info', self.depth_info_callback, 10)

    def color_image_callback(self,data):
        self.msg_color_image = data

    def depth_image_callback(self,data):
        self.msg_depth_image = data
    
    def depth_info_callback(self, data):
        self.msg_depth_info = data

    def timer_callback(self):
        # Publish depth, color images and camera info
        self.pub_color_image.publish(self.msg_color_image)
        self.pub_depth_image.publish(self.msg_depth_image)
        self.pub_depth_info.publish(self.msg_depth_info)
        
def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = image_extractor()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
