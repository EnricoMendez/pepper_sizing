# Author: Enrico Mendez
# Date: 20 Febrero 2024
# Description: Server for the image processing


import time
from sensor_msgs.msg import Image
from interfaces.srv import ImageProcessing
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageServer(Node):
    def __init__(self):
        super().__init__('Image_process')
        self.get_logger().info('Image_process service initialized')

        # Create variables
        self.org_img = np.array((720, 1280, 3))
        # self.depth_map = np.array((720, 1280, 1))
        self.valid_img = False
        self.valid_depth = False 
        
        # Define constants
        self.bridge = CvBridge()

        # Create suscriber 
        self.camera_sub = self.create_subscription(Image,'camera/camera/color/image_raw', self.cam_callback, 10)
        self.depth_sub = self.create_subscription(Image,'/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        # Create server
        self.srv = self.create_service(ImageProcessing, 'image_processing_service', self.image_process)

        # For debuggin
        self.image_deb_pub = self.create_publisher(Image,'Image/debug',10)
        self.image_deb_msg = Image()
        
    def cam_callback(self,msg):
        if not self.valid_img:
            self.get_logger().info("Image recieved")
            self.valid_img = True
        self.org_img = self.bridge.imgmsg_to_cv2(msg)
        self.capture_time = time.localtime(time.time())

    def depth_callback(self, msg):
        if not self.valid_depth:
            self.get_logger().info("Depth image recieved")
            self.valid_depth = True   
        self.depth_map = self.bridge.imgmsg_to_cv2(msg)
        self.depth_capture_time = time.localtime(time.time())

    def image_process(self, request, response):
        while not self.valid_img & self.valid_depth: 
            response.data = 'Image is not avilable' 
            return response
        self.get_logger().info("Message recived")
        id = request.id
        segmented_img = self.segmentation(self.org_img)
        distance = self.distance_calc(segmented_img)
        response.data = 'The id is {}'.format(id)
        return response
    def segmentation(self,src):
        gray_img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray_img, 0,255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        inverted_otsu = cv2.bitwise_not(otsu)
        mask = self.biggest_blob_selection(inverted_otsu)
        self.image_deb_msg = self.bridge.cv2_to_imgmsg(mask*255)
        self.image_deb_pub.publish(self.image_deb_msg)
        return otsu
    def biggest_blob_selection(self, src):
        contours,hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        maxContour = 0
        # Check for the biggest blob
        for contour in contours:
            
            contourSize = cv2.contourArea(contour)
            if contourSize > maxContour:
                maxContour = contourSize
                maxContourData = contour

        # Create a mask from the largest contour
        mask = np.zeros_like(src)
        mask = mask.astype('uint8')
        mask=cv2.fillPoly(mask,[maxContourData],1)
        return mask
    def distance_calc(self,src):
        pass

        
def main(args=None):

    rclpy.init(args=args)

    node = ImageServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()