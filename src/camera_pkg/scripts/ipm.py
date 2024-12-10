#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
import tf2_ros

class IPMNode:
    def __init__(self):
        rospy.init_node('ipm_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_rect_color', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ipm_pub = rospy.Publisher('ipm/image', Image, queue_size=1)

        # Initialize camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = 640  # Default value
        self.image_height = 480 # Default value
        
        # Load parameters from ipm_node namespace
        self.camera_frame = rospy.get_param('/camera/ipm_node/camera_frame')
        self.ground_frame = rospy.get_param('/camera/ipm_node/ground_frame')
        self.height = rospy.get_param('/camera/ipm_node/height')
        self.output_resolution = rospy.get_param('/camera/ipm_node/output_resolution')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            rospy.logwarn_once("Waiting for camera calibration parameters...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            ipm_image = self.apply_ipm(cv_image)
            
            if ipm_image is not None:
                ipm_msg = self.bridge.cv2_to_imgmsg(ipm_image, "bgr8")
                ipm_msg.header = msg.header
                self.ipm_pub.publish(ipm_msg)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera calibration parameters received")

    def apply_ipm(self, image):
        if self.camera_matrix is None:
            rospy.logwarn("Camera calibration parameters not yet received")
            return None

        # Apply Gaussian smoothing to reduce noise
        smoothed_image = cv2.GaussianBlur(image, (5, 5), 0)

        # Define ROI (focus on bottom portion where robot operates)
        roi_height = int(self.image_height * 1.0)  # Use 100% of image height for ROI
        roi_start = int(self.image_height * 0)   # Start from 0% down
        roi = smoothed_image[roi_start:self.image_height, :]

        # Enhance contrast using CLAHE (better than standard histogram equalization)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(3,3))
        lab[:,:,0] = clahe.apply(lab[:,:,0])
        roi_enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # Define source points (in ROI coordinates)
        src_points = np.float32([
            [0, roi.shape[0]],                    # Bottom left
            [roi.shape[1], roi.shape[0]],         # Bottom right
            [roi.shape[1] * 0.3, roi.shape[0] * 0.6],  # Top left
            [roi.shape[1] * 0.7, roi.shape[0] * 0.6]   # Top right
        ])

        # Reduce the transformation area in destination points
        dst_points = np.float32([
            [self.output_resolution[0] * 0.2, self.output_resolution[1]],
            [self.output_resolution[0] * 0.8, self.output_resolution[1]],
            [self.output_resolution[0] * 0.2, self.output_resolution[1] * 0.2],
            [self.output_resolution[0] * 0.8, self.output_resolution[1] * 0.2]
        ])

        # Calculate perspective transform matrix
        M = cv2.getPerspectiveTransform(src_points, dst_points)

        # Apply perspective transformation
        warped = cv2.warpPerspective(
            roi_enhanced,
            M,
            (self.output_resolution[0], self.output_resolution[1]),
            flags=cv2.INTER_LINEAR
        )

        # Add grid overlay for equal areas after transformation
        transformed_width = int(self.output_resolution[0] * 0.7)  # Using 70% of width (0.85 - 0.15)
        transformed_height = self.output_resolution[1]

        # Number of desired grid cells (e.g., 10x10 grid)
        grid_cells = 10

        # Calculate spacing for equal areas
        grid_spacing_x = int(transformed_width / grid_cells)
        grid_spacing_y = int(transformed_height / grid_cells)

        # Draw vertical lines with adjusted spacing
        for i in range(grid_cells + 1):
            x = int(self.output_resolution[0] * 0.15) + (i * grid_spacing_x)
            cv2.line(warped, (x, 0), (x, transformed_height), (50, 50, 50), 1)

        # Draw horizontal lines with equal spacing
        for i in range(grid_cells + 1):
            y = i * grid_spacing_y
            cv2.line(warped, 
                    (int(self.output_resolution[0] * 0.15), y),
                    (int(self.output_resolution[0] * 0.85), y),
                    (50, 50, 50), 1)


        return warped

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = IPMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass