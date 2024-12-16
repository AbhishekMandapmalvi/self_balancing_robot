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
        self.image_sub = rospy.Subscriber('/camera/image_rect', Image, self.image_callback)
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            ipm_image = self.apply_ipm(cv_image)
            
            if ipm_image is not None:
                # Detect obstacles in the IPM image
                obstacles = self.detect_obstacles(ipm_image)
                lines = self.detect_lanes(ipm_image)

                # Draw detected obstacles
                result_image = cv2.cvtColor(ipm_image, cv2.COLOR_GRAY2BGR)
                
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        cv2.line(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Display results
                cv2.imshow("IPM Image", ipm_image)
                cv2.imshow("Lane Detection", result_image)
                cv2.waitKey(1)
                
                # Publish the annotated image
                ipm_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
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

            # Ensure image is in correct format (uint8)
        if image.dtype != np.uint8:
            image = image.astype(np.uint8)
            
        # Apply Gaussian smoothing to reduce noise
        smoothed_image = cv2.GaussianBlur(image, (5, 5), 0)

        # Define ROI (focus on bottom portion where robot operates)
        roi_height = int(self.image_height * 1.0)  # Use 100% of image height for ROI
        roi_start = int(self.image_height * 0)   # Start from 0% down
        roi = smoothed_image[roi_start:self.image_height, :]

        # Ensure single channel before CLAHE
        if len(roi.shape) > 2:
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Enhance contrast for grayscale image
        clahe = cv2.createCLAHE(clipLimit=3.5, tileGridSize=(8,8))
        roi_enhanced = clahe.apply(roi)

        # Additional contrast enhancement
        roi_enhanced = cv2.convertScaleAbs(roi_enhanced, alpha=1.3, beta=10)

        # Define source points (in ROI coordinates)
        src_points = np.float32([
            [0, roi.shape[0]],                    # Bottom left
            [roi.shape[1], roi.shape[0]],         # Bottom right
            [roi.shape[1] * 0.3, roi.shape[0] * 0.60],  # Top left
            [roi.shape[1] * 0.7, roi.shape[0] * 0.60]   # Top right
        ])

        # Reduce the transformation area in destination points
        dst_points = np.float32([
            [self.output_resolution[0] * 0.4, self.output_resolution[1]],
            [self.output_resolution[0] * 0.6, self.output_resolution[1]],
            [self.output_resolution[0] * 0.4, self.output_resolution[1] * 0.5],
            [self.output_resolution[0] * 0.6, self.output_resolution[1] * 0.5]
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

        return warped

    def __del__(self):
        cv2.destroyAllWindows()
        cv2.waitKey(1)

    def detect_obstacles(self, warped_image):
        # Apply adaptive thresholding with THRESH_BINARY instead of THRESH_BINARY_INV
        binary = cv2.adaptiveThreshold(
            warped_image,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,  # Changed from THRESH_BINARY_INV
            11,
            2
        )
        
        # Apply morphological operations
        kernel = np.ones((5,5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w/2
                center_y = y + h/2
                obstacles.append((center_x, center_y, w, h))
        
        return obstacles
    
    def detect_lanes(self, warped_image):
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(warped_image, (5, 5), 0)
        
        # Use adaptive thresholding with THRESH_BINARY instead of THRESH_BINARY_INV
        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,  # Changed from THRESH_BINARY_INV
            21,
            7
        )
        
        # Create mask for region of interest
        height, width = binary.shape
        mask = np.zeros_like(binary)
        polygon = np.array([
            [(width * 0.4, height), (width * 0.6, height),
            (width * 0.6, height * 0.5), (width * 0.4, height * 0.5)]
        ])
        cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)
        
        # Apply mask
        masked_binary = cv2.bitwise_and(binary, mask)
        
        # Use Hough Transform to detect lines
        lines = cv2.HoughLinesP(
            masked_binary, 
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=10
        )
        
        return lines



    def color_threshold(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        return white_mask

if __name__ == '__main__':
    try:
        node = IPMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass