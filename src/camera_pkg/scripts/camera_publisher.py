#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import yaml

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create publishers with correct namespace
    pub = rospy.Publisher('image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    
    bridge = CvBridge()
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        exit(1)
    
    # Get parameters
    width = rospy.get_param('~image_width', 640)
    height = rospy.get_param('~image_height', 480)
    
    # Initialize camera info message
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera"
    camera_info_msg.height = height
    camera_info_msg.width = width
    
    # Load calibration data
    camera_info_url = rospy.get_param('~camera_info_url', '')
    rospy.loginfo(f"Loading calibration from: {camera_info_url}")
    
    if camera_info_url:
        try:
            with open(camera_info_url.replace('file:', ''), 'r') as f:
                calib_data = yaml.safe_load(f)
                rospy.loginfo("Calibration data loaded successfully")
                camera_info_msg.D = calib_data.get('distortion_coefficients', {}).get('data', [])
                camera_info_msg.K = calib_data.get('camera_matrix', {}).get('data', [])
                camera_info_msg.R = calib_data.get('rectification_matrix', {}).get('data', [])
                camera_info_msg.P = calib_data.get('projection_matrix', {}).get('data', [])
                camera_info_msg.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
        except Exception as e:
            rospy.logerr(f"Failed to load calibration: {e}")
    
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera"
            
            # Update camera_info timestamp
            camera_info_msg.header.stamp = msg.header.stamp
            
            pub.publish(msg)
            camera_info_pub.publish(camera_info_msg)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
