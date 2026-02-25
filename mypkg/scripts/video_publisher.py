#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_video(video_path):
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)  # 30 FPS
    bridge = CvBridge()
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=50)  # コントラストと明るさ調整

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        rospy.logerr("Failed to open video file: %s", video_path)
        return

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Publish frame
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()
    rospy.loginfo("Finished publishing video.")

if __name__ == '__main__':
    try:
        video_path = rospy.get_param("~video_path", "/home/mutsumi/data/1.mp4")
        publish_video(video_path)
    except rospy.ROSInterruptException:
        pass

