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

    # 動画ファイルを開く
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        rospy.logerr("Failed to open video file: %s", video_path)
        return

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 画像の明るさとコントラスト調整（必要なら有効化）
        # frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=50)

        # OpenCVの画像をROSメッセージに変換
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # 現在のROS時間をタイムスタンプに設定
        msg.header.stamp = rospy.Time.now()

        # パブリッシュ
        pub.publish(msg)

        # 30FPSに合わせる
        rate.sleep()

    cap.release()
    rospy.loginfo("Video playback completed.")

if __name__ == '__main__':
    try:
        video_path = "/home/mutsumi/data/2.mp4"  # 動画ファイルのパスを設定
        publish_video(video_path)
    except rospy.ROSInterruptException:
        pass

