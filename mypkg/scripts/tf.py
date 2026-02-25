#!/usr/bin/env python3
# coding: utf-8

import rospy
import tf2_ros
import geometry_msgs.msg

def pose_callback(pose_stamped_msg):
    # ブロードキャスタを作る (都度作る方法; パフォーマンス的にはグローバルに一個でもOK)
    br = tf2_ros.TransformBroadcaster()

    # TransformStampedを用意
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()                # タイムスタンプ
    t.header.frame_id = "world"                      # 親フレーム
    t.child_frame_id = "camera"                      # 子フレーム

    # 平行移動 (x, y, z)
    t.transform.translation.x = pose_stamped_msg.pose.position.x
    t.transform.translation.y = pose_stamped_msg.pose.position.y
    t.transform.translation.z = pose_stamped_msg.pose.position.z

    # 回転 (geometry_msgs/Quaternion)
    t.transform.rotation = pose_stamped_msg.pose.orientation

    # 実際のブロードキャスト
    br.sendTransform(t)

def main():
    rospy.init_node('camera_tf_broadcaster')
    # カメラの姿勢を吐き出すトピック名を適宜合わせる
    rospy.Subscriber("/camera_pose", geometry_msgs.msg.PoseStamped, pose_callback)
    rospy.loginfo("camera_tf_broadcaster node started. Waiting for /camera_pose messages...")
    rospy.spin()

if __name__ == '__main__':
    main()

