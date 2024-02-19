#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
import tf2_ros 
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R


bridge = CvBridge()
camera_matrix = None
distortion_coefficients = None
cv_image = None


def camera_info_callback(msg):
    global camera_matrix, distortion_coefficients
    temp = [list(msg.K[0:3]), list(msg.K[3:6]), list(msg.K[6:9])]
    camera_matrix = np.array(temp)
    distortion_coefficients = np.array(msg.D)

def image_callback(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        print(e)
        return
    
    cv2.imshow('image', cv_image)
    cv2.waitKey(1)


def detect_markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    return corners, ids

def publish_tf(frame_id, child_frame_id, translation, rotation):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    # print("imp",(rotation[0], rotation[1], rotation[2]))
    # quat = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    # rotation[0]+= -1.57
    # rotation[1]+= -1.57
    # rotation[2]+= -1.57

    rotation_matrix = np.eye(4)
    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rotation))[0]
    r = R.from_matrix(rotation_matrix[0:3, 0:3])
    quat = r.as_quat() 

    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

def publish_tf_new(frame_id, child_frame_id, translation, rotation):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    quat = quaternion_from_euler(float(rotation[0]), float(rotation[1]), float(rotation[2]))

    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

def main():
    rospy.init_node('aruco_marker_tracker', anonymous=True)
    
    rospy.Subscriber('/rgb/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)

    aruco_marker_pub = rospy.Publisher('/charging_dock_pose', PoseStamped, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)  

    rate=rospy.Rate(5)

    while not rospy.is_shutdown():
        if cv_image is not None:

            corners, ids = detect_markers(cv_image)

            if ids is not None and camera_matrix is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.2, camera_matrix, distortion_coefficients)

                    # rotation = aruco.
                    
                    publish_tf('camera_rgb_optical_frame', f'aruco_marker_{ids[i][0]}', tvec[0][0], rvec[0][0])

                    publish_tf_new(f'aruco_marker_{ids[i][0]}', f'dock_station_{ids[i][0]}', (0,0,0), (-1.5708, -1.5708, 0.0))

                    try:
                        trans = tfBuffer.lookup_transform("odom", f'dock_station_{ids[i][0]}', rospy.Time())
                        # print("trans",euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]))

                        marker_pose=PoseStamped()

                        marker_pose.header.frame_id=f'dock_station_{ids[i][0]}'
                        marker_pose.pose.position = trans.transform.translation
                        marker_pose.pose.orientation = trans.transform.rotation

                        print(marker_pose)

                        aruco_marker_pub.publish(marker_pose)
                        # trans_optical = tfBuffer.lookup_transform("odom", 'camera_rgb_optical_frame', rospy.Time())
                        # print("trans_optical",euler_from_quaternion([trans_optical.transform.rotation.x, trans_optical.transform.rotation.y, trans_optical.transform.rotation.z, trans_optical.transform.rotation.w]))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
