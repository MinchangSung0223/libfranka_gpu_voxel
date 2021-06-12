#!/usr/bin/env python3
import sys
import rospy
import geometry_msgs.msg as gmsg
import sensor_msgs.point_cloud2 as pc2
import tf2_msgs.msg
import numpy as np
import tf.transformations as transformations
import math
import cv2

T = np.loadtxt("TBaseToCamera.txt")
quat = transformations.quaternion_from_matrix(T)

def callback(pc2_msg):
    pc2_msg.header.frame_id ='cam_base_link' #base_cam + '_link'  #
    pub.publish(pc2_msg)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Write a base(center) camera!')
        exit(1)
    elif len(sys.argv) == 2:
        base_cam = sys.argv[1]
        print('Start!')
        rospy.init_node(base_cam, disable_signals=True)
        rospy.Subscriber('/camera/depth/color/points', pc2.PointCloud2, callback)
        #pub = rospy.Publisher('/' + base_cam + '_zf', pc2.PointCloud2, queue_size=10)
        pub = rospy.Publisher('/camera/depth/color/points2', pc2.PointCloud2, queue_size=10)
        pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
             rospy.sleep(0.01)
             t = gmsg.TransformStamped()

             t.header.frame_id = "base_link"
             t.header.stamp = rospy.Time.now()
             t.child_frame_id = "cam_base_link"
             
             t.transform.translation.x = T[0,3];
             t.transform.translation.y = T[1,3];
             t.transform.translation.z = T[2,3];

             t.transform.rotation.x = quat[0];
             t.transform.rotation.y = quat[1];
             t.transform.rotation.z = quat[2];
             t.transform.rotation.w = quat[3];

             tfm = tf2_msgs.msg.TFMessage([t])
             
             pub_tf.publish(tfm)    

    else:
        print('Again!')
