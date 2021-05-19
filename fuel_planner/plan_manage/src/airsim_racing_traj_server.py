#!/usr/bin/env /root/anaconda3/envs/TDNet/bin/python

import rospy
import airsimdroneracinglab
from bspline.msg import Bspline
def bsplineCallback(data):

if __name__ == "__main__":
    client = airsimdroneracinglab.MultirotorClient(ip="localhost")
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.moveOnSplineAsync([[1,1,1],[2,2,1],[3,3,1]])

    bspline_sub = rospy.Subscriber("planning/bspline", bsplineCallback);
ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
ros::Subscriber pg_T_vio_sub = node.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback);
client.moveToPositionAsync()
client.take
rospy.init_node('airsim_publisher', anonymous=True)
    publisher_d = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
    publisher_rgb = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=1)
    publisher_info = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)
    publisher_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
    rate = rospy.Rate(30)  # 30hz
    pub = KinectPublisher()

    while not rospy.is_shutdown():
        responses = client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.DepthPlanar, True, False),
                                         airsim.ImageRequest(0, airsim.ImageType.Scene, False, False)])
        img_depth = pub.getDepthImage(responses[0])
        img_rgb = pub.getRGBImage(responses[1])

        if CLAHE_ENABLED:
            img_rgb = pub.enhanceRGB(img_rgb)

        pub.GetCurrentTime()
        msg_rgb = pub.CreateRGBMessage(img_rgb)
        msg_d = pub.CreateDMessage(img_depth)
        msg_info = pub.CreateInfoMessage()
        msg_tf = pub.CreateTFMessage()

        publisher_rgb.publish(msg_rgb)
        publisher_d.publish(msg_d)
        publisher_info.publish(msg_info)
        publisher_tf.publish(msg_tf)

        del pub.msg_info.D[:]
        del pub.msg_tf.transforms[:]

        rate.sleep()