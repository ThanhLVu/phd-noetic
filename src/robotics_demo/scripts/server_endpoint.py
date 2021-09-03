#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService, ObjectPoseService
from sensor_msgs.msg import CompressedImage, CameraInfo

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, tcp_server),
	    'image_rgb_1': RosSubscriber('/cam_1/color/image_raw/compressed', CompressedImage, tcp_server),
        'image_depth_lower_1': RosSubscriber('/cam_1/depth/image_lower/compressed', CompressedImage, tcp_server),
        'image_depth_upper_1': RosSubscriber('/cam_1/depth/image_upper/compressed', CompressedImage, tcp_server),
        'image_rgb_2': RosSubscriber('/cam_2/color/image_raw/compressed', CompressedImage, tcp_server),
        'image_depth_lower_2': RosSubscriber('/cam_2/depth/image_lower/compressed', CompressedImage, tcp_server),
        'image_depth_upper_2': RosSubscriber('/cam_2/depth/image_upper/compressed', CompressedImage, tcp_server),
        #'camera_pose': RosSubscriber('/camera/pose', PoseStamped, tcp_server),
        #'headset_pose': RosSubscriber('/headset/pose', PoseStamped, tcp_server),
        'info_rgb_1': RosSubscriber('/cam_1/color/camera_info', CameraInfo, tcp_server),
        'info_depth_1': RosSubscriber('/cam_1/depth/camera_info', CameraInfo, tcp_server),
        'info_rgb_2': RosSubscriber('/cam_2/color/camera_info', CameraInfo, tcp_server),
        'info_depth_2': RosSubscriber('/cam_2/depth/camera_info', CameraInfo, tcp_server),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
