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
	'image_rgb': RosSubscriber('/camera/color/image_raw/compressed', CompressedImage, tcp_server),
        'image_depth_lower': RosSubscriber('/camera/depth/image_lower/compressed', CompressedImage, tcp_server),
        'image_depth_upper': RosSubscriber('/camera/depth/image_upper/compressed', CompressedImage, tcp_server),
        #'camera_pose': RosSubscriber('/camera/pose', PoseStamped, tcp_server),
        #'headset_pose': RosSubscriber('/headset/pose', PoseStamped, tcp_server),
        'info_rgb': RosSubscriber('/camera/color/camera_info', CameraInfo, tcp_server),
        'info_depth': RosSubscriber('/camera/depth/camera_info', CameraInfo, tcp_server),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
