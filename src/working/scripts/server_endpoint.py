#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService, ObjectPoseService
from visualization_msgs.msg import Marker

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        'visualization_markers': RosSubscriber('visualization_markers', Marker, tcp_server),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
