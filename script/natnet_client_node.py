#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
natnet_client_ros.py

Copyright (c) 2017 Takahiro Miki

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
'''

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/natnetclient')

import rospy
import natnetclient as natnet

import tf
from geometry_msgs.msg import Pose


class NatnetClientNode:
    def __init__(self):
        ''' ROS node for natnet client'''
        self.read_parameters()
        try:
            self.client = natnet.NatClient(client_ip=self.client_ip, server_ip=self.server_ip,
                data_port=self.data_port, comm_port=self.comm_port, read_rate=self.read_rate)
        except:
            rospy.ERROR("Error at connecting")

    def read_parameters(self):
        self.client_ip = rospy.get_param("/natnet_client_node/client_ip", '192.168.1.48')
        self.server_ip = rospy.get_param("/natnet_client_node/server_ip", '192.168.1.35')
        self.data_port = rospy.get_param("/natnet_client_node/data_port", 1511)
        self.comm_port = rospy.get_param("/natnet_client_node/comm_port", 1510)
        self.read_rate = rospy.get_param("/natnet_client_node/read_rate", 1200)
        self.publish_rate = rospy.get_param("/natnet_client_node/publish_rate", 100)
        self.rigid_bodies = rospy.get_param('/natnet_client_node/rigid_bodies', None)
        self.pose_publishers = {}
        if self.rigid_bodies is None:
            rospy.logerr("No rigid_bodies parameters")
            exit()
        else:
            for name in self.rigid_bodies.keys():
                key = self.rigid_bodies[name]
                pose_topic_name = key['pose']
                self.pose_publishers[name] = rospy.Publisher(pose_topic_name, Pose, queue_size=0)

    def publish_data(self, e):
        for name in self.rigid_bodies.keys():
            key = self.rigid_bodies[name]
            child_frame_id = key['child_frame_id']
            parent_frame_id = key['parent_frame_id']

            # Publish pose
            body = self.client.rigid_bodies[name]
            pose = Pose()
            pose.position = body.position
            pose.orientation = body.quaternion
            self.pose_publishers[name].publish(pose)

            # Publish tf
            br = tf.TransformBroadcaster()
            br.sendTransform((body.position),
                             body.quaternion,
                             rospy.Time.now(),
                             child_frame_id,
                             parent_frame_id)

        
def main(args):
    '''Initializes ros node'''
    rospy.init_node('Mocap', anonymous=False)
    natnet_node = NatnetClientNode()
    rospy.Timer(rospy.Duration(1.0 / natnet_node.publish_rate), natnet_node.publish_data)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down natnet_client_node")

if __name__ == '__main__':
    main(sys.argv)
