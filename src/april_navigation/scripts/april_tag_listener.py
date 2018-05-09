#!/usr/bin/env python  

import rospy
import math
import tf
from april_navigation.msg import AprilTag
import geometry_msgs.msg

def check_for_tag(listener, tag_frame):
    """ 
    Given a tf_listener and the name of the tag frame, 
    return its coordinates if it is visible, and None 
    if it is not visible.
    """
    try:
        (trans,rot) = listener.lookupTransform('/map', tag_frame, rospy.Time(0))
        (robot_trans,robot_rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    (x, y, z) = trans
    (rx, ry, rz) = robot_trans

    # Compute target point
    d = 1.0 # m
    sq_dist = ((y - ry)**2 + (x - rx)**2)
    ratio = (d**2) / sq_dist

    return (x + (ratio * (x - rx)), y + (ratio * (y - ry)), sq_dist)


if __name__ == '__main__':
    rospy.init_node('april_tag_listener')

    listener = tf.TransformListener()

    pub = rospy.Publisher('/april_tags_locations', AprilTag, queue_size=10)
    pub2 = rospy.Publisher('/april_tags', geometry_msgs.msg.PointStamped, queue_size=10)
    move_pub = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)

    t = 0

    rate = rospy.Rate(10.0)

    tag_store = {} # Create a hashtable to store locations of tags
    tag_names = ["tag_0", "tag_1", "tag_2", "tag_3", "tag_4", "tag_5"] # Move to config file
    
    
    while not rospy.is_shutdown():
        for tag_name in tag_names:
            coordinates = check_for_tag(listener, tag_name)
            if coordinates is not None:
                # (x, y) = coordinates
                (x, y, dist) = coordinates
                if tag_name not in tag_store:
                    tag_store[tag_name] = coordinates
                else:
                    (prev_x, prev_y, prev_dist) = tag_store[tag_name]
                    if dist <= prev_dist:
                        tag_store[tag_name] = coordinates
                

        for tag_name in tag_store:
            #publish
            tag_loc = AprilTag()
            tag_loc.header.frame_id = "/map"
            tag_loc.id = tag_name
            (x, y, distance) = tag_store[tag_name]
            tag_loc.point.x = x
            tag_loc.point.y = y
            pub.publish(tag_loc)

            tag_loc = geometry_msgs.msg.PointStamped()
            tag_loc.header.frame_id = "/map"
            tag_loc.point.x = x
            tag_loc.point.y = y
            pub2.publish(tag_loc)

        
        print tag_store

        rate.sleep()
