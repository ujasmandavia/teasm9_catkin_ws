#!/usr/bin/env python  

import rospy
import geometry_msgs.msg
from move_base_example import GoToPose
from april_navigation.msg import AprilTag
from kill_frontier import kill_frontier
import roslib; roslib.load_manifest('sound_play')
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import tf


tag_store = {}
tag_order = ["tag_0", "tag_1", "tag_2", "tag_3", "tag_4", "tag_5"]
num_tags = 4 #MAGIC NUMBER
following_waypoints = False

def tag_location_callback(tag):
    if tag.id not in tag_store:
        print "I see", tag.id
    tag_store[tag.id] = tag.point
    if len(tag_store) == num_tags and not following_waypoints:
        global following_waypoints
        following_waypoints = True
        follow_waypoints()

def follow_waypoints():
    print "Following Waypoints"
    kill_frontier()
    navigator = GoToPose()
    tags_visited = []
    listener = tf.TransformListener()
    while len(tags_visited) < num_tags:
        for tag_id in tag_order:
            if tag_id in tag_store and tag_id not in tags_visited:
                print "Navigating to", tag_id, "\n\n\n"
                tag = tag_store[tag_id]
        
                try:
                    # Customize the following values so they are appropriate for your location
                    position = {'x': tag.x, 'y' : tag.y}
                    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                
                    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                    success = navigator.goto(position, quaternion)

                    (robot_trans,robot_rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))

                    (rx, ry, rz) = robot_trans

                    sq_dist = (tag.y - ry)**2 + (tag.x - rx)**2
                
                    if success or sq_dist < 1.0:
                        tags_visited.append(tag_id)
                        speak(tag_id)
                        rospy.loginfo("Hooray, reached the desired pose")
                    else:
                        rospy.loginfo("The base failed to reach the desired pose")

                        # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)

                except rospy.ROSInterruptException:
                    rospy.loginfo("Ctrl-C caught. Quitting")


def speak(text):
        soundhandle = SoundClient(blocking=True)
        soundhandle.say(text)

if __name__ == '__main__':
    rospy.init_node('april_follow_waypoints')
    following_waypoints = False
    rospy.Subscriber("/april_tags_locations", AprilTag, tag_location_callback)

    rospy.spin()
