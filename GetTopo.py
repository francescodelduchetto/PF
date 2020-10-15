#!usr/bin/env python
from __future__ import division          # stop rounding off during division
import rospy
from strands_navigation_msgs.msg import TopologicalMap


Waypoints = []
def get_waypoints(msg):
    for i in range(0, len(msg.nodes)):
        Waypoints.append([msg.nodes[i].name,msg.nodes[i].pose.position.x, msg.nodes[i].pose.position.y])
    return(Waypoints)
    

if __name__=='__main__':
    rospy.init_node('prior_dist',anonymous = True)
    rospy.Subscriber('topological_map',TopologicalMap,prob)
    rospy.spin()
    
