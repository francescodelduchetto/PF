#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#Nodes = {'A' : [0,0], 'B' : [0,10], 'C' : [0,20], 'D' : [10,0], 'E' : [10,10], 'F' : [10,20]}  #list of Nodes
Nodes = {'A' : [0,0], 'B' : [0,10], 'C' : [0,20], 'D' : [10,0], 'E' : [10,10], 'F' : [10,20], 'G' : [0,30], 'H' : [0,40], 'I' : [0,50], 'J' : [10,30], 'K' : [10,40], 'L' : [10,50]}  #list of Nodes
#Nodes = {'A' : [0,0], 'B' : [0,10]}  #list of Nodes

markerarray=MarkerArray()
rospy.init_node('Display', anonymous = True)
scale = 4
while not rospy.is_shutdown():
    k = 1
    pub = rospy.Publisher('Nodes', MarkerArray, queue_size = 10)
    for i in Nodes:
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE

        
#        marker.action = marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 0.4
        marker.color.r = 255
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = Nodes[i][0]
        marker.pose.position.y = Nodes[i][1]
        marker.pose.position.z = 0
        marker.id =  k + 1
        
        marker2 = Marker()
        marker2.header.frame_id = 'map'
        marker2.type = TEXT_VIEW_FACING = 9
        marker2.text = i
        marker2.scale.x = 0.5 * scale
        marker2.scale.y = 0.5 * scale 
        marker2.scale.z = 0.5 * scale
        marker2.pose.position.x = Nodes[i][0]
        marker2.pose.position.y = Nodes[i][1]
        marker2.pose.position.z = 1 
        marker2.color.a = 1
        marker.color.b = 255
        marker2.id = k + 200
        markerarray.markers.append(marker)
        markerarray.markers.append(marker2)
        k += 1
    pub.publish(markerarray)
    rospy.sleep(0.1)
    
    
        