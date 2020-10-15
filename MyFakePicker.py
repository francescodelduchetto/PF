#!/usr/bin/env python
import rospy
from bayes_people_tracker.msg import PeopleStamped
from people_msgs.msg import PersonStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np



No_of_pickers = 1
scale = 0.8
duration = 100
x_ini = 0
y_ini = 0
var_x = 3
var_y = 3
velx = 1
vely = 1
#vx = 1
#vy = 0
t = 1
No_of_pickers = 1
rospy.init_node('FakePicker', anonymous = True)
iteration = 0
rate = 5

pub = rospy.Publisher('gps_positions',PeopleStamped,queue_size = 10)
pub2 =rospy.Publisher('Fake_Picker_gt',MarkerArray,queue_size = 10)
T = rospy.get_rostime()
while not rospy.is_shutdown(): 
    peoplestamped = PeopleStamped()
    peoplestamped.header.frame_id = '/map'
    personstamp=PersonStamped() 
    personstamp.header.frame_id = '/map'
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1
    marker.color.r = 0
    marker.color.b = 0
    marker.color.g = 1
    marker.id = 1
#    ids += 1
    
#    if iteration < 120:
        
    if iteration <50:
        iteration += 1
        vy = vely
        vx = 0
        xt = x_ini + vx * t 
        yt = y_ini + vy * t
        x_ini = xt
        y_ini = yt
        x = xt + var_x * np.random.randn(1,1)
        y = yt + var_y * np.random.randn(1,1)

#    if iteration >=20 and iteration < 50:
#        iteration += 1
#        vy = 1
#        vx = 0
#        xt = 10 
#        yt = y_ini + vy * t
#        x_ini = xt
#        y_ini = yt
#        x = xt + var_x * np.random.randn(1,1)
#        y = yt + var_y * np.random.randn(1,1)
     
    if iteration >= 50 and iteration < 60:
        iteration += 1
        vy = 0
        vx = velx
        xt = x_ini + vx * t
        yt = y_ini + vy * t
        x_ini = xt
        y_ini = yt
        x = xt + var_x * np.random.randn(1,1)
        y = yt + var_y * np.random.randn(1,1)
    if iteration >= 60 and iteration < 110:
        iteration += 1
        vy = -1 * vely
        vx = 0
        xt = x_ini + vx * t
        yt = y_ini + vy * t
        x_ini = xt
        y_ini = yt
        x = xt + var_x * np.random.randn(1,1)
        y = yt + var_y * np.random.randn(1,1)
    if iteration >= 110 and iteration < 120:
        iteration += 1
        vy = 0
        vx = -1 * velx
        xt = x_ini + vx * t
        yt = y_ini + vy * t
        x_ini = xt
        y_ini = yt
        x = xt + var_x * np.random.randn(1,1)
        y = yt + var_y * np.random.randn(1,1)
    if iteration >= 120:
        iteration = 0
        
        
        
    personstamp.person.name = 'Faker'
    personstamp.person.position.x = x
    personstamp.person.position.y =  y 
    personstamp.person.position.z = 4
    personstamp.header.stamp = rospy.get_rostime()
    peoplestamped.people.append(personstamp)
    marker.pose.position.x = xt
    marker.pose.position.y = yt
    marker.pose.position.z = 8
    marker_array.markers.append(marker) 
    if (rospy.get_rostime().secs - T.secs) > 5:
        pub.publish(peoplestamped)
        pub2.publish(marker_array)
        T = rospy.get_rostime()
    rospy.sleep(1)