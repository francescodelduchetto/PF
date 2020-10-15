
#!/usr/bin/env python
####state['Names'] : Name of pickers ####  state['Names'][i] --> Name of i^th picker 
####state['Particles'] : Particles of pickers ####  state['Particles'][i] --> Particles i^th picker  ####state['Particles'][i][q] q^th Particle of i^th picker  



from __future__ import division                                                # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
np.set_printoptions(threshold=sys.maxsize)  # To print the full list/array etc rather than first,.....,last


###############################
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions','topological_map']
state = {'Names':[]} # Name of picker, Particles of picker

global x_p, y_p, marker_id
marker_id=0
#################################
class PFclass():
    def __init__(self):
        self.pub2 = {}
        self.markerarray2 = {}
        rospy.Subscriber(Topics[3], PeopleStamped, self.PF_cb)  
    def PF_cb(self,GPS_msg):
        global marker_id

        No_of_users = len(GPS_msg.people)                                                         #Number of logged users
        for i in range (0,No_of_users):                                                           #Will run only if new user is detected
            if GPS_msg.people[i].person.name  not in state['Names']:                                          #GPS y location
                state['Names'].append(GPS_msg.people[i].person.name)                              # Name of picker 
                   # Create a topic for every picker
#                self.pub2[i] = rospy.Publisher('picker%02d/trajectory'%(i),MarkerArray,queue_size=10)
                self.pub2[i] = rospy.Publisher('%s/trajectory'%(GPS_msg.people[i].person.name),MarkerArray,queue_size=10)

                self.markerarray2[i] = MarkerArray()
            else:   #Predict
                X = GPS_msg.people[i].person.position.x #GPS X location
                Y = GPS_msg.people[i].person.position.y #GPS y location
                  ################GPSMarker###################
                gps_pos = Marker()
                gps_pos.header.frame_id = "/map"
                gps_pos.type = gps_pos.SPHERE
                gps_pos.action = gps_pos.ADD
                gps_pos.pose.position.x = X
                gps_pos.pose.position.y = Y
                gps_pos.pose.position.z = 0.2
                gps_pos.scale.x = 0.3
                gps_pos.scale.y = 0.3
                gps_pos.scale.z = 0.3
                gps_pos.color.a = 1
                gps_pos.color.r = 0
                gps_pos.color.b = 1
                gps_pos.color.g = 0
                gps_pos.id = marker_id
                marker_id += 1
                self.markerarray2[i].markers.append(gps_pos)
                self.pub2[i].publish(self.markerarray2[i])

if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    
    
    
    
    
    