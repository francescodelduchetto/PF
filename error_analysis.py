#!/usr/bin/env python 



from __future__ import division                                                # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from strands_navigation_msgs.msg import TopologicalMap
import sys
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from pred_prob_gen import prob
np.set_printoptions(threshold=sys.maxsize)  # To print the full list/array etc rather than first,.....,last
#time = [1571396435, 1571396551, 1571396580, 1571396610, 1571396640, 1571396690, 1571397249, 1571397279, 1571397299, 1571397318, 1571397375,
#        1571399136, 1571399240, 1571399260, 1571399291, 1571399380, 1571399410, 1571399746, 1571399776, 1571399806, 1571399821, 1571399880,
#        1571399908, 1571399881]  #GMT #1571392835, 
time = [1571392835, 1571392951, 1571392978, 1571393010, 1571393041, 1571393091, 
         1571393649, 1571393679, 1571393699, 1571393718, 1571393775,
         1571395536, 1571395639, 1571395661, 1571395691, 1571395781, 1571395811, 
         1571396146, 1571396176, 1571396206, 1571396221, 1571396280, 1571396308, 1571396341]
x_loc = [-53.8387,    -54.5,         -55.3437, -56.073, -56.808,-57.5088,#-58.257,
                  -64.1,             -64.8, -65.6, -66.4, -67.1, #-67.8, 
         -57.5185,    -58.2,       -59.0296,-59.7544,-60.4893,-61.2344,#-61.9908,
                     -67.2,         -67.9, -68.6, -69.4, -70.1, -70.8, -71.6,#-72.3
         ]
y_loc = [19.2091,  16.3,     13.3872,10.4411,7.53086,4.68491,#1.76526,
               -21.5 ,        -24.4,-27.3,-30.3,-33.2,#-36.4,
         20.2219, 17.2 ,     14.4124,11.5069,8.62515,5.73022,#2.80137,
               -17.6,         -20.5,-23.4,-26.3,-29.2,-32.1,-35,#-37.9
         ]
#time=[1571392963, 1571393004, 1571393030,                #t2-r1 (camera 1)
#      1571393642, 1571393699, 1571393732,                #t2-r1 (camera 2)
#      1571394301, 1571394367, 1571394403,                #t2-r1 (camera 3)  
#
#      1571395657, 1571395695, 1571395753,                #t2-r2 (camera 1)
#      1571396200, 1571396242, 1571396277,                #t2-r2 (camera 2)
#      1571396630, 1571396682, 1571396715                 #t2-r2 (camera 3)
#]
#x_loc=[-57.3, -58.0, -58.7,                              #t2-r1 (camera 1)
#       -67.6, -68.3, -69.1,                              #t2-r1 (camera 2)
#       -75.08, -75.8, -76.5,                             #t2-r1 (camera 3)
#
#       -58.4, -59.2, -60.0,                              #t2-r2 (camera 1)
#       -68.9, -69.6, -70.4,                              #t2-r2 (camera 2)
#       -76.3, -77.08, -77.82                             #t2-r2 (camera 3)
#
#
#]
#y_loc=[10.7, 7.9, 5.0,                                   #t2-r1 (camera 1)
#       -29.9, -32.8, -35.7,                              #t2-r1 (camera 2)
#       -59.05, -62.0, -64.9,                             #t2-r1 (camera 3)
#
#       11.2, 8.2, 5.3,                                   #t2-r2 (camera 1)
#       -29.6, -32.5, -35.0 ,                              #t2-r2 (camera 2)
#       -58.7, -61.6, -64.5                               #t2-r2 (camera 3)
#     
#]

posearray = PoseArray()
posearray.header.frame_id = '/map'
global den
den = 0

class Errorclass():
    def __init__(self):
        self.tmap = None
        self.rec_map = False
        rospy.Subscriber('gps_positions', PeopleStamped, self.PF_cb)  
        rospy.Subscriber('topological_map', TopologicalMap, self.topo_map_cb)
        self.pub = rospy.Publisher('Ground_truth',PoseArray,queue_size=10)
        self.pub2 = rospy.Publisher('Closest_node_gps',PoseArray,queue_size=10)
        while not self.rec_map:
            rospy.sleep(0.1)
        self.Pr_p, self.Pr_f, self.Pr_b, self.Nodes = prob(self.tmap) 
        self.Node_names = [i[0] for i in self.Nodes]
        self.coords = np.asarray([i[1:] for i in self.Nodes])
        self.error = 0
    def topo_map_cb(self, msg):
        """This function receives the Topological Map"""
        self.tmap = msg
        self.rec_map = True
        
    def PF_cb(self,GPS_msg):
        global den
        if GPS_msg.people[0].header.stamp.secs in time:
            posearray = PoseArray()
            posearray.header.frame_id = '/map'
            posearray2 = PoseArray()
            posearray2.header.frame_id = '/map'
            X = GPS_msg.people[0].person.position.x                                           #GPS X location
            Y = GPS_msg.people[0].person.position.y     
            cn = np.argmin(np.sqrt((self.coords[:,0] - X)**2  +  (self.coords[:,1] - Y)**2))  # closest node to gps location
            xx = self.coords[cn,0]
            yy = self.coords[cn,1]
            timestamp = GPS_msg.people[0].header.stamp.secs
            idx = time.index(timestamp)
            gx = x_loc[idx]
            gy = y_loc[idx]
            e = (xx - gx)**2 + (yy - gy)**2
            self.error = self.error + e
            den += 1
            print self.error, timestamp, '---->',den
#            gcn = np.argmin(np.sqrt((self.coords[:,0] - gx)**2  +  (self.coords[:,1] - gy)**2))  
#            print self.Node_names[cn],'<----->',self.Node_names[gcn]
  
            pose = Pose()
            pose.position.x = xx
            pose.position.y = yy
            pose.position.z = 1
            posearray.poses.append(pose)
            pose2 = Pose()
            pose2.position.x = gx
            pose2.position.y = gy
            pose2.position.z = 1
            posearray2.poses.append(pose2)
            

            self.pub2.publish(posearray)
            self.pub.publish(posearray2)
            time.pop(0)
            x_loc.pop(0)
            y_loc.pop(0)
#            rospy.sleep(1000)
  

if __name__ == '__main__':
    rospy.init_node('Error', anonymous = True)
    sml = Errorclass()
    rospy.spin()