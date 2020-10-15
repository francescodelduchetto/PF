from __future__ import division                                                # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from GetTopo import get_waypoints
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.srv import GetTaggedNodes
import sys
np.set_printoptions(threshold=sys.maxsize)  # To print the full list/array etc rather than first,.....,

Topics = ['gps_positions','topological_map']
active_pickers = []
colors = {'red':[], 'green':[], 'blue':[]}
Waypoints = []


class simplelocalizer():
    def __init__(self):
        self.tmap = None
        self.rec_map = False
        self.pub = {}
        self.markerarray = {}
        rospy.Subscriber(Topics[1], TopologicalMap, self.topo_map_cb)
        while not self.rec_map:
            rospy.sleep(0.1)
        self.Nodes = get_waypoints(self.tmap) 
        self.Node_names = [i[0] for i in self.Nodes]
        self.coords = np.asarray([i[1:] for i in self.Nodes])
        rospy.Subscriber(Topics[0], PeopleStamped, self.GPS_cb)  

    def topo_map_cb(self, msg):
        self.tmap = msg
        self.rec_map = True

    def GPS_cb(self,gdata):
        Num_of_pickers = len(gdata.people)
        for i in range(0,Num_of_pickers):
            if (gdata.people[i].person.name  not in active_pickers):
#                colors['red'].append(float(np.random.rand(1,1)))                                  # generate colors for particles of i^th picker
#                colors['green'].append(float(np.random.rand(1,1)))
#                colors['blue'].append(float(np.random.rand(1,1)))
                active_pickers.append(gdata.people[i].person.name)
                self.pub[i] = rospy.Publisher('picker%02d/closestnode'%(i),MarkerArray,queue_size=10)
                
            else:
                self.markerarray[i] = MarkerArray()
                X = gdata.people[i].person.position.x                                           #GPS X location
                Y = gdata.people[i].person.position.y  
                row = gdata.people[i].person.tags
                print row
                str1 = ''.join(row)
                print str1
                rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
                handle = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes',GetTaggedNodes)
                rep = handle(str1)
                D=[]
                for j in range(0,len(rep.nodes)):
                    d = np.sqrt((self.coords[self.Node_names.index(rep.nodes[j])][0] - X)**2 + (self.coords[self.Node_names.index(rep.nodes[j])][1] - Y)**2)
                    D.append(d)
                try:
                    closest_node = rep.nodes[np.argmin(D)]
                    #print gdata.people[i].person.name, closest_node, self.coords[self.Node_names.index(closest_node)][0], self.coords[self.Node_names.index(closest_node)][1]
                    X = self.coords[self.Node_names.index(closest_node)][0]
                    Y = self.coords[self.Node_names.index(closest_node)][1]
                except:
                    print "row ids mismatch"
                
                est_pos = Marker()
                est_pos.header.frame_id = "/map"
                est_pos.type = est_pos.SPHERE
                est_pos.pose.position.x = X
                est_pos.pose.position.y = Y
                est_pos.pose.position.z = 4
                est_pos.scale.x = 1
                est_pos.scale.y = 1
                est_pos.scale.z = 0
                est_pos.color.a = 1
                est_pos.color.r = 0
                est_pos.color.b = 1
                est_pos.color.g = 0
                est_pos.id = i
                self.markerarray[i].markers.append(est_pos)
                self.pub[i].publish(self.markerarray[i])


if __name__ == '__main__':
    rospy.init_node('Pickers', anonymous = True)
    sml = simplelocalizer()
    rospy.spin()