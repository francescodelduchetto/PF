#!/usr/bin/env python2
import yaml
from bayes_people_tracker.msg import PeopleStamped
from strands_navigation_msgs.msg import TopologicalMap 
import numpy as np
global cn
import rospy
import math as mm


global keys, cost, count
flag = False
data = {}
keys = []
cost = []
count = 0
state = {'Names':[]} # Name of picker, Particles of picker

class calibclass():
    def __init__(self):
        self.tmap = None
        self.rec_map = False
        rospy.Subscriber('topological_map', TopologicalMap, self.topo_map_cb)
        self.Nodes = []
        self.coords = {}
        self.XX = []
        self.YY = []
        self.theta = []
#        self.cost = {}
        while not self.rec_map:
            rospy.sleep(0.1)
#        with open(r'clockhouse_node_tags.yaml') as file:
#            documents = yaml.safe_load(file)
#        for ii, doc in documents.items():
#            if ii == '178-179':
#                nodes1 = doc   
#            if ii == '181-182':
#                nodes2 = doc   
#           
#        nodes = nodes1 + nodes2
        nodes =['t2-r0-c1','t2-r0-c4','t2-r0-c5','t2-r0-c6','t2-r0-c7','t2-r0-c8','t2-r0-c20','t2-r0-c21','t2-r0-c22','t2-r0-c23','t2-r0-c24','t2-r3-c1','t2-r3-c3','t2-r3-c4',
't2-r3-c5','t2-r3-c6','t2-r3-c7','t2-r3-c19','t2-r3-c20','t2-r3-c21','t2-r3-c22','t2-r3-c23','t2-r3-c24','t2-r3-c25']
        for i in range(0, len(self.tmap.nodes)):
            if self.tmap.nodes[i].name in nodes:
                self.Nodes.append([self.tmap.nodes[i].name,self.tmap.nodes[i].pose.position.x, self.tmap.nodes[i].pose.position.y])
        self.Node_names = [i[0] for i in self.Nodes]
        self.coords = np.asarray([i[1:] for i in self.Nodes])
        rospy.Subscriber('gps_positions', PeopleStamped, self.calib_cb) 
    
    def topo_map_cb(self, msg):
        """This function receives the Topological Map"""
        self.tmap = msg
        self.rec_map = True      
    
    def calib_cb(self,GPS_msg):  
        global keys, cost, count
        for i in range (0,len(GPS_msg.people)):              
            if GPS_msg.people[i].person.name  == 'picker02':
                count = -1
                for ii in np.arange(-1.5,2.1,.1):
                    for jj in np.arange(-1.5,2.1,1.1):
                        for theta in np.arange(0.04,0.06,0.001):
#                for ii in np.arange(-1.5,1.5,1.5):
#                    for jj in np.arange(-1.5,1.5,1.5):
#                        for theta in np.arange(0,0.4000,0.001):
                            count += 1
                            string = 'X = ' + str(ii) + ', Y = ' + str(jj) + ', theta = ' + str(theta)
                            Q = np.matrix([[mm.cos(theta), -mm.sin(theta)], [mm.sin(theta), mm.cos(theta)]])
                            X = GPS_msg.people[i].person.position.x 
                            Y = GPS_msg.people[i].person.position.y
                            loc = np.matrix((X,Y))
                            loc = np.transpose(loc)
                            location = Q*(loc) 
                            X = location[0] + ii
                            Y = location[1] + jj
                            closest = np.amin(np.sqrt((self.coords[:,0] - float(X))**2  +  (self.coords[:,1] - float(Y))**2))
##                            cn.append(np.amin(np.sqrt((self.coords[:,0] - X)**2  +  (self.coords[:,1] - Y)**2)))
#                            keys.append([string])
#                            cost.append(closest)
                            if string not in keys:
                                keys.append(string)
                                cost.append(closest)
                            else:
                                cost[count] += closest
                            
                            
#                            if string in self.cost:
#                                self.cost[string].append(closest)
#                            else:
#                                self.cost.update({string:[closest]})
#                                keys.append(string)
#                data = self.cost
            else:
                pass



                       
def cleanup():
    global keys, cost, count
    cost[:] = [x / len(cost) for x in cost]
    ind = np.argmin(cost)
    print keys[ind]
    
if __name__ == "__main__":
    rospy.init_node('callibertion', anonymous = True)
    sml = calibclass()
    rospy.on_shutdown(cleanup)
    rospy.spin()






