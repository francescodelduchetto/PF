#!/usr/bin/env python
from __future__ import division          # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from scipy.spatial import distance
import collections
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


###############################
state = {'Names':[], 'Particles':[],'Weights':[]}
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions']
Nodes = {'A' : [0,0], 'B' : [0,10], 'C' : [0,20], 'D' : [10,0], 'E' : [10,10], 'F' : [10,20]}  #list of Nodes
coords =[] #coordinates of Nodes
Node_names =[] #names of nodes
No_of_ptcl = 10 #number of particles
ini_w = 1 / No_of_ptcl

for i in Nodes:
    coords.append(Nodes[i])
    Node_names.append(i)
dist_matrix = distance.cdist(coords, coords) #distance matrix w.r.t Nodes coordinates
np.fill_diagonal(dist_matrix,10)
dist_matrix =-0.2 * np.sqrt(dist_matrix)
Pr_p = np.exp((dist_matrix)) #Prediction probabilities based on distance between nodes
row_sums = Pr_p.sum(axis=1)
Pr_p = Pr_p / row_sums[:, np.newaxis] #normaalized predicted probabilities
global timer
timer = 0

Pr_p = np.array([[0.3,0.3,0,0.3,0,0],[0.3,0.3,0.3,0,0,0],[0,0.3,0.3,0,0,0.3],[0.3,0,0,0.3,0.3,0],[0,0,0,0.3,0.3,0.3],[0,0,0.3,0,0.3,0.3]])#Hardcoded probability
Pr_p = Pr_p + 0.05
row_sums = Pr_p.sum(axis=1)
Pr_p = Pr_p / row_sums[:, np.newaxis] #normaalized predicted probabilities
#print Pr_p

#################################

class PFclass():
    def __init__(self):
        self.tracks = None
        rospy.Subscriber(Topics[3], PeopleStamped, self.DR)  
        self.pub = {}
        self.posearray = {}
        
        
    def DR(self,GPS_msg):
        No_of_users = len(GPS_msg.people) #Number of logged users
        for i in range (0,No_of_users):   #Will run only if new user
            global timer
            if GPS_msg.people[i].person.name  not in state['Names']:
#                self.pub[i] = rospy.Publisher('picker%02d/particles'%(i),PoseArray,queue_size=10)
#                self.posearray[i] = PoseArray()
#                self.posearray[i].header.frame_id = '/map'
                state['Names'].append(GPS_msg.people[i].person.name)
                X = GPS_msg.people[i].person.position.x #GPS X location
                Y = GPS_msg.people[i].person.position.y #GPS y location
                node = [X, Y] # Not node GPS coords
                dist_2 = np.sum((np.asarray(coords) - np.asarray(node))**2, axis=1) #distances between GPS location and all nodes
                cn = np.argmin(dist_2)  #closest node to gps location
                particles = [Node_names[cn]] * No_of_ptcl
                state['Particles'].extend([particles])
#                ini_w = 1 / No_of_ptcl
#                weights = [ini_w] * No_of_ptcl
#                state['Weights'].extend([weights])

            else:   #Predict
                self.pub[i] = rospy.Publisher('picker%02d/particles'%(i),PoseArray,queue_size=10)
                self.posearray[i] = PoseArray()
                self.posearray[i].header.frame_id = '/map'
#                X = GPS_msg.people[i].person.position.x #GPS X location
#                Y = GPS_msg.people[i].person.position.y #GPS y location
                X=8
                Y=17
                timer += 1
                if timer > 10:
                    X=0
                    Y=0
                node = [X, Y] # Not node, GPS coords
                for j in range(0,len(state['Particles'])):    #Iterate over number of pickers
                    Pp=[]
                    for q in range(0, No_of_ptcl):            #Iterate over Particle q for picker j  
                        cr = state['Particles'][j][q]         #current node of particles q of picker j
                        P = list(np.random.choice(Node_names, 1, p = Pr_p[Node_names.index(cr)])) #predicted node of qth particle of picker j (select node X with probability Pr_p(J))
                        d = np.sqrt(np.sum((coords[Node_names.index(P[0])] - np.asarray(node))**2))   
                        Pp = Pp + P  
                    counter=collections.Counter(Pp)       #handler to count number of particles in each node
                    nodes_occuppied = np.asarray(counter.most_common(len(counter.keys())))   # how many particles in each node
                    nodes1 = nodes_occuppied[:,0]                                            # Nodes with atleast one particle
                    times = nodes_occuppied[:,1].astype(np.float)                            # Number of particles in Xth node
                    D=[]
                    for k in range (0, len(nodes1)):
                        d = np.sqrt(np.sum((coords[Node_names.index(nodes1[k])]) - (np.asarray(node)))**2)               # distance between GPS reading and nodes1
#                        print d
                        D.append(-0.9 * d)
                    D = np.exp(D)                          # Covert distance to weights
                    norm = np.sum(D)                       # convert distance to weights
                    W = D / norm                           #normalize weight
                    #########################################################
#                    W = W * state['Weights'][j]
#                    norm = np.sum(D)                       # convert distance to weights
#                    W = D / norm 
#                    state['Weights'][j] = W
                    #########################################################
#                    print nodes_occuppied, W
                    est = (W*times).argmax()               # weight x number of particles in a node
#                    print nodes1[est]                                              
                    state['Particles'][j] = np.random.choice(nodes1, No_of_ptcl, p=W)     #Select new particles from predicted with probability W 
                    for m in state['Particles'][j]:
                        pose = Pose()
                        pose.position.x = coords[Node_names.index(m)][0] + np.random.randn(1,1)
                        pose.position.y = coords[Node_names.index(m)][1] + np.random.randn(1,1)
#                        print pose.position
                        self.posearray[i].poses.append(pose)
                    self.pub[i].publish(self.posearray[i])
            
#                    print Node_names.index('A')
#                    print coords[Node_names.index(state['Particles'][j])]
#                    print coords.index(state['Particles'][j])
#                    print state['Particles'][j]
                    
#                    for mm in range(0,len(state['particle'][j])):
#                        pose = Pose()
#                        pose.position.x = state['particles]
#                        pose.position.y


if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    

