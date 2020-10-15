#!/usr/bin/env python
from __future__ import division          # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from scipy.spatial import distance
import collections
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


###############################
scale = 0.4  #size of particle for displaying
colors = {'red':[], 'green':[], 'blue':[]}
state = {'Names':[], 'Particles':[],'Weights':[]}
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions']
Nodes2 = {'A' : [0,0], 'B' : [0,10], 'C' : [0,20], 'D' : [10,0], 'E' : [10,10], 'F' : [10,20], 'G' : [0,30], 'H' : [0,40], 'I' : [0,50], 'J' : [10,30], 'K' : [10,40], 'L' : [10,50]}  #list of Nodes
Nodes = collections.OrderedDict(sorted(Nodes2.items()))  #Ordered version of Nodes2. Ordered alphabetically
coords =[] #coordinates of Nodes
Node_names =[] #names of nodes
No_of_ptcl = 500 #number of particles
ini_w = 1 / No_of_ptcl
for i in Nodes:
    coords.append(Nodes[i])
    Node_names.append(i)
dist_matrix = distance.cdist(coords, coords) #distance matrix w.r.t Nodes coordinates
np.fill_diagonal(dist_matrix,10)
dist_matrix = -1 * (dist_matrix)
Pr_p = np.exp((dist_matrix)) #Prediction probabilities based on distance between nodes
#row_sums = Pr_p.sum(axis=1)
#Pr_p = Pr_p / row_sums[:, np.newaxis] #normalized predicted probabilities
Pr_p = 0.1 * np.array([[0.3,0.3,0,0.3,0,0,0,0,0,0,0,0],[0.3,0.3,0.3,0,0,0,0,0,0,0,0,0],[0,0.3,0.3,0,0,0,0.3,0,0,0,0,0],[0.3,0,0,0.3,0.3,0,0,0,0,0,0,0],
                 [0,0,0,0.3,0.3,0.3,0,0,0,0,0,0],[0,0,0,0,0.3,0.3,0,0,0,0.3,0,0],[0,0,0.3,0,0,0,0.3,0.3,0,0,0,0],[0,0,0,0,0,0,0.3,0.3,0.3,0,0,0],
                 [0,0,0,0,0,0,0,0.3,0.3,0,0,0.3],[0,0,0,0,0,0.3,0,0,0,0.3,0.3,0],[0,0,0,0,0,0,0,0,0,0.3,0.3,0.3],[0,0,0,0,0,0,0,0,0.3,0,0.3,0.3]])#Hardcoded probability
Pr_p = Pr_p + 0.0001
#Pr_p = Pr_p*Pr_pd
row_sums = Pr_p.sum(axis=1)
Pr_p = Pr_p / row_sums[:, np.newaxis] #normalized predicted probabilities
#print Pr_p

#################################

class PFclass():
    
    def __init__(self):
        self.tracks = None
        rospy.Subscriber(Topics[3], PeopleStamped, self.DR)  
        self.pub = {}
        self.markerarray = {}

    def DR(self,GPS_msg):
        No_of_users = len(GPS_msg.people) #Number of logged users
        for i in range (0,No_of_users):   #Will run only if new user
            
            if GPS_msg.people[i].person.name  not in state['Names']:
                global weights
                colors['red'].append(float(np.random.rand(1,1)))       # generate colors for particles
                colors['green'].append(float(np.random.rand(1,1)))
                colors['blue'].append(float(np.random.rand(1,1)))
                state['Names'].append(GPS_msg.people[i].person.name)
                X = GPS_msg.people[i].person.position.x #GPS X location
                Y = GPS_msg.people[i].person.position.y #GPS y location
                node = [X, Y] # Not node GPS coords
                coords2 = np.asarray(coords)
#                dist_2 = np.sum((np.asarray(coords) - np.asarray(node))**2, axis=1) #distances between GPS location and all nodes
                dist_2 = np.sqrt ((np.asarray(coords2[:,0]) - node[0])**2 + (np.asarray(coords2[:,1]) - node[1])**2 ) # distance between GPS reading and nodes1

                cn = np.argmin(dist_2)  #closest node to gps location
                particles = [Node_names[cn]] * No_of_ptcl
                state['Particles'].extend([particles])
#                ini_w = 1 / No_of_ptcl
#                weights = [ini_w] * No_of_ptcl
                weights = np.empty(len(Nodes))
                weights.fill(1./len(Nodes))


            else:   #Predict
                self.pub[i] = rospy.Publisher('picker%02d/particles'%(i),MarkerArray,queue_size=10)
                self.markerarray[i] = MarkerArray()
                X = GPS_msg.people[i].person.position.x #GPS X location
                Y = GPS_msg.people[i].person.position.y #GPS y location
                node = [X, Y] # Not node, GPS coords
                for j in range(0,len(state['Particles'])):    #Iterate over number of pickers
                    Pp=[]
                    for q in range(0, No_of_ptcl):            #Iterate over Particle q for picker j  
                        cr = state['Particles'][j][q]         #current node of particles q of picker j
                        P = list(np.random.choice(Node_names, 1, p = Pr_p[Node_names.index(cr)])) #predicted node of qth particle of picker j (select node X with probability Pr_p(J)) 
                        Pp = Pp + P  
                    counter=collections.Counter(Pp)       #handler to count number of particles in each node
                    nodes_occuppied = np.asarray(counter.most_common(len(counter.keys())))   # how many particles in each node
                    nodes1 = nodes_occuppied[:,0]                                            # Nodes with atleast one particle
                    times = np.empty(len(Nodes))
                    times.fill(0)
                    time = nodes_occuppied[:,1].astype(np.float)  
                                       
                    D=np.empty(len(Nodes))
                    D.fill(0)
                    
                    for k in range (0, len(nodes1)):       
                        d = np.sqrt(((coords[Node_names.index(nodes1[k])][0]) - node[0])**2 + (coords[Node_names.index(nodes1[k])][1] - node[1])**2 ) # distance between GPS reading and nodes1
                        D[Node_names.index(nodes1[k])] = -1 * d
                        times[Node_names.index(nodes1[k])] = time[k]

                    D = np.exp(D)  * weights                        # Covert distance to weights
                    print times
                    print D
                    norm = np.sum(D)                       # convert distance to weights
                    W = (D / norm)                    #normalize weight
                    weights = W
                    w=[]
                    for k in range (0, len(nodes1)): 
                        w.append(W[Node_names.index(nodes1[k])])
                    norm = np.sum(w)
                    w = w / norm
                        
#                    
                    #########################################################
                    est = (W*times).argmax()               # weight x number of particles in a node  
#                    state['Particles'][j] = np.random.choice(nodes1, No_of_ptcl, replace=True, p=W)     #Select new particles from predicted with probability W 
                    state['Particles'][j] = np.random.choice(nodes1, No_of_ptcl, replace=True, p=w)     #Select new particles from predicted with probability W 

                    ids = 100
                    ########### Particle Marker#######
                    for m in Pp:   ##Predicted particles
#                    for m in state['Particles'][j]:## Updated particles
                        marker = Marker()
                        marker.header.frame_id = "/map"
                        marker.type = marker.SPHERE
                        marker.pose.position.x = coords[Node_names.index(m)][0] + 0.5 * np.random.randn(1,1)    #Updated particles
                        marker.pose.position.y = coords[Node_names.index(m)][1] + 0.5 * np.random.randn(1,1)    #Updated particles
                        marker.pose.position.z = 4
                        marker.scale.x = scale
                        marker.scale.y = scale
                        marker.scale.z = scale
                        marker.color.a = 1
                        marker.color.r = colors['red'][i]
                        marker.color.b = colors['blue'][i]
                        marker.color.g = colors['green'][i]
                        marker.id = ids
                        ids += 1
                        ################GPSMarker###################
                        gps_pos = Marker()
                        gps_pos.header.frame_id = "/map"
                        gps_pos.type = marker.SPHERE
                        gps_pos.pose.position.x = X
                        gps_pos.pose.position.y = Y
                        gps_pos.pose.position.z = 4
                        gps_pos.scale.x = 2
                        gps_pos.scale.y = 2
                        gps_pos.scale.z = 2
                        gps_pos.color.a = 1
                        gps_pos.color.r = 1
                        gps_pos.color.b = 0
                        gps_pos.color.g = 0
                        gps_pos.id = i
                        ############################################
                        ################EstMarker###################
                        est_pos = Marker()
                        est_pos.header.frame_id = "/map"
                        est_pos.type = marker.SPHERE
                        est_pos.pose.position.x = coords[est][0]
                        est_pos.pose.position.y = coords[est][1]
                        
                        est_pos.pose.position.z = 4
                        est_pos.scale.x = 4
                        est_pos.scale.y = 4
                        est_pos.scale.z = 0
                        est_pos.color.a = 0.05
                        est_pos.color.r = 0
                        est_pos.color.b = 1
                        est_pos.color.g = 0
                        est_pos.id = ids + No_of_ptcl
                        ############################################
                        ############################################
                        self.markerarray[i].markers.append(marker)
                        self.markerarray[i].markers.append(gps_pos)
                        self.markerarray[i].markers.append(est_pos)
                    self.pub[i].publish(self.markerarray[i])

if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    

