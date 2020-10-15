#!/usr/bin/env python
from __future__ import division          # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
from scipy.spatial import distance

###############################
state = {'Names':[], 'Particles':[],'Weights':[]}
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions']
No_of_ptcl = 100
Nodes = {'A' : [0,0], 'B' : [0,10], 'C' : [0,20], 'D' : [10,0], 'E' : [10,10], 'F' : [10,20]}  #list of Nodes
coords =[] #coordinates of Nodes
Node_names =[] #names of nodes
No_of_ptcl = 5 #number of particles
ini_w = 1 / No_of_ptcl
weights = [ini_w] * No_of_ptcl


for i in Nodes:
    coords.append(Nodes[i])
    Node_names.append(i)
dist_matrix = distance.cdist(coords, coords) #distance matrix w.r.t Nodes coordinates
np.fill_diagonal(dist_matrix,10)
dist_matrix =-1 * np.sqrt(dist_matrix)
Pr_p = np.exp(dist_matrix) #Prediction probabilities based on distance between nodes
row_sums = Pr_p.sum(axis=1)
Pr_p = Pr_p / row_sums[:, np.newaxis] #normaalized predicted probabilities
global timer
timer = 0

#Pr_p =np.array([[0.2,0.3,0.1,0.2,0.1,0.1],[0.2,0.3,0.1,0.2,0.1,0.1],[0.2,0.3,0.1,0.2,0.1,0.1],[0.2,0.3,0.1,0.2,0.1,0.1],[0.2,0.3,0.1,0.2,0.1,0.1],[0.2,0.3,0.1,0.2,0.1,0.1]])#Hardcoded probability
#################################

class PFclass():
    def __init__(self):
        self.tracks = None
        rospy.Subscriber(Topics[3], PeopleStamped, self.DR)   
        
    def DR(self,GPS_msg):
        No_of_users = len(GPS_msg.people) #Number of logged users
        for i in range (0,No_of_users):   #Will run only if new user
            global timer
            if GPS_msg.people[i].person.name not in state['Names']:   # Only true if new picker logs in
                state['Names'].append(GPS_msg.people[i].person.name)
                X = GPS_msg.people[i].person.position.x #GPS X location
                Y = GPS_msg.people[i].person.position.y #GPS y location
#                X = 10
#                Y = 20
                node = [X, Y] # Not node GPS coords
                dist_2 = np.sum((np.asarray(coords) - np.asarray(node))**2, axis=1) #distances between GPS location and all nodes
                cn = np.argmin(dist_2)  #closest node to gps location
                particles = [Node_names[cn]] * No_of_ptcl
                state['Particles'].extend([particles])
#                state['Weights'].extend([weights])

            else:   #Predict
#                X = GPS_msg.people[i].person.position.x #GPS X location
#                Y = GPS_msg.people[i].person.position.y #GPS y location
                X=20
                Y=10
                if timer > 10:
                    X=0
                    
                node = [X, Y] # Not node GPS coords
                dist_2 = np.sqrt(np.sum((np.asarray(coords) - np.asarray(node))**2, axis=1)) #distances between GPS location and all nodes
                weights = np.exp(-1 * dist_2)        # select from predicted particles with this probability
                cn = np.argmin(dist_2)               #closest node to gps location
                for j in range(0,len(state['Particles'])):    #Iterate over number of pickers
                    Pp=[]
#                    Ww=[]
                    for q in range(0, No_of_ptcl):            #Iterate over Particle q for picker j  
                        cr = state['Particles'][j][q]         #current node of particles q of picker j
                        P = list(np.random.choice(Node_names, 1, p = Pr_p[Node_names.index(cr)])) #predicted node of qth particle of picker j (select node X with probability Pr_p(J))
                        Pp = Pp + P
                        
#                        w = Pr_p[Node_names.index(P[0]), cn]
#                        Ww.append(w)                 #new weights
#                    state['Particles'][j] = Pp     #Replace old particles with predicted particle for picker j
#                    state['Weights'][j] = np.asarray(state['Weights'][j]) * np.asarray(Ww)
#                    norm = np.sum(state['Weights'][j])
#                    state['Weights'][j] = state['Weights'][j] /norm
                    parts = np.random.choice(Pp, No_of_ptcl, p=weights)
                    
                    print parts
                    
                    
#                    print state['Weights'][j]
#                    print state['Particles'][j]
#                    print '------------------------'
###                    


if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    
