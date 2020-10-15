
#!/usr/bin/env python
####state['Names'] : Name of pickers ####  state['Names'][i] --> Name of i^th picker 
####state['Particles'] : Particles of pickers ####  state['Particles'][i] --> Particles i^th picker  ####state['Particles'][i][q] q^th Particle of i^th picker  



from __future__ import division                                                # stop rounding off during division
import numpy as np
import rospy
from bayes_people_tracker.msg import PeopleStamped
import collections
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from pred_prob_gen import prob, normalize
from strands_navigation_msgs.msg import TopologicalMap
import sys
np.set_printoptions(threshold=sys.maxsize)  # To print the full list/array etc rather than first,.....,last


###############################
scale = 0.1  #size of particle for displaying
colors = {'red':[], 'green':[], 'blue':[]}
state = {'Names':[], 'Particles':[], 'daddy_particles':[],'life':[]} # Name of picker, Particles of picker
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions','topological_map']
parameters = {'duration':[],'timestamp':[],'time_span':[]}
No_of_ptcl = 100
lambdaa = 0.7                                   
meas_power = 0.3     # weight given to gps measurements
meas_power2 = 0.1
global x_p, y_p, marker_id
marker_id=0

#################################
class PFclass():
    def __init__(self):
        self.tmap = None
        self.rec_map = False
        self.pub = {}
        self.pub2 = {}
        self.markerarray = {}
        self.markerarray_est = {}
        self.closest_node = {}
        self.cn = {}
        self.X = {}
        self.Y ={}
        rospy.Subscriber(Topics[3], PeopleStamped, self.PF_cb)  
        rospy.Subscriber(Topics[4], TopologicalMap, self.topo_map_cb)
        while not self.rec_map:
            rospy.sleep(0.1)
        self.Pr_p, self.Nodes = prob(self.tmap) 
        self.Node_names = [i[0] for i in self.Nodes]
        self.coords = np.asarray([i[1:] for i in self.Nodes])

    def topo_map_cb(self, msg):
        """This function receives the Topological Map"""
        self.tmap = msg
        self.rec_map = True

    def PF_cb(self,GPS_msg):
        global x_p, y_p, marker_id
        No_of_users = len(GPS_msg.people)                                                         #Number of logged users
        for i in range (0,No_of_users):                                                           #Will run only if new user is detected
            if GPS_msg.people[i].person.name  not in state['Names']:
                self.closest_node[GPS_msg.people[i].person.name] = 'none'
                self.X[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.x                                           #GPS X location
                self.Y[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.y                                           #GPS y location
                self.cn[GPS_msg.people[i].person.name] = np.argmin(np.sqrt((self.coords[:,0] - self.X[GPS_msg.people[i].person.name])**2  +  (self.coords[:,1] - self.Y[GPS_msg.people[i].person.name])**2))  # closest node to gps location
                colors['red'].append(float(np.random.rand(1,1)))                                  # generate colors for particles of i^th picker
                colors['green'].append(float(np.random.rand(1,1)))
                colors['blue'].append(float(np.random.rand(1,1)))
                state['Names'].append(GPS_msg.people[i].person.name)                              # Name of picker 
                state['Particles'].extend([[self.Node_names[self.cn[GPS_msg.people[i].person.name]]] * No_of_ptcl])                   # Create "No_of_ptcl" number of particles at cn
                state['daddy_particles'].extend([[self.Node_names[self.cn[GPS_msg.people[i].person.name]]] * No_of_ptcl])             # For timestep 1 daddy particles are same as Particles
                parameters['duration'].append([0]*No_of_ptcl)                                     # This goes into the power of exponential e.g exp^(parameter['duration'])
                parameters['time_span'].append([0]*No_of_ptcl)                                    # This is the time a particle have been in a node
                parameters['timestamp'].append(GPS_msg.people[i].header.stamp.secs)               # This is the time at which GPS measurement was received
                state['life'].append([0] * No_of_ptcl)                                            # This is the pdf of particles
                self.pub[i] = rospy.Publisher('%s/PF_trajectory'%(GPS_msg.people[i].person.name),MarkerArray,queue_size=10)
                self.pub2[i] = rospy.Publisher('%s/particles'%(GPS_msg.people[i].person.name),MarkerArray,queue_size=10)
                x_p = self.X[GPS_msg.people[i].person.name]
                y_p = self.Y[GPS_msg.people[i].person.name]
                self.markerarray_est[i] = MarkerArray()

            elif self.closest_node[GPS_msg.people[i].person.name] == self.cn[GPS_msg.people[i].person.name]:
                self.X[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.x                                           #GPS X location
                self.Y[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.y 
                self.cn[GPS_msg.people[i].person.name] = np.argmin(np.sqrt((self.coords[:,0] - self.X[GPS_msg.people[i].person.name])**2  +  (self.coords[:,1] - self.Y[GPS_msg.people[i].person.name])**2))
                #print self.cn[GPS_msg.people[i].person.name]
            elif self.closest_node[GPS_msg.people[i].person.name] != self.cn[GPS_msg.people[i].person.name]:   #Predict
                self.closest_node[GPS_msg.people[i].person.name] = self.cn[GPS_msg.people[i].person.name]
               # self.cn[GPS_msg.people[i].person.name] = np.argmin(np.sqrt((self.coords[:,0] - self.X[GPS_msg.people[i].person.name])**2  +  (self.coords[:,1] - self.Y[GPS_msg.people[i].person.name])**2))
                Pp=[]
                self.markerarray[i] = MarkerArray()
                self.X[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.x                                           #GPS X location
                self.Y[GPS_msg.people[i].person.name] = GPS_msg.people[i].person.position.y 
               # X = GPS_msg.people[i].person.position.x #GPS X location
               # Y = GPS_msg.people[i].person.position.y #GPS y location
                dt = GPS_msg.people[i].header.stamp.secs -  np.asarray(parameters['timestamp'][i])
                parameters['timestamp'][i] = GPS_msg.people[i].header.stamp.secs
                ###############
                for q in range(0, No_of_ptcl):            #Iterate over Particle q for picker i 
                    if state['Particles'][i][q] == state['daddy_particles'][i][q]:   # Check if particle is still in the same node
                        parameters['time_span'][i][q] += dt/100 
                    else: 
                        parameters['time_span'][i][q] =   dt/100
                        state['daddy_particles'][i][q] = state['Particles'][i][q]
                parameters['duration'][i] = [o * -1 * lambdaa for o in parameters['time_span'][i]]     
                state['life'][i] = lambdaa * np.exp(parameters['duration'][i])                            
                for q in range(0,No_of_ptcl):
                    P_q = state['Particles'][i][q]         #particle q              
                    D_q = state['life'][i][q]              #its duration in current node (In form of probability)
                    indx = self.Node_names.index(P_q)           #Index of P_q, to choose correct row of Pr_p
                    Prp = self.Pr_p.copy()
                    P_p_q = Prp[indx]                     # row of Pr_p corresponding to P_q  
                    for n in range(0,len(P_p_q)): 
                        if (P_p_q[n] == 1 and indx != n):       #probability of leaving current node
                            P_p_q[n] =  lambdaa-D_q 
                        elif (P_p_q[n] == 1 and indx == n):     #probability of staying in current node
                            P_p_q[n] = D_q
                        elif P_p_q[n] == 0.1:                   #probaility of all other jumps to nodes in vacinity
                            P_p_q[n] = 0.002#0.01  
                    P_p_q = normalize(P_p_q) 
                    P = list(np.random.choice(self.Node_names, 1, p = P_p_q))
                    Pp = Pp + P                                                   #predicted particles
                ###############           
                counter=collections.Counter(Pp)       #handler to count number of particles in each node
                nodes_occuppied = np.asarray(counter.most_common(len(counter.keys())))   # how many particles in each node
                nodes = nodes_occuppied[:,0]                                            # Nodes with atleast one particle
                times = nodes_occuppied[:,1].astype(np.float)                            # Number of particles in Xth node 
                D=[]
                for k in range (0, No_of_ptcl):       
                    d = np.sqrt((self.coords[self.Node_names.index(Pp[k])][0] - self.X[GPS_msg.people[i].person.name])**2  +  (self.coords[self.Node_names.index(Pp[k])][1] - self.X[GPS_msg.people[i].person.name])**2)# distance between GPS reading and nodes1 
                    #d = 10;
                    if Pp[k] == 0.01:
                        D.append(-1 * meas_power2 * d)
                    else:
                        D.append(-1 * meas_power * d)
                D = np.exp(D)                         # Covert distance to weightss
                W = normalize(D)
                Neff = 1. / np.sum(np.square(W))
                if Neff < No_of_ptcl/5:
                    W = [1/No_of_ptcl]*No_of_ptcl
                W_n=[]    #weights of occupied nodes
                for jj in range(0,len(nodes)):
                    occuppiednode = nodes[jj]
                    times_occuppied = times[jj]
                    weightgiven = W[Pp.index(occuppiednode)]
                    totalweight = times_occuppied * weightgiven
                    W_n.append(totalweight)
                est = np.asarray(W_n).argmax()
                W_n = normalize(np.asarray(W_n))
                state['Particles'][i] = np.random.choice(nodes, No_of_ptcl, replace=True, p=W_n)
                #########################################################
#                est = (W*times).argmax()               # weight x number of particles in a node  
               # if X != x_p or Y != y_p:
                #    state['Particles'][i] = np.random.choice(nodes, No_of_ptcl, replace=True, p=W_n)     #Select new particles from predicted with probability W 
                 #   x_p = X
                  #  y_p = Y
                ########### Particle Marker#######
                ids = 100
                for m in Pp:   ##Predicted particles
#                for m in state['Particles'][i]:## Updated particles
                    marker = Marker()
                    marker.header.frame_id = "/map"
                    marker.type = marker.SPHERE
                    marker.pose.position.x = self.coords[self.Node_names.index(m)][0] + 0.2 * np.random.randn(1,1)    #Updated particles
                    marker.pose.position.y = self.coords[self.Node_names.index(m)][1] + 0.2 * np.random.randn(1,1)    #Updated particles
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
                    self.markerarray[i].markers.append(marker)
    
                    
                        ############################################
                ################EstMarker###################
                est_pos = Marker()
                est_pos.header.frame_id = "/map"
                est_pos.type = est_pos.SPHERE
                est_pos.pose.position.x = self.coords[self.Node_names.index(nodes[est])][0]
                est_pos.pose.position.y = self.coords[self.Node_names.index(nodes[est])][1]  
                est_pos.pose.position.z = 1
                est_pos.scale.x = 1.5
                est_pos.scale.y = 1.5
                est_pos.scale.z = 0
                est_pos.color.a = 0.1
                est_pos.color.r = 0
                est_pos.color.b = 1
                est_pos.color.g = 0
                est_pos.id = marker_id
                marker_id += 1
                self.markerarray_est[i].markers.append(est_pos)
                self.pub[i].publish(self.markerarray[i])
                self.pub2[i].publish(self.markerarray_est[i])

if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    
    
    
    
    
    