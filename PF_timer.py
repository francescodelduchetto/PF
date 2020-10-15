
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
scale = 0.05  #size of particle for displaying
colors = {'red':[], 'green':[], 'blue':[]}
state = {'Names':[], 'Particles':[], 'daddy_particles':[],'life':[]} # Name of picker, Particles of picker
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions','topological_map']
parameters = {'duration':[],'timestamp':[],'time_span':[],'W_p':[]}
No_of_ptcl = 300
lambdaa = 0.1                                
meas_power = 0.5#0.5# 0.2 * (lambdaa)     # weight given to gps measurements
meas_power2 = 0.02
global x_p, y_p, marker_id, den
marker_id=0
den=0

#         
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
#       -68.9, -69.6, -70.4,                               #t2-r2 (camera 2)
#       -76.3, -77.08, -77.82                             #t2-r2 (camera 3)
#
#
#]
#y_loc=[10.7, 7.9, 5.0,                                   #t2-r1 (camera 1)
#       -29.9, -32.8, -35.7,                              #t2-r1 (camera 2)
#       -59.05, -62.0, -64.9,                             #t2-r1 (camera 3)
#
#       11.2, 8.2, 5.3,                                   #t2-r2 (camera 1)
#       -29.6, -32.5, -35.0,                               #t2-r2 (camera 2)
#       -58.7, -61.6, -64.5                               #t2-r2 (camera 3)
#     
#]
#################################
class PFclass():
    def __init__(self):
        self.tmap = None
        self.rec_map = False
        self.pub = {}
        self.pub2 = {}
        self.markerarray = {}
        self.markerarray_est = {}
        self.closest_nodes = {}
        self.direction = {}
        self.velocities = {}
        self.GPS_positions = {}
        self.lambdaas = {}
        self.timestamps = {}
        rospy.Subscriber(Topics[3], PeopleStamped, self.PF_cb)  
        rospy.Subscriber(Topics[4], TopologicalMap, self.topo_map_cb)
        while not self.rec_map:
            rospy.sleep(0.1)
#        self.Pr_p, self.Nodes = prob(self.tmap)
        self.Pr_p, self.Pr_f, self.Pr_b, self.Nodes = prob(self.tmap) 
        self.Node_names = [i[0] for i in self.Nodes]
        self.coords = np.asarray([i[1:] for i in self.Nodes])
    def topo_map_cb(self, msg):
        """This function receives the Topological Map"""
        self.tmap = msg
        self.rec_map = True

    def PF_cb(self,GPS_msg):
        global marker_id, den
        
        No_of_users = len(GPS_msg.people)                                                         #Number of logged users
        for i in range (0,No_of_users):                                                           #Will run only if new user is detected
            if GPS_msg.people[i].person.name  not in state['Names']:
                X = GPS_msg.people[i].person.position.x                                           #GPS X location
                Y = GPS_msg.people[i].person.position.y     
                cn = np.argmin(np.sqrt((self.coords[:,0] - X)**2  +  (self.coords[:,1] - Y)**2))  # closest node to gps location
                self.GPS_positions[GPS_msg.people[i].person.name] = [X, Y]
                self.closest_nodes[GPS_msg.people[i].person.name] = self.Node_names[cn]
                state['Names'].append(GPS_msg.people[i].person.name)                              # Name of picker 
                state['Particles'].extend([[self.Node_names[cn]] * No_of_ptcl])                   # Create "No_of_ptcl" number of particles at cn
                state['daddy_particles'].extend([[self.Node_names[cn]] * No_of_ptcl])             # For timestep 1 daddy particles are same as Particles
                state['life'].append([0] * No_of_ptcl)
                parameters['duration'].append([0]*No_of_ptcl)                                     # This goes into the power of exponential e.g exp^(parameter['duration'])
                parameters['time_span'].append([0]*No_of_ptcl)                                    # This is the time a particle have been in a node
                parameters['W_p'].append([1/No_of_ptcl]*No_of_ptcl)
#                parameters['timestamp'].append(GPS_msg.people[i].header.stamp.secs)               # This is the time at which GPS measurement was received
                self.timestamps[GPS_msg.people[i].person.name] = GPS_msg.people[i].header.stamp.secs           
#                self.pub[i] = rospy.Publisher('picker%02d/particles'%(i),MarkerArray,queue_size=10)        # Create a topic for every picker
#                self.pub2[i] = rospy.Publisher('picker%02d/trajectory'%(i),MarkerArray,queue_size=10)
                self.pub[i] = rospy.Publisher('%s/PF_trajectory'%(GPS_msg.people[i].person.name),MarkerArray,queue_size=10)
                self.pub2[i] = rospy.Publisher('%s/particles'%(GPS_msg.people[i].person.name),MarkerArray,queue_size=10)
                self.markerarray_est[i] = MarkerArray()
                self.direction[GPS_msg.people[i].person.name] = True
                self.velocities[GPS_msg.people[i].person.name] = None
                self.lambdaas[GPS_msg.people[i].person.name] = 0.5
                colors['red'].append(float(np.random.rand(1,1)))                                  # generate colors for particles of i^th picker
                colors['green'].append(float(np.random.rand(1,1)))
                colors['blue'].append(float(np.random.rand(1,1)))
                self.error=0
                self.estimated = [0,0]
                

            else:   #Predict
                if (GPS_msg.people[i].person.position.x > -5000 and GPS_msg.people[i].person.position.x < 5000):
                    Pp=[]
                    self.markerarray[i] = MarkerArray()
                    X = GPS_msg.people[i].person.position.x #GPS X location
                    Y = GPS_msg.people[i].person.position.y #GPS y location
                    dt = GPS_msg.people[i].header.stamp.secs -  self.timestamps[GPS_msg.people[i].person.name]
                    self.timestamps[GPS_msg.people[i].person.name] = GPS_msg.people[i].header.stamp.secs
                    if dt != 0:
                        self.velocities[GPS_msg.people[i].person.name] = ((self.GPS_positions[GPS_msg.people[i].person.name][0] - X)**2 + (self.GPS_positions[GPS_msg.people[i].person.name][1] - Y)**2)/dt    
                        self.lambdaas[GPS_msg.people[i].person.name] = abs(self.velocities[GPS_msg.people[i].person.name])
                    self.GPS_positions[GPS_msg.people[i].person.name][0] = X
                    self.GPS_positions[GPS_msg.people[i].person.name][1] = Y
        #                ###############
                    for q in range(0, No_of_ptcl):            #Iterate over Particle q for picker i 
                        if state['Particles'][i][q] == state['daddy_particles'][i][q]:   # Check if particle is still in the same node
                            parameters['time_span'][i][q] += dt
                        else: 
                            parameters['time_span'][i][q] =   dt
                            state['daddy_particles'][i][q] = state['Particles'][i][q]
#                        print parameters['time_span'][i][q]
                    parameters['duration'][i] = [o * -1 * lambdaa for o in parameters['time_span'][i]]     
                    state['life'][i] = 1 - np.exp(parameters['duration'][i])    #lambdaa * np.exp(parameters['duration'][i])  
#                    print state['life'][i] 
                   # state['life'][i]  = normalize(state['life'][i])
                   # print state['life'][i] 

                    ##########################################################

                    cn = np.argmin(np.sqrt((self.coords[:,0] - X)**2  +  (self.coords[:,1] - Y)**2))
                    current_node = self.Node_names[cn]
                    ind_closest_node = self.closest_nodes[GPS_msg.people[i].person.name].find('c')
                    ind_current_node = self.Node_names[cn].find('c')
                    v1 = self.closest_nodes[GPS_msg.people[i].person.name][ind_closest_node+1:]
                    v2 = current_node[ind_current_node+1:]
                    try:
                        if int(v2)  != int(v1):# current_node != self.closest_nodes[GPS_msg.people[i].person.name]:
                            if (int(v2) - int(v1)) < 0:
                                self.direction[GPS_msg.people[i].person.name] = False
                            elif (int(v2) - int(v1)) > 0:
                                self.direction[GPS_msg.people[i].person.name] = True
                            self.closest_nodes[GPS_msg.people[i].person.name] = current_node
                    except:
                        pass
                    for q in range(0,No_of_ptcl):
                        P_q = state['Particles'][i][q]         #particle q              
                        D_q = state['life'][i][q]              #its duration in current node (In form of probability)
                        indx = self.Node_names.index(P_q)           #Index of P_q, to choose correct row of Pr_p
                        if self.direction[GPS_msg.people[i].person.name] == True:
                            Prp = self.Pr_f.copy()
                            P_p_q = Prp[indx]
                        elif self.direction[GPS_msg.people[i].person.name] == False:
                            Prp = self.Pr_b.copy()
                            P_p_q = Prp[indx]
                        else:
                            Prp = self.Pr_p.copy()
                            P_p_q = Prp[indx]
                            
                        for n in range(0,len(P_p_q)): 
                            if (P_p_q[n] == 1 and indx != n):       #probability of leaving current node
                                P_p_q[n] =  D_q#lambdaa-D_q 
                            elif (P_p_q[n] == 1 and indx == n):     #probability of staying in current node
                                P_p_q[n] = 1 - D_q
                            elif P_p_q[n] == 0.1:                   #probaility of all other jumps to nodes in vacinity
#                                print type(self.coords[indx][0])
#                                print type(self.estimated)
                                if self.coords[indx][0] == self.estimated[0]:
                                    P_p_q[n] =  0.0004*D_q#lambdaa#(lambdaa - D_q)/2000#0.01
                                else:
                                    P_p_q[n] = 0
                        P_p_q = normalize(P_p_q) 
                        #print sum(P_p_q)
                        P = list(np.random.choice(self.Node_names, 1, p = P_p_q))
                        Pp = Pp + P      
#                    state['Particles'][i]=Pp

                    ###############           
                    counter=collections.Counter(Pp)       #handler to count number of particles in each node
                    nodes_occuppied = np.asarray(counter.most_common(len(counter.keys())))   # how many particles in each node
                    nodes = nodes_occuppied[:,0]                                            # Nodes with atleast one particle
                    times = nodes_occuppied[:,1].astype(np.float)                            # Number of particles in Xth node 
                    D=[]
                    for k in range (0, No_of_ptcl):       
                        d = np.sqrt((self.coords[self.Node_names.index(Pp[k])][0]  - X)**2  +  (self.coords[self.Node_names.index(Pp[k])][1]  - Y)**2)# distance between GPS reading and nodes1 
                        if d<0.1:
                            d = d +  float(np.random.normal(0,0.2,1))
                        D.append(-1 * meas_power * d)
                    
                    D = np.exp(D)                         # Covert distance to weightss
                    W = normalize(D)
                    
                    ##################
#                    W = D*parameters['W_p'][i]
#                    W=normalize(W)
#                    parameters['W_p'][i] = W
                   
                    ##################
                        
                    Neff = 1. / np.sum(np.square(W))
#                    print Neff
                    if Neff < 10:#No_of_ptcl/10:
                        W = [1/No_of_ptcl]*No_of_ptcl
                        parameters['W_p'][i] = W
                        print 'Neef'
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
                ########################################################

                    ########### Particle Marker#######
                        ###########################################
                    ids = 100
#                    for m in Pp:   ##Predicted particles
                    for m in state['Particles'][i]:## Updated particles
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
#                    if self.timestamps[GPS_msg.people[i].person.name] in time:
#                        timestamp = GPS_msg.people[0].header.stamp.secs
#                        idx = time.index(timestamp)
#                        gx = x_loc[idx]
#                        gy = y_loc[idx]
                    self.markerarray_est[i] = MarkerArray()
                    est_pos = Marker()
                    est_pos.header.frame_id = "/map"
                    est_pos.type = est_pos.SPHERE
                    est_pos.pose.position.x = self.coords[self.Node_names.index(nodes[est])][0]
                    est_pos.pose.position.y = self.coords[self.Node_names.index(nodes[est])][1]  
                    self.estimated = [self.coords[self.Node_names.index(nodes[est])][0], self.coords[self.Node_names.index(nodes[est])][1]]
                    est_pos.pose.position.z = 6
                    est_pos.scale.x = 0.5
                    est_pos.scale.y = 0.5
                    est_pos.scale.z = 0.5
                    est_pos.color.a = 1
                    est_pos.color.r = 0
                    est_pos.color.b = 0
                    est_pos.color.g = 1
                    est_pos.id = marker_id
                    marker_id += 1
#                        e = (est_pos.pose.position.x - gx)**2 + (est_pos.pose.position.y  - gy)**2
#                        self.error = self.error + e
#                        den += 1
#                        time.pop(0)
#                        x_loc.pop(0)
#                        y_loc.pop(0)
#                    
#                        print self.error, timestamp, den
                    self.markerarray_est[i].markers.append(est_pos)
                    self.pub[i].publish(self.markerarray[i])
                    self.pub2[i].publish(self.markerarray_est[i])
                else:
                    pass


if __name__ == '__main__':
    rospy.init_node('PF', anonymous = True)
    sml = PFclass()
    rospy.spin()
    
    
    
    
    
    