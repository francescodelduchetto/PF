#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from bayes_people_tracker.msg import PeopleStamped
from numpy import *
import numpy as np
from scipy.linalg import block_diag
import math 
from random import gauss
Topics = ['object_3d', 'car_client/get_gps','hedge_pos_a','gps_positions']
kickout_time = 4 
No_of_ptcl = 6   # No of particles
state = {'Names': [], 'GPSx':[], 'GPSy':[], 'velocityx':[], 'velocityy':[], 'dt':[], 'ini_x':[], 'ini_y':[], 'ini_time':[], 'Particles':[], 'w_i':[], 'w_ini':[]}  #Empty dictionary

class PFclass():
    def __init__(self):
        self.tracks = None
        rospy.Timer(rospy.Duration(kickout_time), self.gps_active, oneshot=False) 
        
        
    def gps_active(self,mydata):                    #check for active gps 
        
        num_of_users = len(state['Names'])             #Number of active gps users
        current_time=rospy.get_rostime()            
        for i in range (num_of_users-1,-1,-1):       #range(start,stop,step_size) &  # It is necessary to run this loop in reverse direction to avoid errors as popping the first GPS from array will decrement the indices of all other GPS's.
#           print (current_time.secs-state['ini_time'][i])
           if len(state['Names'])>0 and 1==2:# and (current_time.secs-state['ini_time'][i])>kickout_time:
                state['Names'].pop(i)
                state['GPSx'].pop(i)
                state['GPSy'].pop(i)
                state['velocityx'].pop(i)
                state['velocityy'].pop(i)
                state['dt'].pop(i)
                state['ini_x'].pop(i)
                state['ini_y'].pop(i)
                state['Particles'].pop(i)
        
    def tracker(self,GPS_msg):
#        print state['Names']
#        print state['w_ini']
        No_of_users = len(GPS_msg.people) #Number of logged users
        for i in range (0,No_of_users):
            if GPS_msg.people[i].person.name not in state['Names']:
                state['ini_time'].append(GPS_msg.people[i].header.stamp.secs)
                state['dt'].append('None')
                state['ini_x'].append(GPS_msg.people[i].person.position.x)
                state['ini_y'].append(GPS_msg.people[i].person.position.y)
                vx = 1
                vy = 1
                state['Names'].append(GPS_msg.people[i].person.name)
                state['GPSx'].append(GPS_msg.people[i].person.position.x)
                state['GPSy'].append(GPS_msg.people[i].person.position.y)
                state['velocityx'].append(vx)
                state['velocityy'].append(vy)
                px = [gauss(GPS_msg.people[i].person.position.x, math.sqrt(1)) for n in range(No_of_ptcl)]
                py = [gauss(GPS_msg.people[i].person.position.y, math.sqrt(5000000)) for m in range(No_of_ptcl)]
                state['Particles'].extend([px, py, [1]*No_of_ptcl, [1]*No_of_ptcl])
                state['w_ini'].extend(np.array(ones(No_of_ptcl))/No_of_ptcl)
                state['w_i'].extend(np.array(ones(No_of_ptcl))/No_of_ptcl)
#                state['w_i'].append(np.random.uniform(low=0, high=1, size=(No_of_ptcl,)))
            elif GPS_msg.people[i].person.name in state['Names']:
                state['dt'][i] = GPS_msg.people[i].header.stamp.secs - state['ini_time'][i] 
                if state['dt'][i] == 0:
                    state['dt'][i] = 1                             #This if is only here to avoid dt = 0 i.e., when gps rate is higher than 1 reading/second 
                state['ini_time'][i] = GPS_msg.people[i].header.stamp.secs
                state['GPSx'][i] = GPS_msg.people[i].person.position.x
                state['GPSy'][i] = GPS_msg.people[i].person.position.y
                state['velocityx'][i] = (state['GPSx'][i] - state['ini_x'][i])/state['dt'][i]
                state['velocityy'][i] = state['GPSy'][i] - state['ini_y'][i]/state['dt'][i]
                state['ini_x'][i] = state['GPSx'][i]
                state['ini_y'][i] = state['GPSy'][i] 
                M=np.matrix('1 0 3 0;0 1 0 3; 0 0 1 0; 0 0 0 1')    ##Constraint will be Here
                for j in range(0,No_of_ptcl):
                    measurement = np.array([state['GPSx'],state['GPSy']])
                    pi = [row[j] for row in state['Particles']]  
                    print state['Particles']
                    wi = state['w_i']
                    
                    #i^th particle
                    pi = np.array(pi)
                    pi = pi.reshape(-1,1)
                    print pi

if __name__=='__main__':
    rospy.init_node('PF',anonymous = True)
    sml = PFclass()
    rospy.Subscriber(Topics[3], PeopleStamped, sml.tracker)
    rospy.spin()