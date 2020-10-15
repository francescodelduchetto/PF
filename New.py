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
state = {'Names':[], 'x_i':[], 'y_i':[], 'vx_i':[], 'vy_i':[], 'timestamp':[], 'tp_i':[], 'Particles':[], 'particles':[], 'weights':[], 'new_obs':[]}
No_of_ptcl = 2

class PFclass():
    def __init__(self):
        self.tracks = None
        rospy.Subscriber(Topics[3], PeopleStamped, self.setup)
        rospy.Timer(rospy.Duration(0.2), self.DR, oneshot=False)

    def DR(self, msgdata):
        for no in range(0,len(state['Names'])):
            pii = state['particles'][no]                                          #set of particles of no^th user
            Motion = np.matrix([[1,0,0.2,0], [0,1,0,0.2], [0,0,1,0], [0,0,0,1]])      #Constraints go here
            for i in range(0,No_of_ptcl):
                pi = [row[i] for row in pii]                                      #ith particle of no^th user
                pi = np.asarray(pi)
                pi = pi[:,None]                           
                pip = Motion*pi
#                print state['particles'][no]
#                print state['particles'][no][0:2][1]
#                state['particles'][no][0][i] = pip[0,0]
#                state['particles'][no][1][i] = pip[1,0]   
                if state['new_obs'][no] == 'Initial':
                    pass
                elif state['new_obs'][no] == True:
                    obs = np.asarray([state['x_i'][no], state['y_i'][no]])
                    dx = obs[:,None] - pip[0:2]
                    w = np.exp(-0.5*(np.matmul(dx.reshape(1,-1),dx))/2)
                    w = w * state['weights'][no][i]
                    state['weights'][no][i] = w        
            if True:
                state['weights'][no] = state['weights'][no]/sum(state['weights'][no])
                state['particles'][no][0][0] = pip[0,0]
                state['particles'][no][1][i] = pip[1,0] 
            else:
                pass


    def setup(self, GPS_data):
        No_of_users = len(GPS_data.people) #Number of logged users
        for i in range (0,No_of_users):
            if GPS_data.people[i].person.name not in state['Names']:
                state['Names'].append(GPS_data.people[i].person.name)             #Add new user to state
                state['x_i'].append(GPS_data.people[i].person.position.x)         #Add new user's location to state (GPS location)
                state['y_i'].append(GPS_data.people[i].person.position.y)
                state['vx_i'].append(1)                                           #Add new user initial velocity to state (1 m/s)
                state['vy_i'].append(1)
                state['tp_i'].append(GPS_data.people[i].header.stamp)
                px = [gauss(GPS_data.people[i].person.position.x, math.sqrt(1)) for n in range(No_of_ptcl)]
                py = [gauss(GPS_data.people[i].person.position.y, math.sqrt(1)) for m in range(No_of_ptcl)]
                state['particles'].append([px, py, [2]*No_of_ptcl, [1]*No_of_ptcl])                           #Add new users particles
                state['new_obs'].append('Initial')
                state['weights'].append(np.array(np.ones(No_of_ptcl)))      
            else:
                state['x_i'][i] = GPS_data.people[i].person.position.x
                state['y_i'][i] = GPS_data.people[i].person.position.y
                state['vx_i'][i] = 1     #to be changed later
                state['vy_i'][i] = 1     #to be changed later
                state['new_obs'][i] = True


if __name__=='__main__':
    rospy.init_node('PF',anonymous = True)
    sml = PFclass()
    rospy.spin()