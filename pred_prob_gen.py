#!usr/bin/env python
from __future__ import division          # stop rounding off during division
import rospy
import numpy as np
from strands_navigation_msgs.msg import TopologicalMap


Waypoints = []
#p = 5
#p1 = p                   #probability that particles will stay at current node
#p2 = p/10                #probaility that pariticles will jump to neighboring nodes (p2 is 'denominator' times less that p)
#p3 = p/100               #probaility that pariticles will jump to other (not neighboring) nodes (p3 is 'denominator' times less that p)
p=1
p1=p
p2=p
p3=0

def prob(msg):
    P = np.zeros((len(msg.nodes),len(msg.nodes)))
    for i in range(0, len(msg.nodes)):
        Waypoints.append([msg.nodes[i].name,msg.nodes[i].pose.position.x, msg.nodes[i].pose.position.y])
    Node_names = [i[0] for i in Waypoints]
    for j in range(0, len(msg.nodes)):
       # P[j,j] = 1
        for jj in range(0, len(msg.nodes)):
            d = np.sqrt((msg.nodes[j].pose.position.x - msg.nodes[jj].pose.position.x)**2 + (msg.nodes[j].pose.position.y - msg.nodes[jj].pose.position.y)**2)
            if d<2:
                P[j,jj] = 0.1                            ## jump to neighbnoring row
                P[jj,j] = 0.1                            ## jump to neighbnoring row
        P[j,j] = 1                                       ## stay at current
        
        
        
    for j in range(0, len(msg.nodes)):
        for k in range(0,len(msg.nodes[j].edges)):
            neighbor = msg.nodes[j].edges[k].node
            #######
            index2 = msg.nodes[j].name.find('c')
            v2 = msg.nodes[j].name[index2+1:]
            index1 = neighbor.find('c')
            v1 = neighbor[index1+1:]
            try:
                if int(v1) > int(v2):
                #######
                    idc = Node_names.index(neighbor)
                    P[j,idc] = 1
                   # P[idc,j] = 1
            except:
                pass
    P[P==0] = p3     
    PForward = P
    PBackward = np.transpose(P)
    P = (P + np.transpose(P))/2
    
#    PForward = np.triu(P, k=0)  ##extra
#    PBackward = np.tril(P, k=0) ###extra
    return (P, PForward, PBackward, Waypoints)     ###extra

#    return(PForward,Waypoints)
    

def normalize(arr):
    try:     #for nd array
        row_sums = arr.sum(axis=1)
        if row_sums == 0:
            return arr
        arr = arr / row_sums[:, np.newaxis] 
    except:
        row_sums = arr.sum(axis=0)    #for 1d array
        if row_sums == 0:
            return arr
        arr = arr / row_sums    
    return arr
    
if __name__=='__main__':
    rospy.init_node('prior_dist',anonymous = True)
    rospy.Subscriber('topological_map',TopologicalMap,prob)
    rospy.spin()
    
