#!/usr/bin/env python
from __future__ import division          # stop rounding off during division
import numpy as np
coords =[]



Nodes = [['A',0,0], ['B',0,10], ['C',0,20], ['D',10,0], ['E',10,10], ['F',10,20], ['G',0,30], ['H',0,40], ['I',0,50], ['J',10,30], ['K',10,40], ['L',10,50]]
#Nodes = [['A',0,0], ['B',0,10]]

nodes = [i[0] for i in Nodes]

def call_me():
#    Pr_p =  np.array([[0.3,0.3,0,0.3,0,0,0,0,0,0,0,0],[0.3,0.3,0.3,0,0,0,0,0,0,0,0,0],[0,0.3,0.3,0,0,0,0.3,0,0,0,0,0],[0.3,0,0,0.3,0.3,0,0,0,0,0,0,0],
#                     [0,0,0,0.3,0.3,0.3,0,0,0,0,0,0],[0,0,0,0,0.3,0.3,0,0,0,0.3,0,0],[0,0,0.3,0,0,0,0.3,0.3,0,0,0,0],[0,0,0,0,0,0,0.3,0.3,0.3,0,0,0],
#                     [0,0,0,0,0,0,0,0.3,0.3,0,0,0.3],[0,0,0,0,0,0.3,0,0,0,0.3,0.3,0],[0,0,0,0,0,0,0,0,0,0.3,0.3,0.3],[0,0,0,0,0,0,0,0,0.3,0,0.3,0.3]])#Hardcoded probability
    Pr_p =  np.array([[1,1,0,1,0,0,0,0,0,0,0,0],[1,1,1,0,0,0,0,0,0,0,0,0],[0,1,1,0,0,0,1,0,0,0,0,0],[1,0,0,1,1,0,0,0,0,0,0,0],
                      [0,0,0,1,1,1,0,0,0,0,0,0],[0,0,0,0,1,1,0,0,0,1,0,0],[0,0,1,0,0,0,1,1,0,0,0,0],[0,0,0,0,0,0,1,1,1,0,0,0],
                      [0,0,0,0,0,0,0,1,1,0,0,1],[0,0,0,0,0,1,0,0,0,1,1,0],[0,0,0,0,0,0,0,0,0,1,1,1],[0,0,0,0,0,0,0,0,1,0,1,1]],dtype='float')
                      
    Pr_p[Pr_p == 0] = 0.1
                      
#    Pr_p = np.array([[1,1],[1,1]],dtype='f')
#    print len(Pr_p) 
                      
#    np.fill_diagonal(Pr_p,4)
#    Pr_p = Pr_p + .05
#    row_sums = Pr_p.sum(axis=1)
#    Pr_p = Pr_p / row_sums[:, np.newaxis] #normalized predicted probabilities
    for i in range(0,len(Nodes)):
        coords.append([Nodes[i][1], Nodes[i][2]])
    return (Pr_p, nodes, coords)

            
            

def normalize(arr):
    try:     #for nd array
        row_sums = arr.sum(axis=1)
        arr = arr / row_sums[:, np.newaxis] 
    except:
        row_sums = arr.sum(axis=0)    #for 1d array
        arr = arr / row_sums     
    return arr