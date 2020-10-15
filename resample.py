import numpy as np
import collections

#def resample(weights,particles):
#    weights = weights
#    particles = particles
#    N =  len(particles)
#    cumulative_sum = np.cumsum(weights)
#    cumulative_sum[-1] = 1. # avoid round-off error
#    indexes = np.searchsorted(cumulative_sum,np.random.rand(1,N))
##    print np.random.rand(1,N)
#    # resample according to indexes
#    for ii in range(0,N):
#        particles[ii] = particles[indexes[0][ii]]
##        weights[ii] = weights[indexes[ii]]
##    weights /= np.sum(weights) # normalize
#    return particles

dic = {'A1':3,'A101':3,'A10':3,'A2':3,'A8':9,'A6':3,'A4':3,}
Nodes = collections.OrderedDict(sorted(dic.items(),key=lambda t: t[1]))
print Nodes
