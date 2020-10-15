#! /usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray

#Topology = {row0:[], row1:[], row2:[], row3:[], row5:[]}
Topics = ['object_3d','car_client/get_gps','hedge_pos_a','gps_positions']

class PFclass():
    def __init__(self):
        self.tracks = None
        self.list1 = []
        self.list2 = []
        self.pub = rospy.Publisher('TRACKS', MarkerArray, queue_size = 10)
    def tracker(self,GPS_msg):
        print len(GPS_msg)        
        



if __name__=='__main__':
    rospy.init_node('PF',anonymous=True)
    sml = PFclass()
    rospy.Subscriber(Topics[3], String, PFclass.tracker)
    #rospy.Subscriber()
    

    