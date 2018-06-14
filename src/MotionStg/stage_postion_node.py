#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

# Define PositionType  # TODO:

def position_publisher():

    ################### Establish the connection to controller through Ethernet ############
    g = gclib.py()
    g.GOpen('169.254.93.220 --direct')
    print(g.GInfo())
    c = g.GCommand      # alias the command callable

##########  publisher ###########
    pub = rospy.Publisher('chatter', PositionType, queue_size=10)
    rospy.init_node('stage_position_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pos_x = c('TPA') ;
        pos_y = c('TPB') ;
        pos_z = c('TPC') ;
        position = ['x'=pos_x,'y'=pos_y,'z'=pos_z]
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_publisher()
    except rospy.ROSInterruptException:
        pass
