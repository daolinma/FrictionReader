#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs
import gclib
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray


ScaleLinearStage = 1.0/10000;   # 10000 counts/mm
ScaleRotaryStage = 1.0/800;        #800 counts/deg

# Define PositionType  # TODO: int32?

# Int32MultiArray position

def position_publisher():
    ################### Establish the connection to controller through Ethernet ############
    g = gclib.py()
    g.GOpen('169.254.93.220 --direct')
    print(g.GInfo())
    c = g.GCommand      # alias the command callable

##########  publisher ###########
    pub = rospy.Publisher('stage_pos', Float32MultiArray, queue_size=10)
    rospy.init_node('stage_position_node', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    # data_msg = Int32MultiArray()
    data_msg = Float32MultiArray()
    # data_msg.layout.dim[0].size = 1
    # data_msg.layout.dim[0].stride = 3
    # data_msg.layout.dim[0].label = 'x,y,z'
    data_msg.data = []
    while not rospy.is_shutdown():
        pos_x = c('TPA')
        pos_y = c('TPB')
        pos_z = c('TPC')
        data_msg.data  =[int(pos_x)*ScaleLinearStage,int(pos_y)*ScaleLinearStage,int(pos_z)*ScaleRotaryStage]
        pub.publish(data_msg)

if __name__ == '__main__':
    try:
        position_publisher()
    except rospy.ROSInterruptException:
        pass
