#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray,Bool
from std_srvs.srv import Empty
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import json
# import settings
pos_record = []
wrench_record = []

# settings.init()

def callback_pos(data):
    global pos_record
    rospy.loginfo("I heard %s",data.data)
    pos_record.append(data.data)
    # print pos_record

def callback_wrench(data):
    global wrench_record
    # global wrench_record
    rospy.loginfo("I heard %s",data)
    wrench_record.append(data)

def exp_listener():
    stop_sign = False
    # while stop_sign = False
    rospy.init_node('lisener_node')
    rospy.Subscriber("stage_pos", Float32MultiArray, callback_pos)
    rospy.Subscriber("netft_data", WrenchStamped, callback_wrench)
    # print settings.pos_record
    # stop_sign = rospy.wait_for_message("reading_status", Bool, 3);
    rospy.spin()

def start_read():  ## TODO: get this into a rosservice
    global pos_record
    global wrench_record
    pos_record = []
    wrench_record = []

def save_readings(): ## TODO: get this into a rosservice
    global pos_record
    global wrench_record
    # print pos_record
    filename = rospy.get_param('save_file_name')
    with open(filename, 'w') as outfile:  # write data to 'data.json'
        json.dump({'pos_list':pos_record, 'wrench_list': wrench_record}, outfile)
    rospy.sleep(3)


if __name__ == '__main__':
    try:
        s_1 = rospy.Service('start_read', Empty, start_read)
        s_1 = rospy.Service('save_readings', Empty, save_readings)
        exp_listener()
        print ('mylistener ready!')
    except rospy.ROSInterruptException:
        pass
