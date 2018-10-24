#!/usr/bin/env python
# license removed for brevity

import rospy,numpy
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray,Bool
from std_srvs.srv import Empty,EmptyResponse
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import json
import os
import os.path

# import settings
pos_record = []
wrench_record = []

# settings.init()

# class MyEncoder(json.JSONEncoder):
#     def default(self, obj):
#         if isinstance(obj, numpy.integer):
#             return int(obj)
#         elif isinstance(obj, numpy.floating):
#             return float(obj)
#         elif isinstance(obj, numpy.ndarray):
#             return obj.tolist()
#         else:
#             return super(MyEncoder, self).default(obj)
def ftmsg2listandflip(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]


def callback_pos(data):
    global pos_record
    # rospy.loginfo("I heard %s",data.data)
    pos=[data.data[0],data.data[1],data.data[2]]
    pos_record.append(pos)
    # print pos_record

def callback_wrench(data):
    global wrench_record
    # rospy.loginfo("I heard %s",data)
    ft = ftmsg2listandflip(data)
    wrench_record.append([data.header.stamp.to_sec()] + ft)
    # wrench_record.append(data)


def exp_listener():
    stop_sign = False
    # while stop_sign = False
    rospy.Subscriber("stage_pos", Float32MultiArray, callback_pos)
    rospy.Subscriber("netft_data", WrenchStamped, callback_wrench)
    rospy.spin()

def start_read(req):
    global pos_record
    global wrench_record
    print("initializing data_list")
    pos_record = []
    wrench_record = []
    # print("initialized")
    return EmptyResponse()

def save_readings(req):
    global pos_record
    global wrench_record
    filename = rospy.get_param('save_file_name')
    output_data = {'pos_list':pos_record, 'wrench_list': wrench_record }
    # save_path = '/home/mcube-daolin/catkin_ws/src/motionstage/'
    date = filename.split("-")
    year = date[0] #find the year
    month = date[1] #find the month
    day = date[2] #find day
    # print("This is the year from the savepath:" + " " + year)
    # print("This is the month from the savepath:" + " " + month)
    # print("This is the day from the savepath:" + " " + day)
    save_path = '/home/mcube-daolin/Dropbox (MIT)/Daolin Lab Repo/Projects/2018 Engineering Friction with Micro-textures/Data Collected/' + year + "-" + month + "-" + day + '/'
    filename = save_path + filename
    # file_num = rospy.get_param('save_file_number')
    # if old_filename == filename:
    #     file_num += 1
    # else: 
    #     file_num = 0
    # print("saving file")
    # print(output_data)
    # print(type(output_data))
    # print("This save_path you want to create: ")
    # print(repr(save_path))
    #try instead save_path ~= old save path
    if not os.path.isdir(save_path):#if the directory doesn't exist make one and save the file inside
        print("Directory doesn't exist")
        os.mkdir(save_path)
        # print("Savepath you created: ")
        # print(repr(save_path))
        print("New Directory Created")
        #maybe i can just add the next 2 lines of code after this function  is called
        # file_num += 1
        # rospy.set_param('save_file_number',file_num)
        with open(filename, 'w') as outfile:  # write data to json file
            # print(type(outfile))
            print(filename)
            json.dump(output_data, outfile)   #TODO: find out why failing to save the file.
            outfile.close()
    else: #if the path already exists write the file in the existing folder
        print("Directory Already Exists")
        print('old path: ' + filename)
        #while the filename is not unique add an "a" in from of the filename
        while os.path.isfile(filename):
            print("File Already Exists")
            #change the file name
            expfile_previous = rospy.get_param('save_file_name')
            path_split = filename.split(day + '/')
            # print('Split path:')
            # print(path_split)
            filename = save_path +  'a' + path_split[1]
            # print('new path: ' + filename)
            # exp = 'a'+ 'save_file_name'
        with open(filename, 'w') as outfile:
                json.dump(output_data, outfile)
                outfile.close()
    #    print(number_files)
    #    number_files += 1
    #    filename  = filename + str(number_files)
    #    print("Number of files: " + str(number_files))
    # #    print("Filename: " + filename)
    #     if not os.path.isfile(filename): #if the file doesn't exist already, write
    #         with open(filename, 'w') as outfile:  # write data to json file
    #         # print(type(outfile))
    # #        print(filename)
    #             json.dump(output_data, outfile)   #TODO: find out why failing to save the file.
    #             outfile.close()
    #     else: #if the file exists already give it a new name
    #         with open(filename, 'w') as outfile:
    #             #(append)new_output_data = str(output_data) #Q: is this format acceptable?
    #             #(append) outfile.write(new_output_data) #outputing string of new data to old file
    #             json.dump(output_data, outfile)
    #             outfile.close()

    # print("file saved")
    rospy.sleep(.3)
    return EmptyResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('lisener_node', log_level = rospy.INFO)
        s_1 = rospy.Service('start_read', Empty, start_read)
        s_2 = rospy.Service('save_readings', Empty, save_readings)
        print ('mylistener ready!')
        exp_listener()
    except rospy.ROSInterruptException:
        pass
