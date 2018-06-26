close all
clear all

%% Preparation
directoryname = ['/Users/madaolin/Dropbox (MIT)/Daolin Lab Repo/Projects/2018 Engineering Friction with Micro-textures/Data Collected/']

shape_id = '1';
filelist = dir([directoryname]);

mn = size(filelist)
offset_dir = 3;

 for i = 1:mn(1)-offset_dir
    filename = [pwd,directoryname,filelist(i+offset_dir).name]
    do_plot = 0; 
    [pos_record, wrench_record] = get_data(filename, shape_id, do_plot);

 end 