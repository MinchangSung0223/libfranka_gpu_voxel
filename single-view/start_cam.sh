#!/bin/bash
IFS=$'\n'  
ARR=(`rs-enumerate-devices | grep "Serial Number" | sed 's/[^0-9]//g'`)
roslaunch realsense2_camera demo_pointcloud.launch filters:=pointcloud &
python base_zf.py cam_0 

