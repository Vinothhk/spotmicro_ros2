#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*  Filename:			start_world_launch.py
*  Description:         Use this file to launch world in gazebo simulator 
*  Last Modified:	    12/10/2023
*  Author:				Vinoth Kumar K
*****************************************************************************************
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_spot_micro_gazebo = get_package_share_directory('spot_micro_gazebo')
    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazeb
    description_package_name = "spot_micro_description"
    install_dir = get_package_prefix(description_package_name)
    
    # Set the path to the WORLD model files. Is to find the models inside the models fo
    gazebo_models_path = os.path.join(pkg_spot_micro_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =install_dir + "/share" + ':' + gazebo_models_path
    
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    
    
    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_spot_micro_gazebo, 'worlds', 'empty.world')],
            description='SDF world file'),
          gazebo
    ])