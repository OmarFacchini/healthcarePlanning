# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('problem5')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    deliver_item_cmd = Node(
        package='problem5',
        executable='deliver_item_action_node',
        name='deliver_item_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deploy_box_cmd = Node(
        package='problem5',
        executable='deploy_box_action_node',
        name='deploy_box_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deploy_container_cmd = Node(
        package='problem5',
        executable='deploy_container_action_node',
        name='deploy_container_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    drop_off_patient_cmd = Node(
        package='problem5',
        executable='drop_off_patient_action_node',
        name='drop_off_patient_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    get_patient_cmd = Node(
        package='problem5',
        executable='get_patient_action_node',
        name='get_patient_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    insert_box_cmd = Node(
        package='problem5',
        executable='insert_box_action_node',
        name='insert_box_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    insert_item_cmd = Node(
        package='problem5',
        executable='insert_item_action_node',
        name='insert_item_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    move_busy_carrier_cmd = Node(
        package='problem5',
        executable='move_busy_carrier_action_node',
        name='move_busy_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    move_escorting_accompanier_cmd = Node(
        package='problem5',
        executable='move_escorting_accompanier_action_node',
        name='move_escorting_accompanier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    move_free_accompanier_cmd = Node(
        package='problem5',
        executable='move_free_accompanier_action_node',
        name='move_free_accompanier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    pickup_container_cmd = Node(
        package='problem5',
        executable='pickup_container_action_node',
        name='pickup_container_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(deliver_item_cmd)
    ld.add_action(deploy_box_cmd)
    ld.add_action(deploy_container_cmd)
    ld.add_action(drop_off_patient_cmd)
    ld.add_action(get_patient_cmd)
    ld.add_action(insert_box_cmd)
    ld.add_action(insert_item_cmd)
    ld.add_action(move_busy_carrier_cmd)
    ld.add_action(move_escorting_accompanier_cmd)
    ld.add_action(move_free_accompanier_cmd)
    ld.add_action(pickup_container_cmd)

    return ld