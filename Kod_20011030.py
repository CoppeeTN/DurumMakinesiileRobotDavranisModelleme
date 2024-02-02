#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
import time
import smach
import smach_ros
import os
import signal
from geometry_msgs.msg import Twist,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int32
import actionlib
from math import radians , pi

def open_core():
    try:
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'roscore;  exec bash'])
    except Exception as e:
        print("Hata: {e}")
        
def open_launch(package_name, launch_name):
    try:
        launch_file = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'roslaunch {package_name} {launch_name}; exec bash'])
        return launch_file
    except Exception as e:
        print("Hata: {e}")

def open_node(package_name, node_name):
    try:
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'rosrun {package_name} {node_name}; exec bash'])
    except Exception as e:
        print("Hata: {e}")
        
def get_user_input():
    map_scale = input("Map Scale [kucuk,orta,buyuk]: ")
    x = input("Başlangıç X değeri : ")
    y = input("Başlangıç Y değeri : ")
    return map_scale,x,y

def update_launch_file(map_scale,x,y):
    launch_template = f"""
<launch>
    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="x_pos" default="{x}"/>
    <arg name="y_pos" default="{y}"/>
    <arg name="z_pos" default="0.0"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find proje)/worlds/proje_map_{map_scale}.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
    </include>

    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
</launch>
    """

    with open("proje.launch", "w") as launch_file:
        launch_file.write(launch_template)

def rotate_turtlebot():
    rospy.sleep(2) 
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist_cmd = Twist()
    twist_cmd.angular.z = 0.6
    duration = 2 * pi / twist_cmd.angular.z
    start_time = rospy.get_time()
   
    end_time = start_time + duration
    while rospy.get_time() < end_time:
        cmd_vel_pub.publish(twist_cmd)
        
    twist_cmd.angular.z = 0.0
    cmd_vel_pub.publish(twist_cmd)
    cmd_vel_pub.unregister()
    
def return_start(x, y):

    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    goal.target_pose.pose.orientation.w = 1.0  

    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
        
def state_machine(x,y):
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['Bitiş'])
    
    with sm:
        smach.StateMachine.add('Rotate360', Rotate360(), transitions={'Dönüş bitti':'ExploreMap'})
        smach.StateMachine.add('ExploreMap', ExploreMap(), transitions={'Keşif Başarıyla Bitti':'ReturnStart'})
        return_start_state = ReturnStart(x,y)
        smach.StateMachine.add('ReturnStart', return_start_state, transitions={'Başlangica Dönüldü':'Bitiş'})
   
    sis = smach_ros.IntrospectionServer('ProjeServeri', sm, '/StateMachine')
    sis.start()
    
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop() 
    
  

class Rotate360(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Dönüş bitti'])
    def execute(self,userdata):
        rotate_turtlebot()
        return 'Dönüş bitti'  

class ExploreMap(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['Keşif Başarıyla Bitti'])
        rospy.Subscriber('/explore/frontiers', MarkerArray, self.goal_callback)
        self.goal_received = True

    def goal_callback(self, msg):
        self.goal_received = True

    def start_explore_lite(self):
        self.explore_process = subprocess.Popen(['roslaunch', 'explore_lite', 'explore.launch'])
        
    def stop_explore_lite(self):
        if self.explore_process:
            self.explore_process.terminate()
            self.explore_process.wait()
  

    def execute(self, userdata):
        self.start_explore_lite()
        start_time = rospy.get_time()
        while self.goal_received:
            self.goal_received = False
            time.sleep(12)
        duration = rospy.get_time() - start_time
        print(duration)
        self.stop_explore_lite()
        return 'Keşif Başarıyla Bitti'
        
        
class ReturnStart(smach.State):
    def __init__(self,x,y):
        smach.State.__init__(self,outcomes=['Başlangica Dönüldü'])
        self.x = x
        self.y = y
    def execute(self,userdata):
        start_time = rospy.get_time()
        return_start(self.x,self.y)
        duration = rospy.get_time() - start_time
        print(duration)
        return 'Başlangica Dönüldü'

                        
if __name__ == "__main__": 
    
    map_scale, x, y = get_user_input()
    update_launch_file(map_scale,x,y)
    
    open_launch("proje", "proje.launch")
    time.sleep(5)
    open_launch("proje", "proje_slam.launch")
    time.sleep(5)
    open_node("smach_viewer", "smach_viewer.py")
    time.sleep(5)
    state_machine(x,y)
    

    
    
    
    
