#!/usr/bin/env python
# coding: utf-8

# In[1]:


import os
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

# MoveIt 설정
moveit_config = (
    MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
    .robot_description(file_path="config/panda.urdf.xacro")
    .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    .moveit_cpp(file_path=os.path.join(get_package_share_directory("moveit2_tutorials"), "config", "jupyter_notebook_prototyping.yaml"))
    .to_moveit_configs()
).to_dict()

# ROS 2 초기화 및 MoveItPy 인스턴스 생성
rclpy.init()
panda = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
panda_arm = panda.get_planning_component("panda_arm")
panda_gripper = panda.get_planning_component("hand")

# 경로 계획 및 실행 함수
def plan_and_execute(robot, planning_component, single_plan_parameters=None, multi_plan_parameters=None):
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    if plan_result:
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        print("Planning failed")

# 특정 좌표로 이동시키는 함수
def move_to_pose(robot, planning_component, pose):
    planning_component.set_start_state_to_current_state()
    planning_component.set_goal_state(pose_stamped_msg=pose, pose_link="panda_link8")
    plan_and_execute(robot, planning_component)

# 그리퍼 제어 함수
def open_gripper():
    panda_gripper.set_start_state_to_current_state()
    panda_gripper.set_goal_state("open")
    plan_and_execute(panda, panda_gripper)
    print("Gripper opened")

def close_gripper():
    panda_gripper.set_start_state_to_current_state()
    panda_gripper.set_goal_state("close")
    plan_and_execute(panda, panda_gripper)
    print("Gripper closed")


# In[2]:


# 초기 위치 정의 (특정 좌표로 설정)
initial_pose = PoseStamped()
initial_pose.header.frame_id = "panda_link0"
initial_pose.pose.orientation.w = 0.0
initial_pose.pose.orientation.x = 1.0
initial_pose.pose.orientation.y = 0.0
initial_pose.pose.orientation.z = 0.0
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.position.z = 0.7

# 화분 위치 정의 (3개)
pot_positions = [
    (-0.4, 0.4, 0.5),
    (0.0, 0.4, 0.5),
    (0.4, 0.4, 0.5)
]

# 물뿌리개 위치 정의 (로봇 오른쪽)
watering_can_pose = PoseStamped()
watering_can_pose.header.frame_id = "panda_link0"
watering_can_pose.pose.orientation.w = 0.0
watering_can_pose.pose.orientation.x = 1.0
watering_can_pose.pose.orientation.y = 0.0
watering_can_pose.pose.orientation.z = 0.0
watering_can_pose.pose.position.x = 0.4
watering_can_pose.pose.position.y = -0.4
watering_can_pose.pose.position.z = 0.5

# 보관함 위치 정의 (로봇 왼쪽)
harvest_pose = PoseStamped()
harvest_pose.header.frame_id = "panda_link0"
harvest_pose.pose.orientation.w = 0.0
harvest_pose.pose.orientation.x = 1.0
harvest_pose.pose.orientation.y = 0.0
harvest_pose.pose.orientation.z = 0.0
harvest_pose.pose.position.x = -0.4
harvest_pose.pose.position.y = -0.4
harvest_pose.pose.position.z = 0.5

# 목표 좌표 정의 (화분)
pot_pose = PoseStamped()
pot_pose.header.frame_id = "panda_link0"
pot_pose.pose.orientation.w = 0.0
pot_pose.pose.orientation.x = 1.0
pot_pose.pose.orientation.y = 0.0
pot_pose.pose.orientation.z = 0.0


# In[3]:


# 메인 실행 루프
# 1. 초기 위치로 이동
move_to_pose(panda, panda_arm, initial_pose)
open_gripper()

# 2. 물뿌리개 위치로 이동 및 집기
move_to_pose(panda, panda_arm, watering_can_pose)
close_gripper()

# 3. 각 화분에 물주기
for i in range(3):
    x, y, z = pot_positions[i]
    pot_pose.pose.position.x = x
    pot_pose.pose.position.y = y
    pot_pose.pose.position.z = z
    move_to_pose(panda, panda_arm, pot_pose)
    time.sleep(2)  # 2초간 기울여서 물을 뿌리기

    # 물뿌리기 후 살짝 위로 이동
    intermediate_pose = PoseStamped()
    intermediate_pose.header.frame_id = "panda_link0"
    intermediate_pose.pose.orientation.w = 0.0
    intermediate_pose.pose.orientation.x = 1.0
    intermediate_pose.pose.orientation.y = 0.0
    intermediate_pose.pose.orientation.z = 0.0
    intermediate_pose.pose.position.x = x
    intermediate_pose.pose.position.y = y
    intermediate_pose.pose.position.z = z + 0.1  
    move_to_pose(panda, panda_arm, intermediate_pose)

# 4. 물뿌리개를 제자리에 둠
move_to_pose(panda, panda_arm, watering_can_pose)
open_gripper()

# 5. 각 화분에서 열매를 자르기
for i in range(3):
    x, y, z = pot_positions[i]
    pot_pose.pose.position.x = x
    pot_pose.pose.position.y = y
    pot_pose.pose.position.z = z
    move_to_pose(panda, panda_arm, pot_pose)
    close_gripper() 

    # 6. 자른 열매를 보관함에 넣기
    move_to_pose(panda, panda_arm, harvest_pose)
    open_gripper()

# 초기 위치로 복귀
move_to_pose(panda, panda_arm, initial_pose)
close_gripper()


# In[ ]:




