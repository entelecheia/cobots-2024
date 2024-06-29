# 202121010 - 로봇 팔을 통한 과일 선별 과정 자동화

![project_img](./figs/project_img.jpeg)

## 프로젝트 개요

본 프로젝트는 ROS2 기반 협동 로봇을 활용하여 과일의 신선도 및 크기를 판별하고, 선별된 과일을 박스에 포장하는 자동화 시스템을 개발하는 것을 목표로 한다. 로봇은 정밀한 움직임을 통해 과일을 선별하고 포장하는 과정을 담당한다.

## 프로젝트 목표

- 로봇 손의 정밀한 움직임을 활용하여 감을 선별하고 포장하는 기능 개발
- 과일의 신선도와 크기를 판별하는 자동화 시스템 구현
- 과일 인식 기술을 개발하여 선별 기준 설정

## 방법론

- ROS2를 이용한 로봇 프로그래밍
- 컴퓨터 비전 기술을 이용한 감 인식 개발
- 정밀 움직임을 위한 센서 통합
- 사용자 관리 및 모니터링을 위한 애플리케이션 개발

## 필요 자원

- 하드웨어: 두산 협동 로봇, 고해상도 카메라, 다양한 센서
- 소프트웨어: ROS2, OpenCV 컴퓨터 비전 라이브러리

## 프로젝트 참여자

- 학번: 202121010
- 이름: 양필성
- 이메일: lal4995@gmail.com

## 예상 예산

| 품목        | 가격(원)   |
| ----------- | ---------- |
| 웹캠        | 64,800     |
| **총 예산** | **64,800** |

## 예상 일정 (총 기간: 8주)

| 주차   | 활동                                       |
| ------ | ------------------------------------------ |
| 주 1-2 | 프로젝트 계획 및 초기 설계                 |
| 주 3-4 | 로봇 프로그래밍 및 시뮬레이션              |
| 주 5-6 | 과일 선별 알고리즘 구현 및 파이프라인 구축 |
| 주 7   | 테스트 및 피드백                           |
| 주 8   | 최종 발표, 프로젝트 마무리 및 문서화       |

## 위험 관리

기술적 실패 및 일정 지연에 대비하여 소프트웨어 및 하드웨어에 대한 충분한 이해와 테스트를 통해 위험 최소화

## 프로젝트 결과

## 목차

1. [실행 방법](#실행-방법)
2. [주요 코드](#주요-코드)
3. [실행 결과](#실행-결과)
4. [한계점](#한계점)

```{notice}
이번 ROS 프로젝트는 Docker에서 진행을 하며python을 통해 구현을 했습니다.
```

## 실행 방법

1. Docker image

   - Docker image는 다음과 같습니다:
     `ghcr.io/ros-planning/moveit2_tutorials:$DOCKER_IMAGE`
   - Docker compose

     ```yml
     # Example command:
     # Humble on a Nvidia system:
     # DOCKER_IMAGE=humble-tutorial docker compose run gpu
     # Rolling without discrete graphics:
     # DOCKER_IMAGE=rolling-tutorial docker compose run cpu

     services:
     cpu:
         image: ghcr.io/ros-planning/moveit2_tutorials:$DOCKER_IMAGE
         container_name: moveit2_container
         privileged: true
         network_mode: host
         command: /bin/bash
         volumes:
         - /tmp/.X11-unix:/tmp/.X11-unix
         - $XAUTHORITY:/root/.Xauthority
         - $PWD:/root/ws_moveit/src/moveit2_tutorials
         environment:
         QT_X11_NO_MITSHM: 1
         DISPLAY: $DISPLAY
     gpu:
         image: ghcr.io/ros-planning/moveit2_tutorials:$DOCKER_IMAGE
         container_name: moveit2_container
         privileged: true
         network_mode: host
         command: /bin/bash
         deploy:
         resources:
             reservations:
             devices:
                 - driver: nvidia
                 count: 1
                 capabilities: [gpu]
         volumes:
         - /tmp/.X11-unix:/tmp/.X11-unix
         - $XAUTHORITY:/root/.Xauthority
         - $PWD:/root/ws_moveit/src/moveit2_tutorials
         environment:
         QT_X11_NO_MITSHM: 1
         DISPLAY: $DISPLAY
         NVIDIA_VISIBLE_DEVICES: all
         NVIDIA_DRIVER_CAPABILITIES: all
     ```

   - Docker 내부

     ```sh
     export DOCKER_IMAGE=humble-tutorial
     export XAUTHORITY=~/.Xauthority

     docker compose -f .docker/docker-compose.yml run gpu
     ```

2. Docker 내부에서 launch 파일 실행.

   - Jupyter를 통해 구현했기 때문에 jupyer lunch.py 실행

   ```sh
   ros2 launch moveit2_tutorials jupyter_notebook_prototyping.launch.py start_servo:=true
   ```

3. ros2_project.ipynb 파일 실행

## 주요 코드

### 필요한 라이브러리 임포트 및 moveit config 작성

```python
import os
import sys
import yaml
import rclpy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# we need to specify our moveit_py config at the top of each notebook we use.
# this is since we will start spinning a moveit_py node within this notebook.

moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("moveit2_tutorials"),
                "config",
                "jupyter_notebook_prototyping.yaml"
        )
    )
    .to_moveit_configs()
    ).to_dict()
```

### rcply 초기화 및 panda 불러오기

```python
# initialise rclpy (only for logging purposes)
rclpy.init()

# instantiate moveit_py instance and a planning component for the panda_arm
panda = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
panda_arm = panda.get_planning_component("panda_arm")
panda_gripper = panda.get_planning_component("hand")
```

### 기능 코드

```python
def plan_and_execute(
    robot,
    planning_component,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """A helper function to plan and execute a motion."""
    # plan to goal
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
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

# 그리퍼를 여는 함수 (가상의 함수로 대체)
def open_gripper():
    panda_gripper.set_start_state_to_current_state()
    panda_gripper.set_goal_state("open")
    plan_and_execute(panda, panda_gripper)
    print("Gripper opened")

# 그리퍼를 닫는 함수 (가상의 함수로 대체)
def close_gripper():
    panda_gripper.set_start_state_to_current_state()
    panda_gripper.set_goal_state("close")
    plan_and_execute(panda, panda_gripper)
    print("Gripper closed")


place_pose_xy = [(-0.2, -0.2), (-0.25, -0.2),(-0.2, -0.25),(-0.25,-0.25),(-0.2,-0.3),(-0.25,-0.3)]

# 초기 위치 정의 (특정 좌표로 설정)
initial_pose = PoseStamped()
initial_pose.header.frame_id = "panda_link0"
initial_pose.pose.orientation.w = 0.0
initial_pose.pose.orientation.x = 0.0
initial_pose.pose.orientation.y = 1.0
initial_pose.pose.orientation.z = 0.0
initial_pose.pose.position.x = 0.3
initial_pose.pose.position.y = 0.3
initial_pose.pose.position.z = 0.7

# 목표 좌표 정의
pick_pose = PoseStamped()
pick_pose.header.frame_id = "panda_link0"
pick_pose.pose.orientation.w = 0.0
pick_pose.pose.orientation.z = 0.0
pick_pose.pose.orientation.y = 1.0
pick_pose.pose.orientation.z = 0.0
#pick_pose.pose.position.x = -0.28
#pick_pose.pose.position.y = -0.2
pick_pose.pose.position.z = 0.1

place_pose = PoseStamped()
place_pose.header.frame_id = "panda_link0"
place_pose.pose.orientation.w = 0.0
place_pose.pose.orientation.x = 0.0
place_pose.pose.orientation.y = 1.0
place_pose.pose.orientation.z = 0.0
#place_pose.pose.position.x = 0.5
#place_pose.pose.position.y = 0.0
place_pose.pose.position.z = 0.3


for i in range(6):
    # 1. 초기 위치로 이동
    move_to_pose(panda, panda_arm, initial_pose)

    # 2. 특정 위치로 이동
    x = random.uniform(0.0, 0.5)  # Random x position
    y = random.uniform(0.0, 0.5)  # Random y position

    pick_pose.pose.position.x = x
    pick_pose.pose.position.y = y
    move_to_pose(panda, panda_arm, pick_pose)

    # 3. gripper 통해 객체 잡기 (구현x)
    open_gripper()

    close_gripper()

    x, y = place_pose_xy[i]
    place_pose.pose.position.x = x
    place_pose.pose.position.y = y

    # 4. 객체를 특정 위치로 이동
    move_to_pose(panda, panda_arm, place_pose)

    # 5. 객체 놓기
    open_gripper()


move_to_pose(panda, panda_arm, initial_pose)
```

## 실행 결과

실행 결과는 다음 영상과 같습니다.

<video width="700px" controls="true" autoplay="true" loop="true">
    <source src="https://github.com/chu-aie/cobots-2024/assets/104025187/609e8bdc-084e-425d-854a-ebe2abf24507
    " type="video/mp4">
</video>

## 한계점

1. MoveIt2의 장애물 객체 지원 부족:
   - 현재 MoveIt2는 Python 인터페이스를 통해 장애물 객체를 생성하는 기능을 지원하지 않습니다. 이로 인해 시뮬레이션 환경에서 장애물 객체를 생성하지 못했습니다.
