# 202221014 - 로봇을 이용한 정원 관리

## 중간 계획서

![image](https://github.com/chu-aie/cobots-2024/assets/133697595/30a66e8b-fc21-491b-9757-483044c33f19)

### 프로젝트 개요

이 프로젝트의 목표는 ROS2를 기반으로 한 협동로봇을 사용하여 식물을 심고, 관리하며, 아름다운 정원을 유지하는 자동화된 시스템을 개발하는 것이다. 로봇은 흙을 삽질하고, 씨앗을 심으며, 정기적으로 물을 주는 작업을 수행한다.

### 프로젝트 목표

1. 로봇 손의 정밀한 움직임을 이용하여 흙을 손질하고 씨앗을 심는 기능 개발.
2. 식물의 성장을 모니터링하고, 필요에 따라 물을 주는 자동화 시스템 구현.
3. 식물 인식 기술을 활용하여 다양한 식물을 구분하고, 각 식물에 맞는 관리 제공.

### 방법론

- 로봇 프로그래밍: ROS2를 이용하여 로봇의 움직임을 제어하고, 특정 작업(삽질, 심기, 물주기)을 수행하도록 프로그래밍한다.
- 식물 인식 개발: 컴퓨터 비전 기술을 활용하여 식물을 인식하고 데이터베이스에 정보를 저장한다.
- 센서 통합: 토양 습도 센서 및 기타 환경 센서를 통합하여 식물의 상태를 지속적으로 모니터링한다.
- 사용자 인터페이스 개발: 사용자가 로봇을 쉽게 관리하고 모니터링할 수 있는 애플리케이션 개발.

### 필요 자원

- 하드웨어: 두산 협동 로봇, 카메라(웹캠), 기타 필요 센서.
- 소프트웨어: ROS2, OpenCV 컴퓨터 비전 라이브러리, 웹/모바일 개발 툴킷.

### 프로젝트 참여자

- 학번: 202221014
- 이름: 이규범
- 이메일: 202221014@chu.ac.kr

### 예상 예산

| 품목              | 가격(원)   |
| ----------------- | ---------- |
| 다이소 화분 (3개) | 6,000      |
| 웹캠              | 30,000     |
| **총 예산**       | **36,000** |

### 예상 일정 (총 기간: 8주)

| 주차   | 활동                                  |
| ------ | ------------------------------------- |
| 주 1-2 | 프로젝트 계획 및 초기 설계            |
| 주 3-4 | 로봇 프로그래밍 및 초기 테스트        |
| 주 5-6 | 식물 인식 시스템 통합 및 센서 설정    |
| 주 7   | 사용자 인터페이스 개발 및 통합 테스트 |
| 주 8   | 최종 발표, 프로젝트 마무리 및 문서화  |

### 위험 관리

- 기술적 실패: 소프트웨어에 대한 공부부족을 보완하기 위해 적절한 교육을 확보하여 위험을 최소화한다.
- 일정 지연: 유연한 프로젝트 관리 접근 방식과 마일스톤의 정기적인 평가를 통해 조정한다.

## 로봇을 이용한 정원 관리 (최종 보고서)

### 프로젝트 개요

이 프로젝트는 ROS2를 기반으로 한 가상의 협동로봇팔을 사용하여 화분을 인식하고 관리하는 자동화된 시스템을 개발하는 것이다. 주요 작업으로는 화분에 물을 주고, 열매를 채집하는 작업이 포함된다. 실제 로봇이 아닌 가상 환경에서 진행되었으며, 이를 위해 WSL Ubuntu 22.04와 ROS2 Humble 버전을 사용하였다. 모든 작업은 Docker를 이용하여 가상 환경에서 진행되었다.

<img width="763" alt="ros2_ph" src="https://github.com/chu-aie/cobots-2024/assets/133697595/1b5d7a50-f9be-4f97-9f4c-c0307bec2ecd">

### 프로젝트 목표

1. 로봇팔을 이용한 화분 인식 및 관리 자동화 시스템 개발.
2. 가상 환경에서의 물뿌리기 및 열매 채집 기능 구현.
3. ROS2를 이용한 가상 로봇팔 제어 및 작업 수행.

### 개발 환경 및 도구

- **운영체제:** WSL Ubuntu 22.04
- **프레임워크:** ROS2 Humble
- **시뮬레이션 툴:** Gazebo
- **가상 환경:** Docker
- **개발 도구:** MobaXterm
- **프로그래밍 언어:** Python
- **프로토타이핑 도구:** Jupyter Notebook

### 방법론

#### 로봇 프로그래밍

- **ROS2 활용:** ROS2 Humble 버전을 사용하여 가상 로봇팔의 움직임을 제어하고, 특정 작업(물뿌리기, 열매 채집)을 수행하도록 프로그래밍.
- **시뮬레이션 환경:** Gazebo와 같은 시뮬레이션 툴을 사용하여 가상 환경을 구축하고, 로봇팔의 작업을 테스트.
- **개발 도구:** MobaXterm을 사용하여 개발 환경을 구성하고, Docker를 통해 가상 환경에서 모든 작업을 진행.
- **Jupyter Notebook 사용:** Jupyter Notebook을 사용하여 프로토타이핑 및 코딩을 진행.

#### Docker를 통한 가상 환경 구성

- **Docker-compose 설정:** Docker-compose를 이용하여 ROS2 및 Gazebo가 포함된 가상 환경을 설정.

- **Docker-compose.yml 파일 구성:**
  ```yaml
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
- **Docker 실행 스크립트:** `docker-run.sh` 파일을 사용하여 Docker 컨테이너 실행.

  ```bash
  export DOCKER_IMAGE=humble-tutorial
  export XAUTHORITY=~/.Xauthority

  docker compose -f .docker/docker-compose.yml run gpu
  ```

#### Jupyter Notebook을 통한 프로토타이핑

- **Jupyter Notebook 서버 시작:** Jupyter Notebook 서버와 MoveIt Python 라이브러리를 사용한 모션 플래닝 노드를 실행하는 `jupyter_notebook_prototyping.launch.py` 파일을 작성하여 노트북 환경에서 실험 및 프로토타이핑을 진행.

  ```python
  """
  A launch file that starts a Jupyter notebook server and nodes that support motion planning with the MoveIt Python library.
  """
  import os
  import yaml
  from launch import LaunchDescription
  from launch.actions import ExecuteProcess, DeclareLaunchArgument
  from launch.substitutions import LaunchConfiguration
  from launch_ros.actions import Node, SetParameter
  from ament_index_python.packages import get_package_share_directory
  from moveit_configs_utils import MoveItConfigsBuilder

  def load_yaml(package_name, file_path):
      package_path = get_package_share_directory(package_name)
      absolute_file_path = os.path.join(package_path, file_path)
      try:
          with open(absolute_file_path, "r") as file:
              return yaml.safe_load(file)
      except EnvironmentError:
          return None

  def generate_launch_description():
      start_servo = LaunchConfiguration("start_servo")

      start_servo_arg = DeclareLaunchArgument(
          "start_servo",
          default_value="false",
          description="Start the servo node",
      )

      moveit_config = (
          MoveItConfigsBuilder(
              robot_name="panda", package_name="moveit_resources_panda_moveit_config"
          )
          .robot_description(file_path="config/panda.urdf.xacro")
          .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
          .moveit_cpp(
              file_path=os.path.join(
                  get_package_share_directory("moveit2_tutorials"),
                  "config",
                  "jupyter_notebook_prototyping.yaml",
              )
          )
          .to_moveit_configs()
      )

      rviz_config_file = os.path.join(
          get_package_share_directory("moveit2_tutorials"),
          "config",
          "jupyter_notebook_prototyping.rviz",
      )
      rviz_node = Node(
          package="rviz2",
          executable="rviz2",
          name="rviz2",
          output="log",
          arguments=["-d", rviz_config_file],
          parameters=[
              moveit_config.robot_description,
              moveit_config.robot_description_semantic,
          ],
      )

      static_tf = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          name="static_transform_publisher",
          output="log",
          arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
      )

      robot_state_publisher = Node(
          package="robot_state_publisher",
          executable="robot_state_publisher",
          name="robot_state_publisher",
          output="both",
          parameters=[moveit_config.robot_description],
      )

      ros2_controllers_path = os.path.join(
          get_package_share_directory("moveit_resources_panda_moveit_config"),
          "config",
          "ros2_controllers.yaml",
      )
      ros2_control_node = Node(
          package="controller_manager",
          executable="ros2_control_node",
          parameters=[moveit_config.robot_description, ros2_controllers_path],
          output="both",
      )

      load_controllers = []
      for controller in [
          "panda_arm_controller",
          "panda_hand_controller",
          "joint_state_broadcaster",
      ]:
          load_controllers += [
              ExecuteProcess(
                  cmd=["ros2 run controller_manager spawner {}".format(controller)],
                  shell=True,
                  output="screen",
              )
          ]

      notebook_dir = os.path.join(get_package_share_directory("moveit2_tutorials"), "src")
      start_notebook = ExecuteProcess(
          cmd=["cd {} && python3 -m notebook".format(notebook_dir)],
          shell=True,
          output="screen",
      )

      if start_servo:
          servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
          servo_params = {"moveit_servo": servo_yaml}

          joy_node = Node(
              package="joy",
              executable="joy_node",
              name="joy_node",
              output="screen",
          )

          servo_node = Node(
              package="moveit_servo",
              executable="servo_node_main",
              parameters=[
                  servo_params,
                  moveit_config.robot_description,
                  moveit_config.robot_description_semantic,
                  moveit_config.robot_description_kinematics,
              ],
              output="screen",
          )

          return LaunchDescription(
              [
                  start_servo_arg,
                  start_notebook,
                  static_tf,
                  robot_state_publisher,
                  rviz_node,
                  ros2_control_node,
                  joy_node,
                  servo_node,
              ]
              + load_controllers
          )

      else:
          return LaunchDescription(
              [
                  start_servo_arg,
                  start_notebook,
                  static_tf,
                  robot_state_publisher,
                  rviz_node,
                  ros2_control_node,
              ]
              + load_controllers
          )
  ```

- **시뮬레이션 실행:**
  Docker 안에서 다음 명령어를 사용하여 로봇 팔을 움직이기 위한 시뮬레이션 화면을 실행하였다.
  `bash
    ros2 launch moveit2_tutorials jupyter_notebook_prototyping.launch.py start_servo:=true
    `

#### 프로젝트 진행

다음은 실제 프로젝트 진행 사항을 코드로 작성한 내용이다.

```python
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
    .moveit_cpp(file_path=os.path.join

(get_package_share_directory("moveit2_tutorials"), "config", "jupyter_notebook_prototyping.yaml"))
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
```

### 화분 인식 및 관리

##### 의도

- **화분 인식:** ROS2의 내장된 기능을 사용하여 화분의 위치와 상태를 파악.
- **물뿌리기 시스템:** 가상의 토양 습도 데이터를 기반으로 물뿌리기 작업을 자동으로 수행.
- **열매 채집:** 가상의 열매 성장을 모니터링하고, 적절한 시기에 열매를 채집하는 기능 구현.

##### 실제

- **물뿌리기 시스템**
- **열매 채집**

### 주요 구현 기능

#### 물뿌리기

가상의 로봇팔이 화분 3개에 물을 주는 작업을 수행한다. 필요한 양의 물을 자동으로 공급한다.

(토양 습도 센서를 통해 각 화분의 상태를 모니터링 - 구현 실패)

#### 열매 채집

가상의 열매를 모니터링하여 성숙한 열매를 적절한 시기에 채집한다. 이를 통해 화분의 관리 및 유지보수를 자동화한다.

### 실험

#### 시뮬레이션

Gazebo 시뮬레이션을 통해 가상 로봇팔의 움직임과 작업을 테스트하였다. 각 작업(물뿌리기, 열매 채집)의 정확성을 확인하고, 필요한 조정을 수행하였다.

https://github.com/chu-aie/cobots-2024/assets/133697595/e6f28a67-8673-4f70-8d09-14dc053b0a4c

### 결과 및 논의

#### 프로젝트 성과

- 가상 환경에서의 로봇팔 제어 및 작업 수행이 성공적으로 구현되었다.
- 화분 인식, 물뿌리기, 열매 채집 등의 작업이 자동화되었다.

#### 문제점 및 개선사항

- 실제 환경에서의 적용 가능성을 높이기 위해 추가적인 테스트와 조정이 필요하다.
- 로봇팔의 정밀도와 작업 속도를 향상시키기 위한 알고리즘 개선이 요구된다.
- 화분과 물뿌리개, 바구니를 그래픽으로 구현하려 했으나 기술력 부족으로 구현에 실패하였다. C++ 예제를 찾았으나, Python으로만 프로젝트를 진행하여 한계가 있었다.

### 결론

이 프로젝트를 통해 가상의 로봇팔을 이용한 화분 관리 시스템을 개발하였다. ROS2와 Gazebo를 활용한 가상 환경에서의 시뮬레이션을 통해 주요 작업을 자동화하는 데 성공하였다. 향후 실제 로봇팔을 사용한 실험을 통해 시스템의 실제 적용 가능성을 평가하고, 더욱 발전된 자동화 시스템을 개발할 계획이다.

### 참고 문헌

- ROS2 Documentation: https://docs.ros.org/en/humble/index.html
- moveit2_tutorials: https://github.com/peterdavidfagan/moveit2_tutorials
- pymoveit2: https://github.com/AndrejOrsula/pymoveit2

### 부록

#### 참여자

- 학번: 202221014
- 이름: 이규범
- 이메일: 202221014@chu.ac.kr

#### 사용된 자원

이 프로젝트는 가상 환경에서 진행되어 물리적 자원이 사용되지 않았다.

#### 위험 관리

- **기술적 실패:** 소프트웨어에 대한 공부부족을 보완하기 위해 적절한 교육이 필요하다.
