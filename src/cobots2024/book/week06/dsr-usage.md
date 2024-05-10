# DSR 사용방법

## Joint State Publish

Joint State Publish는 컨트롤러 없이 독립적으로 실행할 수 있습니다.
로봇 모델과 `joint_state_publisher_gui` 패키지를 사용하여 Rviz에서 로봇이 움직이는 것을 볼 수 있습니다.

```bash
ros2 launch dsr_launcher2 dsr_joint_state_pub.launch.py model:=a0912 color:=blue
```

<img src="https://user-images.githubusercontent.com/47092672/97652654-40da3b00-1aa2-11eb-8621-2a36e3159de0.png" width="70%">

`dsr_joint_state_pub.launch.py` 파일을 실행하면 지정된 로봇 모델(예: a0912)과 색상(예: blue)을 가진 로봇이 Rviz에 표시됩니다.
이 때 joint_state_publisher_gui를 통해 로봇의 joint를 직접 조작할 수 있습니다.
이는 실제 로봇이나 컨트롤러 없이도 로봇의 동작을 시뮬레이션 해볼 수 있는 편리한 기능입니다.

## Virtual Mode

Launch 파일의 "mode" 인자를 virtual로 설정하면, launch.py를 실행할 때 자동으로 DRCF 에뮬레이터가 실행됩니다.

### dsr_control2 노드 실행

아래 명령어를 사용하여 Control Node를 실행할 수 있습니다.

```bash
ros2 launch dsr_launcher2 single_robot_rviz.launch.py model:=a0912 color:=blue
```

이 명령어는 `single_robot_rviz.launch.py` 파일을 실행하여 지정된 로봇 모델(a0912)과 색상(blue)을 가진 로봇을 Rviz에 표시합니다.
동시에 DRCF 에뮬레이터와 dsr_control2 노드가 자동으로 실행되어 가상 환경에서 로봇을 제어할 수 있게 됩니다.

### 예제 스크립트 실행

dsr_example2 패키지에 포함된 예제 스크립트를 사용하여 로봇을 구동할 수 있습니다.
컨트롤러와 로봇이 정상적으로 연결되었는지 확인한 후, 아래 명령어를 입력하세요.

```bash
ros2 run dsr_example2_py dsr_service_motion_simple
```

이 명령어는 `dsr_service_motion_simple` 예제 스크립트를 실행합니다.
이 스크립트는 로봇의 간단한 모션을 demonstrates합니다.
가상 모드에서는 에뮬레이터와 연동되어 가상 로봇이 동작하는 것을 확인할 수 있습니다.

## Real Mode

실제 로봇을 구동하려면 **real mode**를 사용하세요.
로봇 컨트롤러의 기본 IP는 _192.168.127.100_ 이고 포트는 _12345_ 입니다.

### dsr_control2 노드 실행

```bash
ros2 launch dsr_launcher2 single_robot_rviz.launch.py mode:=real host:=192.168.68.10 port:=12345
```

이 명령어는 `single_robot_rviz.launch.py` 파일을 실행하여 실제 로봇과 연결합니다.
`mode` 인자를 real로 설정하고, `host`와 `port` 인자에 실제 로봇 컨트롤러의 IP와 포트를 지정합니다.
이렇게 하면 dsr_control2 노드가 실제 로봇 컨트롤러와 통신하여 실제 로봇을 제어할 수 있게 됩니다.

### 예제 스크립트 실행

dsr_example2 패키지에 포함된 예제 스크립트를 사용하여 로봇을 구동할 수 있습니다.
컨트롤러와 로봇이 정상적으로 연결되었는지 확인한 후, 아래 명령어를 입력하세요.

```bash
ros2 run dsr_example2_py dsr_service_motion_simple
```

이 명령어는 `dsr_service_motion_simple` 예제 스크립트를 실행합니다.
이 스크립트는 로봇의 간단한 모션을 demonstrates합니다.
실제 모드에서는 실제 로봇이 동작하는 것을 확인할 수 있습니다.

---

## dsr_launcher2

dsr_launcher2 패키지에는 control node, rviz, gazebo를 연동할 수 있는 launch.py 파일이 포함되어 있습니다.
따라서 launch 파일을 실행하기 전에 실제 로봇 컨트롤러 또는 에뮬레이터와 연결이 필요합니다.
아래 명령어로 시뮬레이션을 수행할 수 있습니다.

```bash
ros2 launch dsr_launcher2 single_robot_rviz.launch.py
```

<img src="https://user-images.githubusercontent.com/47092672/97654894-3f5f4180-1aa7-11eb-83f0-90eb071d1f60.gif" width="70%">

이 명령어는 `single_robot_rviz.launch.py` 파일을 실행하여 가상 모드에서 로봇을 Rviz에 표시합니다.
DRCF 에뮬레이터와 dsr_control2 노드가 자동으로 실행되어 가상 환경에서 로봇을 제어할 수 있습니다.

```bash
ros2 launch dsr_launcher2 single_robot_gazebo.launch.py
```

<img src="https://user-images.githubusercontent.com/47092672/99232226-fe9c5200-2834-11eb-8719-f87cc56d55c7.gif" width="70%">

이 명령어는 `single_robot_gazebo.launch.py` 파일을 실행하여 가상 모드에서 로봇을 Gazebo 시뮬레이터에 표시합니다.
Gazebo 시뮬레이터와 dsr_control2 노드가 자동으로 실행되어 물리 기반 시뮬레이션 환경에서 로봇을 제어할 수 있습니다.

## Moveit

moveit2 패키지를 사용하려면 아래 패키지들을 설치해야 합니다.

```bash
cd /opt/ros2_ws/src
git clone https://github.com/ros-planning/moveit2
git clone -b ros2 --single-branch https://github.com/ros-planning/warehouse_ros
git clone -b ros2 --single-branch  https://github.com/ros-planning/warehouse_ros_mongo
git clone -b ros2 --single-branch https://github.com/ros-planning/srdfdom
git clone -b ros2 --single-branch https://github.com/ros-planning/geometric_shapes
git clone -b use_new_joint_handle https://github.com/ShotaAk/fake_joint
```

ROS2 패키지와 호환되는 fake_joint 패키지를 빌드하기 위해 아래와 같은 추가 작업을 해주세요.

```bash
cd /opt/ros2_ws/src
cp doosan-robot2/common2/resource/fake_joint_driver_node.cpp fake_joint/fake_joint_driver/src/fake_joint_driver_node.cpp
```

아래 명령어를 통해 의존성 패키지를 설치할 수 있습니다.

```bash
cd /opt/ros2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro foxy -y
rm -rf src/doosan-robot2/moveit_config_*/COLCON_IGNORE    # colcon build 전에 moveit 패키지를 활성화하는 명령어
colcon build
. install/setup.bash
```

moveit_config 패키지에서 fake_controller로 moveit2를 실행할 수 있습니다.
아래 명령어 형식을 참고하세요.

> ros2 launch moveit*config*<robot_model> <robot_model>.launch.py

```bash
ros2 launch moveit_config_m1013 m1013.launch.py
```

moveit2는 제어 노드와 연동하여 실행할 수 있습니다. 실제 로봇과 연동하여 moveit2의 planning 기능을 사용하려면 아래 명령어를 입력하세요.
위에서 언급한 **dsr_control2 노드 실행** 항목의 인자들을 참고하세요.

```bash
ros2 launch dsr_control2 dsr_moveit2.launch.py
```

<img src="https://user-images.githubusercontent.com/47092672/102734069-3f324280-4382-11eb-9165-cdec6b52de17.gif" width="80%">

이 명령어는 `dsr_moveit2.launch.py` 파일을 실행하여 dsr_control2 노드와 moveit2를 연동합니다.
실제 로봇과 연결된 상태에서 moveit2의 모션 플래닝 기능을 사용할 수 있습니다.
Rviz에서 moveit2의 인터페이스를 통해 로봇의 목표 자세를 설정하면, moveit2가 경로를 계획하고 dsr_control2 노드를 통해 실제 로봇이 해당 경로를 따라 움직이게 됩니다.

이상으로 doosan-robot2 ROS2 패키지의 사용 방법에 대해 자세히 알아보았습니다.
Virtual Mode와 Real Mode에서의 dsr_control2 노드 실행 방법, 예제 스크립트를 통한 로봇 구동, dsr_launcher2 패키지를 이용한 시뮬레이션, 그리고 moveit2와의 연동까지 doosan-robot2 패키지가 제공하는 다양한 기능들을 확인할 수 있었습니다.
이제 여러분만의 로봇 프로젝트에 doosan-robot2 패키지를 활용해보세요!
