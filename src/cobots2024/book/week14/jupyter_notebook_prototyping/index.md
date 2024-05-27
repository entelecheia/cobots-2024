# Jupyter 노트북을 사용한 프로토타이핑

이 튜토리얼에서는 MoveIt 2 Python API와 함께 Jupyter 노트북을 사용하는 방법을 배웁니다. 이 튜토리얼은 다음과 같은 섹션으로 구성되어 있습니다:

- **시작하기**: 튜토리얼 설정 요구사항에 대한 개요입니다.
- **실행 파일 이해하기**: 실행 파일 명세에 대한 개요입니다.
- **노트북 설정**: 노트북 임포트와 구성입니다.
- **모션 플래닝 예제**: `moveit_py` API를 사용하여 모션을 계획하는 예제입니다.
- **원격 조작 예제**: `moveit_py` API를 사용하여 조이스틱으로 로봇을 원격 조작하는 예제입니다.

## 시작하기

이 튜토리얼을 완료하려면 MoveIt 2와 해당 튜토리얼이 포함된 colcon 작업 공간을 설정해야 합니다. 이러한 작업 공간을 설정하는 방법에 대한 개요는 [시작하기 가이드](../../week10/getting_started)에 제공되어 있습니다.

작업 공간을 설정한 후에는 다음 명령을 실행하여 이 튜토리얼의 코드를 실행할 수 있습니다(이 튜토리얼의 서보 섹션에는 PS4 듀얼쇼크가 필요합니다. 없는 경우 이 매개변수를 false로 설정하는 것을 고려하십시오):

```bash
ros2 launch moveit2_tutorials jupyter_notebook_prototyping.launch.py start_servo:=true
```

- 이렇게 하면 이 튜토리얼을 완료하는 데 필요한 노드가 실행됩니다.

- 중요한 점은 이 튜토리얼의 코드를 실행하기 위해 연결할 수 있는 Jupyter 노트북 서버도 실행된다는 것입니다.

- 브라우저에서 자동으로 Jupyter 노트북 인터페이스가 열리지 않으면 브라우저에서 http://localhost:8888로 이동하여 노트북 서버에 연결할 수 있습니다.

- 토큰을 입력하라는 메시지가 표시되는데, 이는 실행 파일을 실행할 때 터미널에 인쇄되며 이 토큰을 입력하여 노트북 서버에 연결할 수 있습니다.

- 토큰이 포함된 URL도 터미널 출력에 인쇄되므로, 토큰을 수동으로 입력하지 않고도 이 URL을 직접 사용하여 노트북 서버에 연결할 수 있습니다.

이 단계를 완료하면 튜토리얼을 진행할 준비가 된 것입니다. 노트북 코드를 실행하기 전에 노트북 인스턴스를 실행하는 방법을 이해할 수 있도록 실행 파일 명세에 대한 간단한 개요를 제공합니다.

## 실행 파일 이해하기

이 튜토리얼에서 사용되는 [실행 파일](https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping/launch/jupyter_notebook_prototyping.launch.py)이 다른 튜토리얼과 비교할 때 차이점은 Jupyter 노트북 서버를 시작한다는 것입니다.
우리는 일반적인 실행 파일 코드를 간략히 검토하고 주로 노트북 서버 시작에 초점을 맞출 것입니다.

필요한 패키지 임포트:

```python
import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
```

YAML 파일을 로드하기 위한 유틸리티 정의:

```python
def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        try:
                with open(absolute_file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError:
                return None
```

서보 노드를 시작하기 위한 실행 인자 정의:

```python
start_servo = LaunchConfiguration('start_servo')

start_servo_arg = DeclareLaunchArgument(
        'start_servo',
        default_value='false',
        description='Start the servo node.')
```

MoveIt 구성을 정의합니다. 이 단계는 나중에 노트북을 구성할 때도 중요합니다:

```python
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
                        "jupyter_notebook_prototyping.yaml"
                )
        )
        .to_moveit_configs()
)
```

MoveIt 구성을 정의하면 다음 노드 집합을 시작합니다:

- **rviz_node**: 시각화를 위해 rviz2를 시작합니다.
- **static_tf**: 월드 프레임과 판다 베이스 프레임 사이의 정적 변환을 발행합니다.
- **robot_state_publisher**: 업데이트된 로봇 상태 정보(변환)를 발행합니다.
- **ros2_control_node**: 관절 그룹을 제어하는 데 사용됩니다.

```python
rviz_config_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "config", "jupyter_notebook_prototyping.rviz",
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
        parameters=[ros2_controllers_path],
        remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
        ],
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
                output="screen",)
                ]
```

이러한 각 노드의 설정을 정의한 후에는 Jupyter 노트북 서버를 시작하는 프로세스도 정의합니다:

```python
notebook_dir = os.path.join(get_package_share_directory("moveit2_tutorials"), "src")
start_notebook = ExecuteProcess(
        cmd=["cd {} && python3 -m notebook".format(notebook_dir)],
        shell=True,
        output="screen",
)
```

서보를 시작하려는 경우 조이와 서보 노드도 정의합니다. 마지막으로 `LaunchDescription`을 반환합니다:

```python
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
```

서보를 시작하지 않는 경우 정의한 다른 모든 노드와 프로세스를 포함하는 실행 설명을 반환합니다:

```python
return LaunchDescription(
        [
        start_servo_arg,
        static_tf,
        robot_state_publisher,
        rviz_node,
        ros2_control_node,
        start_notebook,
        ]
        + load_controllers
        )
```

## 노트북 설정

이제 Jupyter 노트북 서버를 실행했으므로 노트북의 코드 실행을 시작할 수 있습니다. 첫 번째 단계는 필요한 패키지를 임포트하는 것입니다:

```python
import os
import sys
import yaml
import rclpy
import numpy as np

# 메시지 라이브러리
from geometry_msgs.msg import PoseStamped, Pose

# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# 설정 파일 라이브러리
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
```

필요한 패키지를 임포트한 후에는 `moveit_py` 노드 구성을 정의해야 합니다. 이는 다음과 같이 `MoveItConfigsBuilder`를 사용하여 수행합니다:

```python
moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
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
).to_dict()
```

여기서 생성된 구성 인스턴스를 딕셔너리로 변환하여 `moveit_py` 노드를 초기화하는 데 사용할 수 있습니다. 마지막으로 `moveit_py` 노드를 초기화합니다:

```python
# rclpy 초기화(로깅 목적으로만)
rclpy.init()

# moveit_py 인스턴스와 panda_arm에 대한 planning 컴포넌트 인스턴스화
panda = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
panda_arm = panda.get_planning_component("panda_arm")
```

## 모션 플래닝 예제

먼저 나중에 계획된 궤적을 계획하고 실행할 때 사용할 도우미 함수를 만듭니다:

```python
def plan_and_execute(
        robot,
        planning_component,
        single_plan_parameters=None,
        multi_plan_parameters=None,
):
        """모션을 계획하고 실행하는 도우미 함수."""
        # 목표로 계획
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

        # 계획 실행
        if plan_result:
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                print("Planning failed")
```

노트북 내에서 간단한 모션의 계획과 실행을 시연할 수 있습니다:

```python
# 미리 정의된 상태를 사용하여 plan start state 설정
panda_arm.set_start_state("ready")

# 미리 정의된 상태를 사용하여 pose goal 설정
panda_arm.set_goal_state(configuration_name = "extended")

# 목표로 계획
plan_and_execute(panda, panda_arm)
```

우리는 대화식으로 모션 플래닝을 수행할 수 있습니다(모션 플래닝 API의 자세한 내용은 모션 플래닝 튜토리얼 참조). 코드를 개발하다가 다음과 같은 실수를 할 수 있다고 가정해 보겠습니다:

```python
# 미리 정의된 상태를 사용하여 plan start state 설정
panda_arm.set_start_state("ready") # 이는 현재 로봇 구성과 충돌하여 오류가 발생합니다

# 이번에는 pose 메시지를 사용하여 목표 설정
pose_goal = PoseStamped()
pose_goal.header.frame_id = "panda_link0"
pose_goal.pose.orientation.w = 1.0
pose_goal.pose.position.x = 0.28
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.5
panda_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "panda_link8")

# 목표로 계획
plan_and_execute(panda, panda_arm)
```

노트북을 사용하고 있기 때문에 파일을 다시 컴파일할 필요 없이 이 실수를 쉽게 수정할 수 있습니다. 위의 노트북을 아래와 일치하도록 편집하고 셀을 다시 실행하기만 하면 됩니다:

```python
# 현재 상태를 plan start state로 설정
panda_arm.set_start_state_to_current_state()

# 이번에는 pose 메시지를 사용하여 목표 설정
pose_goal = PoseStamped()
pose_goal.header.frame_id = "panda_link0"
pose_goal.pose.orientation.w = 1.0
pose_goal.pose.position.x = 0.28
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.5
panda_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "panda_link8")

# 목표로 계획
plan_and_execute(panda, panda_arm)
```

## 원격 조작 예제

사용자는 로봇의 실시간 원격 조작을 수행하고 싶을 수도 있습니다. Python API를 사용하면 모든 프로세스를 종료하고 다시 실행하지 않고도 대화식으로 원격 조작을 시작/중지할 수 있습니다. 이 예제에서는 로봇을 원격 조작하고, 모션 플래닝을 수행한 다음, 다시 로봇을 원격 조작하는 동기 부여 예제를 통해 노트북에서 이것이 어떻게 가능한지 보여줍니다.

이 섹션에서는 `moveit_py`와 원격 조작을 지원하는 장치가 필요합니다. 여기서는 PS4 듀얼쇼크 컨트롤러를 활용합니다.

로봇 원격 조작을 시작하려면 PS4 듀얼쇼크 컨트롤러를 원격 조작 장치로 인스턴스화합니다.

```python
from moveit.servo_client.devices.ps4_dualshock import PS4DualShockTeleop

# 원격 조작 장치 인스턴스화
ps4 = PS4DualShockTeleop(ee_frame_name="panda_link8")

# 로봇 원격 조작 시작
ps4.start_teleop()
```

로봇을 기본 구성으로 되돌리기 위해 모션 플래닝을 수행하려면 로봇 원격 조작을 중지하고 아래에 시연된 대로 기존 모션 플래닝 API를 활용하면 됩니다:

```python
# 로봇 원격 조작 중지
ps4.stop_teleop()

# 계획 및 실행
# 미리 정의된 상태를 사용하여 plan start state 설정
panda_arm.set_start_state_to_current_state()

# 미리 정의된 상태를 사용하여 pose goal 설정
panda_arm.set_goal_state(configuration_name = "ready")

# 목표로 계획
plan_and_execute(panda, panda_arm)
```

이것은 로봇을 기본 구성으로 되돌립니다. 이 구성에서 우리는 다시 한 번 로봇 원격 조작을 시작할 수 있습니다:

```python
ps4.start_teleop()
```

이 튜토리얼에서는 MoveIt 2 Python API와 Jupyter 노트북을 사용하여 모션 플래닝 및 원격 조작을 수행하는 방법을 살펴보았습니다. Jupyter 노트북은 파일을 다시 컴파일할 필요 없이 빠른 프로토타이핑과 대화식 개발을 가능하게 하므로 로봇 개발에 유용한 도구가 될 수 있습니다.

MoveIt 2와 Python을 사용한 모션 플래닝에 대해 더 알아보려면 MoveIt 2 문서와 예제를 참조하는 것이 좋습니다.
