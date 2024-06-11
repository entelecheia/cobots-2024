# 노드 이해하기

## 배경

### 1. ROS 2 그래프

앞으로 몇 가지 튜토리얼을 통해 "ROS (2) 그래프"라고 하는 일련의 핵심 ROS 2 개념에 대해 배우게 될 것입니다.

ROS 그래프는 데이터를 동시에 함께 처리하는 ROS 2 요소의 네트워크입니다. 모든 실행 파일과 실행 파일 간의 연결을 모두 매핑하고 시각화한다면 이를 모두 포괄합니다.

### 2. ROS 2의 노드

ROS의 각 노드는 휠 모터 제어 또는 레이저 거리 측정기의 센서 데이터 게시와 같은 단일 모듈식 목적을 담당해야 합니다. 각 노드는 주제, 서비스, 액션 또는 매개변수를 통해 다른 노드와 데이터를 주고받을 수 있습니다.

![](figs/Nodes-TopicandService.gif)

전체 로봇 시스템은 협력하여 작동하는 많은 노드로 구성됩니다. ROS 2에서는 단일 실행 파일(C++ 프로그램, Python 프로그램 등)에 하나 이상의 노드가 포함될 수 있습니다.

## 작업

### 1. ros2 run

`ros2 run` 명령은 패키지에서 실행 파일을 시작합니다.

```console
ros2 run <package_name> <executable_name>
```

turtlesim을 실행하려면 새 터미널을 열고 다음 명령을 입력하세요:

```console
ros2 run turtlesim turtlesim_node
```

이전 강의에서 보았듯이 turtlesim 창이 열립니다.

여기서 패키지 이름은 `turtlesim`이고 실행 파일 이름은 `turtlesim_node`입니다.

그러나 아직 노드 이름은 모릅니다. `ros2 node list`를 사용하여 노드 이름을 찾을 수 있습니다.

### 2. ros2 node list

`ros2 node list`는 실행 중인 모든 노드의 이름을 보여줍니다. 이는 노드와 상호 작용하려는 경우 또는 많은 노드를 실행하는 시스템이 있고 이를 추적해야 하는 경우에 특히 유용합니다.

turtlesim이 다른 터미널에서 계속 실행되는 동안 새 터미널을 열고 다음 명령을 입력하세요:

```console
ros2 node list
```

터미널에 노드 이름이 반환됩니다:

```console
/turtlesim
```

다른 새 터미널을 열고 다음 명령을 사용하여 teleop 노드를 시작하세요:

```console
ros2 run turtlesim turtle_teleop_key
```

여기서는 다시 `turtlesim` 패키지를 참조하지만 이번에는 `turtle_teleop_key`라는 실행 파일을 대상으로 합니다.

`ros2 node list`를 실행한 터미널로 돌아가 다시 실행하세요. 이제 두 개의 활성 노드 이름이 표시됩니다:

```console
/turtlesim
/teleop_turtle
```

#### 2.1 리맵핑

`리맵핑 <https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules>`\_\_을 사용하면 노드 이름, 주제 이름, 서비스 이름 등의 기본 노드 속성을 사용자 지정 값으로 재할당할 수 있습니다. 마지막 튜토리얼에서는 `turtle_teleop_key`에서 리맵핑을 사용하여 cmd_vel 주제를 변경하고 **turtle2**를 대상으로 지정했습니다.

이제 `/turtlesim` 노드의 이름을 재할당해 보겠습니다. 새 터미널에서 다음 명령을 실행하세요:

```console
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

turtlesim에서 다시 `ros2 run`을 호출하고 있으므로 다른 turtlesim 창이 열립니다. 그러나 이제 `ros2 node list`를 실행한 터미널로 돌아가 다시 실행하면 세 개의 노드 이름이 표시됩니다:

```console
/my_turtle
/turtlesim
/teleop_turtle
```

### 3. ros2 node info

이제 노드의 이름을 알았으므로 다음을 사용하여 노드에 대한 더 많은 정보에 액세스할 수 있습니다:

```console
ros2 node info <node_name>
```

최신 노드인 `my_turtle`을 검사하려면 다음 명령을 실행하세요:

```console
ros2 node info /my_turtle
```

`ros2 node info`는 구독자, 게시자, 서비스 및 액션 목록을 반환합니다. 즉, 해당 노드와 상호 작용하는 ROS 그래프 연결입니다. 출력은 다음과 같아야 합니다:

```console
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

이제 `/teleop_turtle` 노드에서 동일한 명령을 실행해 보고 `my_turtle`과 연결이 어떻게 다른지 확인하세요.

다가오는 튜토리얼에서 메시지 유형을 포함한 ROS 그래프 연결 개념에 대해 자세히 배우게 될 것입니다.

## 요약

노드는 로봇 시스템에서 단일 모듈식 목적을 제공하는 기본 ROS 2 요소입니다.

이 튜토리얼에서는 `turtlesim` 패키지에서 만든 노드를 `turtlesim_node`와 `turtle_teleop_key` 실행 파일을 실행하여 활용했습니다.

`ros2 node list`를 사용하여 활성 노드 이름을 검색하고 `ros2 node info`를 사용하여 단일 노드를 내부 점검하는 방법을 배웠습니다. 이러한 도구는 복잡하고 실제적인 로봇 시스템에서 데이터 흐름을 이해하는 데 매우 중요합니다.
