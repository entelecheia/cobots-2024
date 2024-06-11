# 첫 번째 C++ MoveIt 프로젝트

이 튜토리얼에서는 MoveIt을 사용하여 첫 번째 C++ 애플리케이션을 작성하는 과정을 단계별로 안내합니다.

## 사전 요구 사항

- ROS 2 설치 및 환경 설정이 완료되어 있어야 합니다.
- ROS 2의 기본 개념을 이해하고 있어야 합니다. 이를 위해 "간단한 발행자와 구독자 작성하기(C++)"까지의 ROS 2 공식 튜토리얼을 완료하는 것이 좋습니다.

## 단계

### 1. 패키지 생성하기

터미널을 열고 `ros2` 명령이 작동하도록 ROS 2 환경을 설정합니다.

`ws_moveit2` 디렉토리로 이동한 후 `src` 디렉토리로 들어갑니다. 소스 코드는 `src` 디렉토리에 위치하게 됩니다.

ROS 2 명령행 도구를 사용하여 새 패키지를 생성합니다:

```bash
ros2 pkg create \
 --build-type ament_cmake \
 --dependencies moveit_ros_planning_interface rclcpp \
 --node-name hello_moveit hello_moveit
```

이 명령을 실행하면 새 디렉토리에 몇 가지 파일이 생성됩니다.

`moveit_ros_planning_interface`와 `rclcpp`를 의존성으로 추가했습니다. 이렇게 하면 `package.xml`과 `CMakeLists.txt` 파일에 필요한 변경 사항이 생성되어 이 두 패키지에 의존할 수 있게 됩니다.

선호하는 편집기에서 `ws_moveit2/src/hello_moveit/src/hello_moveit.cpp` 파일을 엽니다.

### 2. ROS 노드와 실행기 생성하기

첫 번째 코드 블록은 약간의 보일러플레이트 코드이지만 ROS 2 튜토리얼에서 본 적이 있을 것입니다.

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // ROS 초기화 및 노드 생성
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // ROS 로거 생성
  auto const logger = rclcpp::get_logger("hello_moveit");

  // 다음 단계가 여기에 들어갑니다

  // ROS 종료
  rclcpp::shutdown();
  return 0;
}
```

#### 2.1. 빌드 및 실행

다음 단계로 넘어가기 전에 프로그램을 빌드하고 실행하여 모든 것이 올바른지 확인합니다.

`ws_moveit2` 디렉토리로 돌아가서 다음 명령을 실행합니다:

```bash
colcon build --mixin debug
```

빌드가 성공하면 **새 터미널을 열고** 해당 터미널에서 작업 공간 환경 스크립트를 설정하여 프로그램을 실행할 수 있도록 합니다.

```bash
cd ~/ws_moveit2
source install/setup.bash
```

프로그램을 실행하고 출력을 확인합니다.

```bash
ros2 run hello_moveit hello_moveit
```

프로그램이 오류 없이 실행되고 종료되어야 합니다.

#### 2.2. 코드 살펴보기

맨 위에 포함된 헤더는 표준 C++ 헤더와 나중에 사용할 ROS 및 MoveIt 헤더입니다.

그 다음, rclcpp를 초기화하는 일반적인 호출이 있고 노드를 생성합니다.

```cpp
auto const node = std::make_shared<rclcpp::Node>(
  "hello_moveit",
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
```

첫 번째 인수는 ROS가 고유한 노드를 만드는 데 사용할 문자열입니다. 두 번째 인수는 ROS 매개변수 사용 방식 때문에 MoveIt에 필요합니다.

마지막으로 ROS를 종료하는 코드가 있습니다.

### 3. MoveGroupInterface를 사용하여 계획 및 실행

"다음 단계가 여기에 들어갑니다" 주석 대신 다음 코드를 추가합니다:

```cpp
// MoveIt MoveGroup 인터페이스 생성
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");

// 목표 자세 설정
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// 목표 자세로의 계획 생성
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// 계획 실행
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
```

#### 3.1. 빌드 및 실행

이전과 마찬가지로 코드를 실행하기 전에 빌드해야 합니다.

`ws_moveit2` 디렉토리에서 다음 명령을 실행합니다:

```bash
colcon build --mixin debug
```

빌드가 성공하면 이전 튜토리얼에서 사용한 데모 실행 파일을 재사용하여 RViz와 MoveGroup 노드를 시작해야 합니다. 별도의 터미널에서 작업 공간을 설정하고 다음을 실행합니다:

```bash
ros2 launch moveit2_tutorials demo.launch.py
```

그런 다음 `Displays` 창의 `MotionPlanning/Planning Request`에서 `Query Goal State` 체크박스를 선택 해제합니다.

![RViz 1](./figs/rviz_1.png)

세 번째 터미널에서 작업 공간을 설정하고 프로그램을 실행합니다.

```bash
ros2 run hello_moveit hello_moveit
```

그러면 RViz의 로봇이 움직이면서 다음과 같은 자세로 끝나야 합니다:

![RViz 2](./figs/rviz_2.png)

데모 실행 파일을 먼저 실행하지 않고 `hello_moveit` 노드를 실행하면 10초 동안 대기한 후 다음과 같은 오류를 출력하고 종료합니다.

```bash
[ERROR] [1644181704.350825487] [hello_moveit]: Could not find parameter robot_description and did not receive robot_description via std_msgs::msg::String subscription within 10.000000 seconds.
```

이는 `demo.launch.py` 실행 파일이 로봇 설명을 제공하는 `MoveGroup` 노드를 시작하기 때문입니다. `MoveGroupInterface`가 생성되면 로봇 설명이 포함된 토픽을 게시하는 노드를 찾습니다. 10초 내에 해당 노드를 찾지 못하면 이 오류를 출력하고 프로그램을 종료합니다.

#### 3.2. 코드 살펴보기

먼저 MoveGroupInterface를 생성합니다. 이 객체는 move_group과 상호 작용하여 궤적을 계획하고 실행할 수 있도록 합니다. 이 프로그램에서 생성하는 유일한 변경 가능한 객체라는 점에 유의하세요. 여기서 생성하는 `MoveGroupInterface` 객체에 대한 또 다른 인터페이스인 `"panda_arm"`에도 주목해야 합니다. 이는 이 `MoveGroupInterface`로 작업할 로봇 설명에 정의된 관절 그룹입니다.

```cpp
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");
```

그런 다음 목표 자세를 설정하고 계획을 수립합니다. 목표 자세만 설정(`setPoseTarget`을 통해)된다는 점에 유의하세요. 시작 자세는 암시적으로 joint state publisher에서 게시한 위치이며, `MoveGroupInterface::setStartState*` 함수 집합을 사용하여 변경할 수 있습니다(이 튜토리얼에서는 다루지 않음).

이 다음 섹션에서 주목할 또 다른 사항은 `target_pose` 메시지 유형 및 계획 구성에 람다를 사용한다는 점입니다. 이는 선언적 스타일로 작성할 수 있는 현대적인 C++ 코드베이스에서 찾을 수 있는 패턴입니다. 이 패턴에 대한 자세한 내용은 이 튜토리얼 끝에 있는 링크를 참조하세요.

```cpp
// 목표 자세 설정
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// 목표 자세로의 계획 생성
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();
```

마지막으로 계획이 성공적이면 계획을 실행하고, 그렇지 않으면 오류를 기록합니다:

```cpp
// 계획 실행
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
```

## 요약

- ROS 2 패키지를 생성하고 MoveIt을 사용하여 첫 번째 프로그램을 작성했습니다.
- MoveGroupInterface를 사용하여 이동을 계획하고 실행하는 방법을 배웠습니다.

## 추가 읽을 거리

- 객체를 상수로 초기화할 수 있도록 람다를 사용했습니다. 이를 IIFE라고 하는 기술입니다. [C++ Stories에서 이 패턴에 대해 자세히 읽어보세요](https://www.cppstories.com/2016/11/iife-for-complex-initialization/).
- 또한 가능한 모든 것을 const로 선언했습니다. [const의 유용성에 대해 여기에서 자세히 읽어보세요](https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/).

## 다음 단계

다음 강의에서는 여기에서 만든 프로그램을 확장하여 MoveIt이 수행하는 작업을 쉽게 이해할 수 있는 시각적 마커를 생성합니다.
