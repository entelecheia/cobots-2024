# RViz에서 시각화하기

이 튜토리얼에서는 RViz에서 시각화를 렌더링하여 MoveIt 애플리케이션이 수행하는 작업을 더 쉽게 이해할 수 있는 도구를 소개합니다.

## 단계

### 1. moveit_visual_tools 의존성 추가

`hello_moveit` 프로젝트의 `package.xml` 파일에서 다른 `<depend>` 문 뒤에 다음 줄을 추가하세요:

```xml
<depend>moveit_visual_tools</depend>
```

그런 다음 `CMakeLists.txt` 파일의 `find_package` 문 섹션에 다음 줄을 추가하세요:

```cmake
find_package(moveit_visual_tools REQUIRED)
```

파일의 아래쪽에서 `ament_target_dependencies` 매크로 호출을 확장하여 새로운 의존성을 포함하도록 수정하세요:

```cmake
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
```

의존성을 올바르게 추가했는지 확인하려면 `hello_moveit.cpp` 소스 파일에 필요한 include를 추가하세요:

```cpp
#include <moveit_visual_tools/moveit_visual_tools.h>
```

이 모든 작업이 제대로 되었는지 테스트하려면 작업 공간 디렉토리에서 터미널을 열고(opt에서 ROS 설치를 설정해야 함) colcon으로 빌드하세요:

```bash
cd ~/ws_moveit2
colcon build --mixin debug
```

### 2. ROS 실행기 생성 및 스레드에서 노드 실행

MoveItVisualTools를 초기화하기 전에 ROS 노드에서 실행되는 실행기가 필요합니다. 이는 MoveItVisualTools가 ROS 서비스 및 토픽과 상호 작용하는 방식 때문에 필요합니다.

```cpp
#include <thread>  // <---- 맨 위의 include 집합에 이 줄을 추가하세요

  ...

  // ROS 로거 생성
  auto const logger = rclcpp::get_logger("hello_moveit");

  // SingleThreadedExecutor를 생성하여 MoveItVisualTools가 ROS와 상호 작용하도록 합니다
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // MoveIt MoveGroup 인터페이스 생성
  ...

  // ROS 종료
  rclcpp::shutdown();  // <--- 이렇게 하면 스레드의 spin 함수가 반환됩니다
  spinner.join();  // <--- 종료하기 전에 스레드를 join합니다
  return 0;
}
```

이러한 각 변경 사항 후에는 작업 공간을 다시 빌드하여 구문 오류가 없는지 확인해야 합니다.

### 3. MoveItVisualTools 생성 및 초기화

다음으로 MoveGroupInterface 생성 후 MoveItVisualTools를 생성하고 초기화합니다.

```cpp
// MoveIt MoveGroup 인터페이스 생성
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");

// MoveItVisualTools 생성 및 초기화
auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
moveit_visual_tools.deleteAllMarkers();
moveit_visual_tools.loadRemoteControl();
```

생성자에는 ROS 노드, 로봇의 기본 링크, 사용할 마커 토픽(이에 대해서는 나중에 더 설명) 및 로봇 모델(move_group_interface에서 가져옴)을 전달합니다. 다음으로 모든 마커를 삭제하는 호출을 수행합니다. 이렇게 하면 이전 실행에서 남겨진 RViz의 렌더링된 상태가 모두 지워집니다. 마지막으로 원격 제어를 로드합니다. 원격 제어는 RViz에서 프로그램과 상호 작용할 수 있는 버튼을 제공하는 매우 간단한 플러그인입니다.

### 4. 시각화를 위한 클로저 작성

생성 및 초기화 후에는 프로그램에서 나중에 사용하여 RViz에서 시각화를 렌더링하는 데 도움이 되는 클로저(현재 범위의 변수에 액세스할 수 있는 함수 객체)를 생성합니다.

```cpp
// 시각화를 위한 클로저 생성
auto const draw_title = [&moveit_visual_tools](auto text) {
  auto const text_pose = [] {
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().z() = 1.0;
    return msg;
  }();
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
};
auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
};
auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
     jmg = move_group_interface.getRobotModel()->getJointModelGroup(
         "panda_arm")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };
```

각 세 개의 클로저는 참조에 의해 `moveit_visual_tools`를 캡처하고 마지막 클로저는 계획 중인 관절 모델 그룹 객체에 대한 포인터를 캡처합니다. 이들은 각각 `moveit_visual_tools`의 함수를 호출하여 RViz의 무언가를 변경합니다. 첫 번째 `draw_title`은 로봇 베이스 위 1미터 위치에 텍스트를 추가합니다. 이는 프로그램의 상태를 높은 수준에서 보여주는 유용한 방법입니다. 두 번째는 `prompt`라는 함수를 호출합니다. 이 함수는 사용자가 RViz에서 `next` 버튼을 누를 때까지 프로그램을 차단합니다. 이는 디버깅할 때 프로그램을 단계별로 실행하는 데 도움이 됩니다. 마지막 클로저는 계획한 궤적의 도구 경로를 그립니다. 이는 도구의 관점에서 계획된 궤적을 이해하는 데 종종 도움이 됩니다.

이렇게 람다를 만드는 이유가 궁금할 수 있는데, 그 이유는 단순히 이후에 나오는 코드를 더 쉽게 읽고 이해할 수 있게 하기 위함입니다. 소프트웨어를 작성할 때는 종종 기능을 쉽게 재사용하고 독립적으로 테스트할 수 있는 명명된 함수로 분할하는 것이 도움이 됩니다. 다음 섹션에서 이렇게 생성한 함수를 어떻게 사용하는지 볼 수 있습니다.

### 5. 프로그램 단계 시각화

이제 프로그램 중간에 있는 코드를 확장합니다. 계획 및 실행 코드를 업데이트하여 이러한 새로운 기능을 포함하도록 수정하세요:

```cpp
// 목표 자세 설정
auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// 목표 자세로의 계획 생성
prompt("RvizVisualToolsGui 창에서 'Next'를 눌러 계획하세요");
draw_title("Planning");
moveit_visual_tools.trigger();
auto const [success, plan] = [&move_group_interface] {
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// 계획 실행
if (success) {
  draw_trajectory_tool_path(plan.trajectory_);
  moveit_visual_tools.trigger();
  prompt("RvizVisualToolsGui 창에서 'Next'를 눌러 실행하세요");
  draw_title("Executing");
  moveit_visual_tools.trigger();
  move_group_interface.execute(plan);
} else {
  draw_title("Planning Failed!");
  moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planing failed!");
}
```

RViz에서 렌더링되는 내용을 변경하는 각 호출 후에 `moveit_visual_tools`에서 `trigger` 메서드를 호출해야 한다는 점을 빠르게 알 수 있습니다. 이는 RViz로 전송되는 메시지가 마커 토픽의 대역폭을 줄이기 위해 `trigger`를 호출할 때까지 일괄 처리되어 전송되기 때문입니다.

마지막으로 프로젝트를 다시 빌드하여 모든 코드 추가 사항이 올바른지 확인하세요.

```bash
cd ~/ws_moveit2
source /opt/ros/{DISTRO}/setup.bash
colcon build --mixin debug
```

### 6. RViz에서 시각화 활성화

새 터미널을 열고 작업 공간을 설정한 다음 RViz를 여는 데모 실행 파일을 시작하세요.

```bash
cd ~/ws_moveit2
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```

"Displays" 탭에서 "MotionPlanning"의 선택을 해제하여 숨깁니다. 다음 부분에서는 "MotionPlanning" 플러그인을 사용하지 않을 것입니다.

![MotionPlanning 선택 해제](./figs/uncheck_motion_planning.png)

![MotionPlanning 선택 해제됨](./figs/unchecked_motion_planning.png)

프로그램에 추가한 프롬프트와 상호 작용하는 버튼을 추가하려면 "Panels/Add New Panel" 메뉴에서 대화상자를 엽니다:

![패널 메뉴](./figs/panel_menu.png)

그런 다음 `RvizVisualToolsGui`를 선택하고 OK를 클릭합니다. 이렇게 하면 왼쪽 하단에 나중에 사용할 `Next` 버튼이 있는 새 패널이 생성됩니다.

![RvizVisualToolsGui 추가](./figs/add_rviz_tools_gui.png)

![Next 버튼](./figs/next_button.png)

마지막으로 추가한 시각화를 렌더링하기 위해 `Marker Array`를 추가해야 합니다. "Displays" 패널에서 "Add" 버튼을 클릭하세요.

![Add 버튼](./figs/add_button.png)

`Marker Array`를 선택하고 `OK`를 클릭하세요.

![Marker Array](./figs/marker_array.png)

Displays 패널의 항목 하단으로 스크롤하고 새 Marker Array가 사용하는 토픽을 `/rviz_visual_tools`로 편집하세요.

![Marker Array 토픽](./figs/marker_array_topic.png)

이제 시각화가 포함된 새 프로그램을 실행할 준비가 되었습니다.

### 7. 프로그램 실행

새 터미널에서 작업 공간으로 이동하여 작업 공간을 설정하고 `hello_moveit`를 실행하세요:

```bash
cd ~/ws_moveit2
source install/setup.bash
ros2 run hello_moveit hello_moveit
```

프로그램이 다음과 같은 로그와 함께 중지되었음을 알 수 있습니다:

```
[INFO] [1652822889.492940200] [hello_moveit.remote_control]: Waiting to continue: Press 'Next' in the RvizVisualToolsGui window to plan
```

RViz에서 `Next` 버튼을 클릭하고 애플리케이션이 진행되는 것을 확인하세요.

![계획 중](./figs/planning.png)

Next 버튼을 클릭한 후 애플리케이션이 계획을 수립하고, 로봇 위에 제목을 추가하며, 도구 경로를 나타내는 선을 그리는 것을 볼 수 있습니다. 계속하려면 `Next`를 다시 눌러 로봇이 계획을 실행하는 것을 확인하세요.

![실행 중](./figs/executing.png)

## 요약

MoveIt으로 작성한 프로그램을 확장하여 RViz의 GUI와 상호 작용함으로써 버튼으로 프로그램을 단계별로 실행하고, 로봇 위에 텍스트를 렌더링하며, 계획한 도구 경로를 표시할 수 있게 되었습니다.

## 추가 읽을 거리

- MoveItVisualTools에는 로봇 모션을 시각화하기 위한 많은 유용한 기능이 있습니다. [여기에서 자세히 읽어볼 수 있습니다](https://github.com/ros-planning/moveit_visual_tools/tree/ros2).

## 다음 단계

다음 튜토리얼 `물체 주변 계획`에서는 여기에서 만든 프로그램을 확장하여 충돌 환경을 추가하고 이러한 변경 사항으로 로봇 계획을 확인할 것입니다.
