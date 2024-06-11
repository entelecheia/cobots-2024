# MoveIt Task Constructor를 사용한 집어서 놓기

이 강의에서는 [MoveIt Task Constructor](https://github.com/ros-planning/moveit_task_constructor/tree/ros2/)를 사용하여 집어서 놓기 작업을 계획하는 패키지를 만드는 과정을 안내합니다. MoveIt Task Constructor는 여러 개의 하위 작업(스테이지라고 함)으로 구성된 작업을 계획하는 방법을 제공합니다.

<video width="700px" controls="true" autoplay="true" loop="true">
    <source src="../../assets/video/mtc-demo.webm" type="video/webm">
</video>

## 시작하기

### MoveIt Task Constructor 다운로드

Colcon 작업 공간으로 이동하여 MoveIt Task Constructor 소스를 가져옵니다:

```bash
cd ~/ws_moveit2/src
git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2
```

### 새 패키지 만들기

다음 명령을 사용하여 새 패키지를 만듭니다:

```bash
ros2 pkg create --build-type ament_cmake --node-name mtc_tutorial mtc_tutorial
```

이렇게 하면 `mtc_tutorial`이라는 새 폴더가 생성되고 `src/mtc_node`에 Hello World 예제가 포함됩니다. 다음으로 `package.xml`에 의존성을 추가합니다. 다음과 유사해야 합니다:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
<name>mtc_tutorial</name>
<version>0.0.0</version>
<description>TODO: Package description</description>
<maintainer email="youremail@domain.com">user</maintainer>
<license>TODO: License declaration</license>

<buildtool_depend>ament_cmake</buildtool_depend>

<depend>moveit_task_constructor_core</depend>
<depend>rclcpp</depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
    <build_type>ament_cmake</build_type>
</export>
</package>
```

또한 `CMakeLists.txt`에도 의존성을 추가합니다. 파일은 다음과 유사해야 합니다:

```cmake
cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mtc_tutorial src/mtc_tutorial.cpp)
ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
target_include_directories(mtc_tutorial PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mtc_tutorial
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## MoveIt Task Constructor를 사용한 프로젝트 설정

이 섹션에서는 MoveIt Task Constructor를 사용하여 최소한의 작업을 구축하는 데 필요한 코드를 살펴봅니다.

### 코드

선택한 편집기에서 `mtc_tutorial.cpp`를 열고 다음 코드를 붙여넣습니다.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
```

### 코드 분석

코드 상단에는 이 패키지에서 사용하는 ROS 및 MoveIt 라이브러리가 포함되어 있습니다.

- `rclcpp/rclcpp.hpp`에는 핵심 ROS 2 기능이 포함되어 있습니다.
- `moveit/planning_scene/planning_scene.h` 및 `moveit/planning_scene_interface/planning_scene_interface.h`에는 로봇 모델 및 충돌 객체와 상호 작용하기 위한 기능이 포함되어 있습니다.
- `moveit/task_constructor/task.h`, `moveit/task_constructor/solvers.h` 및 `moveit/task_constructor/stages.h`에는 예제에서 사용되는 MoveIt Task Constructor의 다양한 구성 요소가 포함되어 있습니다.
- `tf2_geometry_msgs/tf2_geometry_msgs.hpp` 및 `tf2_eigen/tf2_eigen.hpp`는 이 초기 예제에서는 사용되지 않지만, MoveIt Task Constructor 작업에 더 많은 스테이지를 추가할 때 자세 생성에 사용될 것입니다.

다음 줄에서는 새 노드에 대한 로거를 가져옵니다. 또한 편의를 위해 `moveit::task_constructor`에 대한 네임스페이스 별칭을 만듭니다.

```cpp
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
```

MoveIt Task Constructor의 주요 기능을 포함할 클래스를 정의하여 시작합니다. 또한 MoveIt Task Constructor 작업 객체를 클래스의 멤버 변수로 선언합니다. 이는 주어진 애플리케이션에 반드시 필요한 것은 아니지만, 나중에 시각화를 위해 작업을 저장하는 데 도움이 됩니다. 아래에서 각 함수를 개별적으로 살펴보겠습니다.

```cpp
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};
```

이 줄은 나중에 실행기에 사용될 노드 기본 인터페이스를 가져오는 getter 함수를 정의합니다.

```cpp
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}
```

다음 줄은 지정된 옵션으로 노드를 초기화합니다.

```cpp
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}
```

이 클래스 메서드는 예제에 사용되는 계획 장면을 설정하는 데 사용됩니다. `object.primitives[0].dimensions`로 지정된 치수와 `pose.position.z` 및 `pose.position.x`로 지정된 위치를 사용하여 실린더를 만듭니다. 이 숫자를 변경하여 실린더의 크기를 조정하고 이동할 수 있습니다. 실린더를 로봇의 도달 범위 밖으로 이동하면 계획이 실패합니다.

```cpp
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}
```

이 함수는 MoveIt Task Constructor 작업 객체와 상호 작용합니다. 먼저 작업을 생성하는데, 여기에는 일부 속성 설정과 스테이지 추가가 포함됩니다. 이에 대해서는 `createTask` 함수 정의에서 더 자세히 다룰 것입니다. 다음으로 `task.init()`는 작업을 초기화하고 `task.plan(5)`는 5개의 성공적인 계획이 발견될 때까지 계획을 생성합니다. 다음 줄은 RViz에서 시각화할 솔루션을 게시합니다. 시각화가 필요하지 않은 경우 이 줄을 제거할 수 있습니다. 마지막으로 `task.execute()`는 계획을 실행합니다. 실행은 RViz 플러그인과의 액션 서버 인터페이스를 통해 수행됩니다.

```cpp
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}
```

위에서 언급했듯이 이 함수는 MoveIt Task Constructor 객체를 생성하고 초기 속성을 설정합니다. 이 경우 작업 이름을 "demo_task"로 설정하고 로봇 모델을 로드하며 일부 유용한 프레임의 이름을 정의하고 `task.setProperty(property_name, value)`를 사용하여 해당 프레임 이름을 작업의 속성으로 설정합니다. 다음 몇 개의 코드 블록에서는 이 함수 본문을 채울 것입니다.

```cpp
mtc::Task MTCTaskNode::createTask()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
```

이제 노드에 예제 스테이지를 추가합니다. 첫 번째 줄은 `current_state_ptr`을 `nullptr`로 설정합니다. 이렇게 하면 특정 시나리오에서 스테이지 정보를 재사용할 수 있도록 스테이지에 대한 포인터가 생성됩니다. 이 줄은 현재 사용되지 않지만 나중에 작업에 더 많은 스테이지를 추가할 때 사용됩니다. 다음으로 `current_state` 스테이지(생성기 스테이지)를 만들고 작업에 추가합니다. 이렇게 하면 로봇이 현재 상태에서 시작됩니다. 이제 `CurrentState` 스테이지를 생성했으므로 나중에 사용할 수 있도록 `current_state_ptr`에 포인터를 저장합니다.

```cpp
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
```

로봇 모션을 계획하려면 솔버를 지정해야 합니다. MoveIt Task Constructor에는 솔버에 대한 세 가지 옵션이 있습니다:

- `PipelinePlanner`는 일반적으로 OMPL로 기본 설정되는 MoveIt의 계획 파이프라인을 사용합니다.
- `CartesianPath`는 카테시안 공간에서 엔드 이펙터를 직선으로 이동하는 데 사용됩니다.
- `JointInterpolation`은 시작 관절 상태와 목표 관절 상태 사이를 보간하는 간단한 플래너입니다. 일반적으로 빠르게 계산되지만 복잡한 모션을 지원하지 않으므로 단순한 모션에 사용됩니다.

또한 카테시안 플래너에 특화된 일부 속성을 설정합니다.

```cpp
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
```

이제 플래너를 추가했으므로 로봇을 이동시킬 스테이지를 추가할 수 있습니다. 다음 줄은 `MoveTo` 스테이지(전파기 스테이지)를 사용합니다. 핸드를 여는 것은 비교적 단순한 동작이므로 관절 보간 플래너를 사용할 수 있습니다. 이 스테이지는 "open hand" 자세로 이동을 계획하는데, 이는 판다 로봇의 :moveit_resources_codedir:`SRDF<panda_moveit_config/config/panda.srdf>`에 정의된 명명된 자세입니다. 작업을 반환하고 createTask() 함수를 마칩니다.

```cpp
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  return task;
}
```

마지막으로 `main` 함수가 있습니다. 다음 줄은 위에서 정의한 클래스를 사용하여 노드를 생성하고 클래스 메서드를 호출하여 기본 MTC 작업을 설정하고 실행합니다. 이 예제에서는 작업 실행이 완료된 후 RViz에서 솔루션을 검사하기 위해 실행기를 취소하지 않습니다.

```cpp
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
```

## 데모 실행

### 실행 파일

`move_group`, `ros2_control`, `static_tf`, `robot_state_publisher` 및 `rviz`를 실행하려면 실행 파일이 필요합니다. [여기](./launch/mtc_demo.launch.py)는 강의 패키지에서 사용하는 실행 파일입니다. 이 파일을 패키지의 실행 디렉토리에 넣으세요.

MoveIt Task Constructor 노드를 실행하려면 적절한 매개변수로 `mtc_tutorial` 실행 파일을 시작하는 두 번째 실행 파일이 필요합니다. URDF, SRDF 및 OMPL 매개변수를 로드하거나 MoveIt Configs Utils를 사용하여 로드합니다. 실행 파일은 다음과 유사해야 합니다:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
```

이 파일을 `pick_place_demo.launch.py`로 패키지의 실행 디렉토리에 저장하세요. 실행 파일이 제대로 설치되도록 `CMakeLists.txt`에 다음 줄을 추가해야 합니다:

    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

이제 Colcon 작업 공간을 빌드하고 설정하세요:

```bash
cd ~/ws_moveit2
colcon build --mixin release
source ~/ws_moveit2/install/setup.bash
```

첫 번째 실행 파일을 실행하여 시작하세요. 강의에서 제공하는 파일을 사용하려면 다음을 실행하세요:

```bash
ros2 launch moveit2_tutorials mtc_demo.launch.py
```

RViz가 로드되어야 합니다. 자신만의 실행 파일을 사용하는 경우 무언가를 보려면 먼저 RViz를 구성해야 합니다. 강의 패키지의 실행 파일을 사용하는 경우 이미 구성되어 있습니다.

### RViz 구성

RViz에서 로봇과 MoveIt Task Constructor 솔루션을 보려면 RViz 구성을 변경해야 합니다. 먼저 RViz를 시작하세요. 다음 단계에서는 MoveIt Task Constructor 솔루션 시각화를 위해 RViz를 설정하는 방법을 다룹니다.

1. **MotionPlanning** 디스플레이가 활성 상태인 경우 선택을 해제하여 숨깁니다.
2. **Global Options**에서 **Fixed Frame**을 `map`에서 `panda_link0`로 변경합니다(아직 변경하지 않은 경우).
3. 창 왼쪽 하단의 **Add** 버튼을 클릭합니다.
4. `moveit_task_constructor_visualization`에서 **Motion Planning Tasks**를 선택하고 OK를 클릭합니다. 왼쪽 하단에 **Motion Planning Tasks** 디스플레이가 나타나야 합니다.
5. **Displays**의 **Motion Planning Tasks**에서 **Task Solution Topic**을 `/solution`으로 변경합니다.

메인 뷰에 판다 암이 보이고 왼쪽 하단에 Motion Planning Tasks 디스플레이가 열려 있지만 아무것도 없어야 합니다. `mtc_tutorial` 노드를 실행하면 이 패널에 MTC 작업이 표시됩니다.

### 데모 실행

`mtc_tutorial` 노드를 다음과 같이 실행하세요:

```bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

로봇 앞에 녹색 실린더가 있는 상태에서 핸드를 여는 단일 스테이지로 암이 작업을 실행하는 모습을 볼 수 있습니다. 다음과 유사해야 합니다:

![첫 번째 스테이지](./figs/first_stages.png)

자신만의 패키지를 만들지 않았지만 어떻게 보이는지 확인하고 싶다면 강의에서 이 파일을 실행할 수 있습니다:

```bash
ros2 launch moveit2_tutorials mtc_demo_minimal.launch.py
```

## 스테이지 추가

지금까지 간단한 작업을 생성하고 실행하는 과정을 살펴보았습니다. 실행은 되지만 많은 작업을 수행하지는 않습니다. 이제 작업에 집어서 놓기 스테이지를 추가하기 시작할 것입니다. 아래 이미지는 작업에서 사용할 스테이지의 개요를 보여줍니다.

![스테이지](./figs/stages.png)

기존 핸드 열기 스테이지 다음부터 스테이지 추가를 시작할 것입니다:

```cpp
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
  // 여기에 더 많은 스테이지를 정의하는 코드 줄을 추가하세요
```

### 집기 스테이지

물체를 집어들 수 있는 위치로 암을 이동해야 합니다. 이는 이름에서 알 수 있듯이 연결 역할을 하는 `Connect` 스테이지를 사용하여 수행됩니다. 즉, 이전 스테이지와 이후 스테이지의 결과 사이를 연결하려고 시도합니다. 이 스테이지는 이름 `move_to_pick`과 계획 그룹과 플래너를 지정하는 `GroupPlannerVector`로 초기화됩니다. 그런 다음 스테이지에 대한 시간 제한을 설정하고 스테이지의 속성을 설정한 후 작업에 추가합니다.

```cpp
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
```

다음으로 MoveIt Task Constructor 스테이지 객체에 대한 포인터를 생성하고 현재는 `nullptr`로 설정합니다. 나중에 이를 사용하여 스테이지를 저장할 것입니다.

```cpp
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
```

다음 코드 블록은 `SerialContainer`를 생성합니다. 이는 작업에 추가할 수 있는 컨테이너로 여러 하위 스테이지를 포함할 수 있습니다. 이 경우 집기 동작과 관련된 스테이지를 포함하는 직렬 컨테이너를 생성합니다. 작업에 스테이지를 추가하는 대신 관련 스테이지를 직렬 컨테이너에 추가할 것입니다. `exposeTo`를 사용하여 부모 작업의 작업 속성을 새 직렬 컨테이너에 선언하고 configureInitFrom()을 사용하여 초기화합니다. 이를 통해 포함된 스테이지에서 이러한 속성에 액세스할 수 있습니다.

```cpp
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
```

그런 다음 물체에 접근하기 위한 스테이지를 생성합니다. 이 스테이지는 `MoveRelative` 스테이지로, 현재 위치에서 상대적인 이동을 지정할 수 있습니다. `MoveRelative`는 전파기 스테이지입니다. 즉, 이웃 스테이지에서 솔루션을 받아 다음 또는 이전 스테이지로 전파합니다. `cartesian_planner`를 사용하면 엔드 이펙터를 직선으로 이동하는 솔루션을 찾을 수 있습니다. 속성을 설정하고 이동할 최소 및 최대 거리를 설정합니다. 이제 이동하려는 방향을 나타내는 `Vector3Stamped` 메시지를 생성합니다. 이 경우 핸드 프레임에서 Z 방향입니다. 마지막으로 이 스테이지를 직렬 컨테이너에 추가합니다.

```cpp
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
```

이제 그립 자세를 생성하는 스테이지를 만듭니다. 이는 생성기 스테이지이므로 이전 및 이후 스테이지와 상관없이 결과를 계산합니다. 첫 번째 스테이지인 `CurrentState`도 생성기 스테이지입니다. 첫 번째 스테이지와 이 스테이지를 연결하려면 이미 위에서 생성한 연결 스테이지를 사용해야 합니다. 이 코드는 스테이지 속성을 설정하고 그립 전의 자세, 각도 델타 및 모니터링된 스테이지를 설정합니다. 각도 델타는 `GenerateGraspPose` 스테이지의 속성으로 생성할 자세의 수를 결정하는 데 사용됩니다. 솔루션을 생성할 때 MoveIt Task Constructor는 여러 가지 다양한 방향에서 물체를 잡으려고 시도하며, 방향 간의 차이는 각도 델타로 지정됩니다. 델타가 작을수록 그립 방향이 더 가깝게 됩니다. 현재 스테이지를 정의할 때 `current_state_ptr`을 설정하는데, 이는 이제 역기구학 솔버에 물체 자세 및 모양에 대한 정보를 전달하는 데 사용됩니다. 이 스테이지는 이전처럼 직렬 컨테이너에 직접 추가되지 않습니다. 생성된 자세에 대해 역기구학을 수행해야 하기 때문입니다.

```cpp
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
```

위에서 생성된 자세에 대해 역기구학을 계산하기 전에 먼저 프레임을 정의해야 합니다. 이는 `geometry_msgs`의 `PoseStamped` 메시지를 사용하거나 이 경우와 같이 Eigen 변환 행렬과 관련 링크의 이름을 사용하여 변환을 정의할 수 있습니다. 여기서는 변환 행렬을 정의합니다.

```cpp
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;
```

이제 `ComputeIK` 스테이지를 생성하고 이름 `generate pose IK`와 위에서 정의한 `generate grasp pose` 스테이지를 전달합니다. 일부 로봇은 주어진 자세에 대해 여러 개의 역기구학 솔루션을 가지고 있습니다. 최대 8개까지의 솔루션을 계산하도록 제한을 설정합니다. 또한 최소 솔루션 거리를 설정하는데, 이는 솔루션이 얼마나 달라야 하는지에 대한 임계값입니다. 솔루션의 관절 위치가 이전 솔루션과 너무 유사하면 유효하지 않은 것으로 표시됩니다. 그런 다음 일부 추가 속성을 구성하고 `ComputeIK` 스테이지를 직렬 컨테이너에 추가합니다.

```cpp
      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
```

물체를 집어 올리려면 핸드와 물체 사이의 충돌을 허용해야 합니다. 이는 `ModifyPlanningScene` 스테이지를 사용하여 수행할 수 있습니다. `allowCollisions` 함수를 사용하면 비활성화할 충돌을 지정할 수 있습니다. `allowCollisions`는 이름 컨테이너와 함께 사용할 수 있으므로 `getLinkModelNamesWithCollisionGeometry`를 사용하여 핸드 그룹에서 충돌 지오메트리가 있는 모든 링크의 이름을 가져올 수 있습니다.

```cpp
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
```

충돌이 허용되면 이제 핸드를 닫을 수 있습니다. 이는 위의 `open hand` 스테이지와 유사하게 `MoveTo` 스테이지를 사용하여 수행되지만, SRDF에 정의된 `close` 위치로 이동합니다.

```cpp
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }
```

이제 `ModifyPlanningScene` 스테이지를 다시 사용하여 `attachObject`를 사용해 물체를 핸드에 부착합니다. `current_state_ptr`에서 했던 것과 유사하게 이 스테이지에 대한 포인터를 가져와서 나중에 물체의 놓기 자세를 생성할 때 사용합니다.

```cpp
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
```

다음으로 `approach_object` 스테이지와 유사하게 `MoveRelative` 스테이지를 사용하여 물체를 들어올립니다.

```cpp
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
```

이로써 물체를 집어올리는 데 필요한 모든 스테이지가 완성되었습니다. 이제 직렬 컨테이너(모든 하위 스테이지 포함)를 작업에 추가합니다. 코드를 그대로 빌드하면 로봇이 물체를 집어올리도록 계획하는 모습을 볼 수 있습니다.

```cpp
    task.add(std::move(grasp));
  }
```

### 놓기 스테이지

이제 집기를 정의하는 스테이지가 완료되었으므로 물체를 놓는 스테이지를 정의하는 것으로 넘어갑니다. 곧 물체를 놓을 자세를 생성하기 위해 생성기 스테이지를 사용할 것이므로 두 스테이지를 연결하는 `Connect` 스테이지로 시작합니다.

```cpp
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
```

놓기 스테이지를 위한 직렬 컨테이너도 생성합니다. 이는 집기 직렬 컨테이너와 유사하게 수행됩니다. 다음 스테이지는 작업이 아닌 직렬 컨테이너에 추가됩니다.

```cpp
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
```

다음 스테이지는 물체를 놓을 자세를 생성하고 해당 자세에 대한 역기구학을 계산합니다. 집기 직렬 컨테이너의 `generate grasp pose` 스테이지와 다소 유사합니다. 먼저 자세를 생성하고 작업 속성을 상속하는 스테이지를 만듭니다. `geometry_msgs`의 `PoseStamped` 메시지를 사용하여 물체를 놓을 자세를 지정합니다. 이 경우 `"object"` 프레임에서 `y = 0.5`를 선택합니다. 그런 다음 `setPose`를 사용하여 목표 자세를 스테이지에 전달합니다. 다음으로 `setMonitoredStage`를 사용하고 이전의 `attach_object` 스테이지에 대한 포인터를 전달합니다. 이를 통해 스테이지에서 물체가 어떻게 부착되어 있는지 알 수 있습니다. 그런 다음 `GeneratePlacePose` 스테이지를 전달하여 `ComputeIK` 스테이지를 생성합니다. 나머지는 위의 집기 스테이지와 동일한 논리를 따릅니다.

```cpp
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }
```

이제 물체를 놓을 준비가 되었으므로 `MoveTo` 스테이지와 관절 보간 플래너를 사용하여 핸드를 엽니다.

```cpp
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
```

물체를 더 이상 잡을 필요가 없으므로 물체와의 충돌도 다시 활성화할 수 있습니다. 이는 충돌을 비활성화할 때와 거의 동일한 방식으로 `allowCollisions`를 사용하여 수행되지만, 마지막 인수를 `true` 대신 `false`로 설정합니다.

```cpp
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }
```

이제 `detachObject`를 사용하여 물체를 분리할 수 있습니다.

```cpp
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
```

`approach object` 및 `lift object` 스테이지와 유사하게 `MoveRelative` 스테이지를 사용하여 물체에서 후퇴합니다.

```cpp
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
```

놓기 직렬 컨테이너를 마무리하고 작업에 추가합니다.

```cpp
    task.add(std::move(place));
  }
```

마지막 단계는 홈으로 돌아가는 것입니다. `MoveTo` 스테이지를 사용하고 판다 SRDF에 정의된 자세인 `ready`를 목표 자세로 전달합니다.

```cpp
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
```

이 모든 스테이지는 아래 줄 위에 추가되어야 합니다.

```cpp
  // 이 줄 위에 모든 스테이지가 작업에 추가됩니다

  return task;
}
```

축하합니다! 이제 MoveIt Task Constructor를 사용하여 집어서 놓기 작업을 정의했습니다!

## RViz로 시각화

각 구성 스테이지를 포함한 작업이 Motion Planning Tasks 창에 표시됩니다. 스테이지를 클릭하면 오른쪽에 스테이지에 대한 추가 정보가 표시됩니다. 오른쪽 창에는 다양한 솔루션과 관련 비용이 표시됩니다. 스테이지 유형과 로봇 구성에 따라 하나의 솔루션만 표시될 수 있습니다.

솔루션 비용 중 하나를 클릭하면 해당 스테이지의 계획을 따르는 로봇의 애니메이션이 표시됩니다. 창 오른쪽 상단의 "Exec" 버튼을 클릭하여 모션을 실행합니다.

MoveIt 강의에 포함된 전체 MoveIt Task Constructor 예제를 실행하려면 다음을 실행하세요:

```bash
ros2 launch moveit2_tutorials mtc_demo.launch.py
```

그리고 두 번째 터미널에서 다음을 실행하세요:

```bash
ros2 launch moveit2_tutorials pick_place_demo.launch.py
```

### 터미널에서 디버깅

MTC를 실행할 때 터미널에 다음과 같은 다이어그램이 출력됩니다:

```bash
[demo_node-1]     1  - ←   1 →   -  0 / initial_state
[demo_node-1]     -  0 →   0 →   -  0 / move_to_home
```

이 예제는^ 두 개의 스테이지를 보여줍니다. 첫 번째 스테이지("initial_state")는 `CurrentState` 유형의 스테이지로, PlanningScene을 초기화하고 해당 시점에 존재하는 충돌 객체를 캡처합니다. 이 스테이지에 대한 포인터를 사용하여 로봇의 상태를 검색할 수 있습니다. CurrentState는 `Generator`에서 상속되므로 솔루션을 앞뒤로 전파합니다. 이는 양방향 화살표로 표시됩니다. 첫 번째 `1`은 하나의 솔루션이 이전 스테이지로 성공적으로 역방향 전파되었음을 나타냅니다. 화살표 사이의 두 번째 `1`은 하나의 솔루션이 생성되었음을 나타냅니다. `0`은 다음 스테이지가 실패했기 때문에 솔루션이 다음 스테이지로 성공적으로 전방 전파되지 않았음을 나타냅니다.

두 번째 스테이지("move_to_home")는 `MoveTo` 유형의 스테이지입니다. 전파 방향을 이전 스테이지에서 상속하므로 두 화살표가 모두 앞을 가리킵니다. 왼쪽에서 오른쪽으로 `0`은 다음을 의미합니다:

- 스테이지가 이전 스테이지에서 솔루션을 받지 못함
- 스테이지가 솔루션을 생성하지 않음
- 스테이지가 솔루션을 다음 스테이지로 전방 전파하지 않음

이 경우 "move_to_home"이 실패의 근본 원인임을 알 수 있습니다. 문제는 충돌이 발생한 홈 상태였습니다. 새로운 충돌이 없는 홈 위치를 정의하면 문제가 해결되었습니다.

### 다양한 힌트

작업에서 개별 스테이지에 대한 정보를 검색할 수 있습니다. 예를 들어 여기서는 스테이지의 고유 ID를 검색합니다:

    uint32_t const unique_stage_id = task_.stages()->findChild(stage_name)->introspectionId();

CurrentState 유형의 스테이지는 단순히 로봇의 현재 상태를 검색하는 것이 아닙니다. 또한 PlanningScene 객체를 초기화하여 해당 시점에 존재하는 충돌 객체를 캡처합니다.

MTC 스테이지는 순방향과 역방향으로 전파될 수 있습니다. RViz GUI에서 화살표를 보면 스테이지가 전파되는 방향을 쉽게 확인할 수 있습니다. 역방향으로 전파할 때는 많은 작업의 논리가 반대로 됩니다. 예를 들어 `ModifyPlanningScene` 스테이지에서 물체와의 충돌을 허용하려면 `allowCollisions(true)` 대신 `allowCollisions(false)`를 호출해야 합니다. 이에 대한 논의는 [여기](https://github.com/ros-planning/moveit_task_constructor/issues/349)에서 읽을 수 있습니다.

## 요약

MoveIt Task Constructor를 사용하여 복잡한 집어서 놓기 작업을 구성했습니다. 이 과정에서 다음과 같은 내용을 다루었습니다:

- MoveIt Task Constructor의 개념과 스테이지 유형
- 생성기, 전파기, 연결기 스테이지를 사용하여 작업 정의
- 직렬 컨테이너를 사용하여 하위 작업 구성
- RViz에서 작업 시각화 및 실행
- 터미널에서 MTC 디버깅

이 강의을 통해 MoveIt Task Constructor를 사용하여 복잡한 작업을 분해하고 구성하는 방법을 배웠습니다. 이제 이 지식을 활용하여 자신만의 로봇 작업을 만들 수 있습니다.
