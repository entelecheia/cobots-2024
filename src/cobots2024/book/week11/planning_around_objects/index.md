# 물체 주변 계획하기

이 강의에서는 계획 장면에 물체를 삽입하고 그 주변에서 계획하는 방법을 소개합니다.

## 단계

### 1. Planning Scene Interface 포함 추가

소스 파일 상단의 include 목록에 다음을 추가하세요:

```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>
```

### 2. 목표 자세 변경

먼저 목표 자세를 다음과 같이 수정하여 로봇이 다른 위치로 계획하도록 합니다:

```cpp
// 목표 자세 설정
auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = 0.4;  // <---- 이 값이 변경되었습니다
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);
```

### 3. 충돌 물체 생성

다음 코드 블록에서는 충돌 물체를 생성합니다. 먼저 주목할 점은 이 물체가 로봇의 프레임에 배치된다는 것입니다. 장면에서 장애물의 위치를 보고하는 인식 시스템이 있다면 이런 종류의 메시지를 구축할 것입니다. 이는 단순한 예제이므로 수동으로 생성하고 있습니다. 이 코드 블록의 끝 부분에서 주목할 점은 이 메시지의 작업을 `ADD`로 설정한다는 것입니다. 이렇게 하면 물체가 충돌 장면에 추가됩니다. 이전 단계에서 목표 자세를 설정한 코드와 계획을 생성하는 코드 사이에 이 코드 블록을 배치하세요.

```cpp
// 로봇이 회피할 충돌 물체 생성
auto const collision_object = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // 미터 단위로 상자의 크기 정의
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // 상자의 자세 정의 (frame_id 기준)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();
```

### 4. 계획 장면에 물체 추가

마지막으로 이 물체를 충돌 장면에 추가해야 합니다. 이를 위해 `PlanningSceneInterface` 객체를 사용합니다. 이 객체는 ROS 인터페이스를 사용하여 계획 장면의 변경 사항을 `MoveGroup`에 전달합니다. 이 코드 블록은 충돌 물체를 생성하는 코드 블록 바로 다음에 위치해야 합니다.

```cpp
// 충돌 물체를 장면에 추가
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObject(collision_object);
```

### 5. 프로그램 실행 및 변경 사항 관찰

이전 강의에서 했던 것처럼 `demo.launch.py` 스크립트를 사용하여 RViz를 시작하고 프로그램을 실행하세요. Docker 컨테이너 중 하나를 사용하는 경우 RvizVisualToolsGui 패널이 이미 추가된 다른 RViz 설정을 다음과 같이 지정할 수 있습니다:

```bash
ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz
```

![물체 주변 계획](./figs/planning_around_object.png)

## 요약

- MoveIt으로 작성한 프로그램을 확장하여 장면의 물체 주변에서 계획을 수립했습니다.
- 전체 hello_moveit.cpp 소스 코드 사본은 다음 폴더에 있습니다: [book/week11/planning_around_objects/hello_moveit.cpp](./hello_moveit.cpp)

## 다음 단계

다음 강의에서는 더 어려운 모션 계획을 해결하도록 설계된 상위 계층 도구를 소개합니다.
