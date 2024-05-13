# RViz에서 MoveIt 빠르게 시작하기

이 튜토리얼에서는 RViz와 MoveIt 디스플레이 플러그인을 사용하여 MoveIt에서 모션 플랜을 생성하는 방법을 배웁니다. RViz는 ROS의 주요 시각화 도구이며 로보틱스 디버깅에 매우 유용합니다. MoveIt 디스플레이 플러그인을 사용하면 가상 환경(플래닝 장면)을 설정하고, 로봇의 시작 및 목표 상태를 대화형으로 생성하며, 다양한 모션 플래너를 테스트하고, 출력을 시각화할 수 있습니다. 시작해 봅시다!

![RViz 플러그인 헤더](./figs/rviz_plugin_head.png)

## 시작하기

아직 수행하지 않았다면 먼저 MoveIt 설치 및 환경 설정 과정을 완료해야 합니다. 2022년 9월 26일 기준으로 Cyclone DDS를 활성화해야 합니다.

## 단계 1: 데모 실행 및 플러그인 구성

- 다음 명령을 사용하여 데모를 실행합니다:

  ```bash
  ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz
  ```

- 처음 실행하는 경우 RViz에 빈 월드가 표시되고 Motion Planning 플러그인을 추가해야 합니다:

  - RViz에 빈 월드가 표시됩니다:

    ![빈 RViz 월드](./figs/rviz_empty.png)

  - RViz Displays 탭에서 _Add_ 버튼을 클릭합니다:

    ![Add 버튼 클릭](./figs/rviz_click_add.png)

  - moveit_ros_visualization 폴더에서 "MotionPlanning"을 DisplayType으로 선택하고 "Ok"를 클릭합니다.

    ![Motion Planning 플러그인 추가](./figs/rviz_plugin_motion_planning_add.png)

  - 이제 RViz에 Panda 로봇이 표시됩니다:

    ![Panda 로봇이 표시된 RViz](./figs/rviz_start.png)

- Motion Planning 플러그인을 로드한 후에는 구성할 수 있습니다. "Displays" 하위 창의 "Global Options" 탭에서 **Fixed Frame** 필드를 `/panda_link0`로 설정합니다.

- 이제 로봇(여기서는 Panda)에 맞게 플러그인을 구성할 수 있습니다. "Displays" 내의 "MotionPlanning"을 클릭합니다.

  - **Robot Description** 필드가 `robot_description`으로 설정되어 있는지 확인합니다.

  - **Planning Scene Topic** 필드가 `/monitored_planning_scene`으로 설정되어 있는지 확인합니다.
    토픽 이름을 클릭하여 토픽 이름 드롭다운을 표시합니다.

  - **Planned Path** 아래의 **Trajectory Topic**이 `/display_planned_path`로 설정되어 있는지 확인합니다.

  - **Planning Request**에서 **Planning Group**을 `panda_arm`으로 변경합니다. 이는 왼쪽 하단의 MotionPlanning 패널에서도 확인할 수 있습니다.

![RViz 플러그인 시작](./figs/rviz_plugin_start.png)

## 단계 2: 시각화된 로봇으로 실험하기

네 가지 다른 오버랩 시각화가 있습니다:

1. `/monitored_planning_scene` 계획 환경에서 로봇의 구성(기본적으로 활성화됨).

2. 로봇의 계획된 경로(기본적으로 활성화됨).

3. 녹색: 모션 계획을 위한 시작 상태(기본적으로 비활성화됨).

4. 주황색: 모션 계획을 위한 목표 상태(기본적으로 활성화됨).

체크박스를 사용하여 각 시각화의 표시 상태를 켜고 끌 수 있습니다:

1. **Scene Robot** 트리 메뉴에서 **Show Robot Visual** 체크박스를 사용하여 계획 장면 로봇을 표시합니다.

2. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 사용하여 계획된 경로를 표시합니다.

3. **Planning Request** 트리 메뉴에서 **Query Start State** 체크박스를 사용하여 시작 상태를 표시합니다.

4. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 사용하여 목표 상태를 표시합니다.

- 이러한 모든 체크박스를 사용하여 다양한 시각화를 켜고 끌 수 있습니다.

![로봇 시각화 실험](./figs/rviz_plugin_visualize_robots.png)

## 단계 3: Panda와 상호 작용하기

다음 단계에서는 장면 로봇, 시작 상태 및 목표 상태만 필요합니다:

1. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택합니다.

2. **Scene Robot** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택 해제합니다.

3. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 선택합니다.

4. **Planning Request** 트리 메뉴에서 **Query Start State** 체크박스를 선택합니다.

이제 두 개의 대화형 마커가 있어야 합니다. 주황색 로봇 팔에 해당하는 한 마커는 모션 계획을 위한 "목표 상태"를 설정하는 데 사용되고, 녹색 로봇 팔에 해당하는 다른 마커는 모션 계획을 위한 "시작 상태"를 설정하는 데 사용됩니다. 대화형 마커가 보이지 않으면 RViz 상단 메뉴에서 **Interact**를 누릅니다(참고: 일부 도구는 숨겨져 있을 수 있으며, 아래 그림과 같이 상단 메뉴에서 "+"를 눌러 **Interact** 도구를 추가할 수 있습니다).

![RViz에서 상호 작용](./figs/rviz_plugin_interact.png)

이제 이러한 마커를 사용하여 로봇 팔을 드래그하고 방향을 변경할 수 있습니다. 한 번 시도해 보세요!

### 충돌 상태로 이동하기

이 섹션에서는 계획된 경로와 목표 상태를 숨깁니다:

1. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택 해제합니다.

2. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 선택 해제합니다.

이제 시작 상태(녹색 로봇 팔)만 보여야 합니다. 로봇 팔의 두 링크가 서로 충돌하는 구성으로 이동해 보세요. (이것이 어렵다면 MotionPlanning 플러그인의 Planning 탭 아래에 있는 "Use Collision-Aware IK" 체크박스가 선택 해제되어 있는지 확인하세요.) 이렇게 하면 충돌 중인 링크가 빨간색으로 바뀝니다.

![충돌 상태의 로봇](./figs/rviz_plugin_collision.png)

이제 "Use Collision-Aware IK" 체크박스를 선택하고 다시 로봇 팔의 두 링크를 충돌시켜 보세요. 체크박스를 선택하면 IK 솔버는 원하는 엔드 이펙터 자세에 대해 충돌이 없는 솔루션을 계속 찾으려고 시도합니다. 체크박스를 선택하지 않으면 솔버는 솔루션에서 충돌이 발생하도록 허용합니다. 체크박스 상태에 관계없이 충돌 중인 링크는 항상 빨간색으로 표시됩니다.

![충돌 인식 IK 체크박스](./figs/rviz_plugin_collision_aware_ik_checkbox.png)

### 도달 가능한 작업 공간 밖으로 이동하기

엔드 이펙터를 도달 가능한 작업 공간 밖으로 이동하려고 하면 어떻게 되는지 확인해 보세요.

![유효하지 않은 상태](./figs/rviz_plugin_invalid.png)

다음 섹션으로 넘어가기 전에 계획된 경로와 목표 상태를 다시 활성화하세요:

1. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택합니다.

2. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 선택합니다.

### 관절 또는 Null Space로 이동하기

**Joints** 탭을 사용하여 단일 관절과 7-DOF 로봇의 중복 관절을 이동할 수 있습니다. 아래 애니메이션과 같이 "null space exploration" 슬라이더를 움직여 보세요.

<video width="700px" controls="true" autoplay="true" loop="true">
    <source src="../../assets/video/rviz_joints_nullspace.webm" type="video/webm">
    엔드 이펙터는 가만히 있는 동안 관절이 움직입니다
</video>

## 단계 4: Panda로 모션 계획 사용하기

- 이제 MoveIt RViz 플러그인에서 Panda로 모션 계획을 시작할 수 있습니다.

  - 시작 상태를 원하는 위치로 이동합니다.

  - 목표 상태를 다른 원하는 위치로 이동합니다.

  - 두 상태 모두 로봇 자체와 충돌하지 않는지 확인합니다.

  - 계획된 경로가 시각화되고 있는지 확인합니다. 또한 **Planned Path** 트리 메뉴에서 **Show Trail** 체크박스를 선택합니다.

- **MotionPlanning** 창의 **Planning** 탭에서 **Plan** 버튼을 누릅니다. 로봇 팔이 움직이는 모습과 궤적을 볼 수 있어야 합니다.

![계획된 경로](./figs/rviz_plugin_planned_path.png)

### 궤적 경유점 내부 살펴보기

RViz에서 궤적을 점 단위로 시각적으로 살펴볼 수 있습니다.

- "Panels" 메뉴에서 "Trajectory - Trajectory Slider"를 선택합니다. RViz에 새로운 슬라이더 패널이 표시됩니다.

- 목표 자세를 설정한 다음 Plan을 실행합니다.

- "Slider" 패널을 사용해 보세요. 예를 들어 슬라이더를 움직이거나 "Play" 버튼을 누릅니다.

참고: 엔드 이펙터를 새로운 목표로 이동한 후에는 Play를 실행하기 전에 반드시 Plan을 실행해야 합니다. 그렇지 않으면 이전 목표에 대한 경유점(가능한 경우)이 표시됩니다.

![슬라이더 패널](./figs/rviz_plugin_slider.png)

### 카테시안 모션 계획하기

"Use Cartesian Path" 체크박스가 활성화되면 로봇은 엔드 이펙터를 카테시안 공간에서 선형으로 이동하려고 시도합니다.

![자유 계획](./figs/rviz_plan_free.png)

![카테시안 계획](./figs/rviz_plan_cartesian.png)

### 궤적 실행 및 속도 조정

성공적인 계획 후 "Plan & Execute" 또는 "Execute"를 클릭하면 궤적이 로봇으로 전송됩니다. 이 튜토리얼에서는 `demo.launch`를 사용했기 때문에 로봇은 시뮬레이션됩니다.

초기에는 기본 속도와 가속도가 로봇 최대값의 10% (`0.1`)로 조정됩니다. 아래에 표시된 Planning 탭에서 이러한 배율 인자를 변경하거나 로봇의 `moveit_config`에서 `joint_limits.yaml` 파일의 기본값을 변경할 수 있습니다.

![충돌 인식 IK 체크박스](./figs/rviz_plugin_collision_aware_ik_checkbox.png)

## 다음 단계

### RViz Visual Tools

많은 튜토리얼에서는 `moveit_visual_tools`를 사용하여 데모를 단계별로 진행합니다. 다음 튜토리얼로 넘어가기 전에 **RvizVisualToolsGui**를 활성화하는 것이 좋습니다.

"Panels" 메뉴에서 "Add New Panels"를 선택합니다. 메뉴에서 "RvizVisualToolsGui"를 선택하고 OK를 클릭합니다. RViz에 새로운 패널이 추가된 것을 볼 수 있습니다.

![RvizVisualToolsGui 추가](./figs/rviz_add_rviz_visual_tools.png)

![RViz 패널](./figs/rviz_panels.png)

### 구성 저장하기

RViz에서는 `File->Save Config`를 통해 구성을 저장할 수 있습니다. 다음 튜토리얼로 넘어가기 전에 이 작업을 수행해야 합니다. 새로운 이름으로 구성을 저장하려면 `File->Save Config As`를 사용하고 다음과 같이 구성 파일을 참조할 수 있습니다:

```bash
ros2 launch moveit2_tutorials demo.launch.py rviz_config:=your_rviz_config.rviz
```

`your_rviz_config.rviz`를 `book/week10/quickstart_in_rviz/launch/` 디렉토리에 저장한 파일 이름으로 바꾸고 작업 공간을 빌드하여 찾을 수 있도록 합니다.

### 다음 튜토리얼

`첫 번째 MoveIt 프로젝트`에서는 MoveIt을 사용하여 이동을 계획하고 실행하는 C++ 프로그램을 만들 것입니다.
