# 모션 플래닝 Python API

이 튜토리얼에서는 `moveit_py`를 위한 모션 플래닝 API의 기본 사항을 다룰 것입니다.
이 튜토리얼은 다음 섹션으로 구성되어 있습니다:

- **시작하기**: 튜토리얼 설정 요구 사항에 대한 개요입니다.
- **플래닝 매개변수 이해하기**: 지원되는 플래너에 대한 매개변수 설정에 대한 개요입니다.
- **단일 파이프라인 플래닝 (기본 구성)**: 미리 지정된 로봇 구성을 사용하여 플래닝합니다.
- **단일 파이프라인 플래닝 (로봇 상태)**: 로봇 상태 인스턴스를 사용하여 플래닝합니다.
- **단일 파이프라인 플래닝 (포즈 목표)**: 포즈 목표를 사용하여 플래닝합니다.
- **단일 파이프라인 플래닝 (사용자 정의 제약 조건)**: 사용자 정의 제약 조건을 사용하여 플래닝합니다.
- **다중 파이프라인 플래닝**: 여러 플래닝 파이프라인을 병렬로 실행합니다.
- **플래닝 Scene 사용하기**: 충돌 객체 추가 및 제거, 충돌 확인.

## 시작하기

이 튜토리얼을 완료하려면 MoveIt 2와 해당 튜토리얼이 포함된 작업 공간을 설정해야 합니다.
이러한 작업 공간을 설정하는 방법에 대한 개요는 [시작하기 가이드](../../week10/getting_started)에 제공되어 있습니다.
자세한 내용은 이 가이드를 참조하십시오.

작업 공간을 설정한 후에는 다음 명령을 실행하여 이 튜토리얼의 코드를 실행할 수 있습니다:

```bash
ros2 launch moveit2_tutorials motion_planning_python_api_tutorial.launch.py
```

## 플래닝 매개변수 이해하기

MoveIt은 여러 플래닝 라이브러리를 기본적으로 지원하며,
우리가 사용하고자 하는 플래너에 대한 설정/매개변수를 제공하는 것이 중요합니다.

이를 위해 우리는 `moveit_py` 노드와 관련된 매개변수를 정의하는 yaml 구성 파일을 지정합니다.

이러한 구성 파일의 예는 다음과 같습니다:

```yaml
planning_scene_monitor_options:
  name: "planning_scene_monitor"
  robot_description: "robot_description"
  joint_state_topic: "/joint_states"
  attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
  publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
  monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
  wait_for_initial_state_timeout: 10.0

planning_pipelines:
  pipeline_names:
    ["ompl", "pilz_industrial_motion_planner", "chomp", "ompl_rrt_star"]

plan_request_params:
  planning_attempts: 1
  planning_pipeline: ompl
  max_velocity_scaling_factor: 1.0
  max_acceleration_scaling_factor: 1.0

ompl_rrtc:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: ompl
    planner_id: "RRTConnectkConfigDefault"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 1.0

ompl_rrt_star:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: ompl_rrt_star
    planner_id: "RRTstarkConfigDefault"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 1.5

pilz_lin:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: pilz_industrial_motion_planner
    planner_id: "PTP"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 0.8

chomp:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: chomp
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 1.5
```

구성 파일의 첫 번째 블록은 `planning_scene_monitor`가 구독하는 토픽과 같은 플래닝 scene 모니터 옵션을 설정합니다.

```yaml
planning_scene_monitor_options:
  name: "planning_scene_monitor"
  robot_description: "robot_description"
  joint_state_topic: "/joint_states"
  attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
  publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
  monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
  wait_for_initial_state_timeout: 10.0
```

구성 파일의 두 번째 블록은 사용하려는 플래닝 파이프라인을 설정합니다.
MoveIt은 OMPL, Pilz Industrial Motion Planner, STOMP (Stochastic Trajectory Optimization for Motion Planning), SBPL (Search-Based Planning Library), CHOMP (Covariant Hamiltonian Optimization for Motion Planning) 등 여러 모션 플래닝 라이브러리를 지원합니다.
`moveit_py` 노드를 구성할 때는 사용하려는 플래닝 파이프라인에 대한 구성을 지정해야 합니다:

```yaml
planning_pipelines:
  pipeline_names:
    ["ompl", "pilz_industrial_motion_planner", "chomp", "ompl_rrt_star"]
```

이러한 명명된 각 파이프라인에 대해 `planner_id`를 통해 사용할 플래너와 계획 시도 횟수와 같은 기타 설정을 식별하는 구성을 제공해야 합니다:

```yaml
ompl_rrtc:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: ompl
    planner_id: "RRTConnectkConfigDefault"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 0.5

ompl_rrt_star:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: ompl_rrt_star
    planner_id: "RRTstarkConfigDefault"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 1.5

pilz_lin:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: pilz_industrial_motion_planner
    planner_id: "PTP"
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 0.8

chomp:
  plan_request_params:
    planning_attempts: 1
    planning_pipeline: chomp
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planning_time: 1.5
```

지정된 이러한 매개변수는 `moveit_py` 노드 매개변수로 사용 가능하며 플래닝을 수행할 때 런타임에 활용됩니다.
이것이 우리가 다음에 조사할 내용입니다.

## moveit_py 및 플래닝 컴포넌트 인스턴스화

모션을 계획하기 전에 `moveit_py` 노드와 파생된 플래닝 컴포넌트를 인스턴스화해야 합니다.
또한 `rclpy` 로거 객체를 인스턴스화할 것입니다:

```python
rclpy.init()
logger = rclpy.logging.get_logger("moveit_py.pose_goal")

# MoveItPy 인스턴스 생성 및 플래닝 컴포넌트 가져오기
panda = MoveItPy(node_name="moveit_py")
panda_arm = panda.get_planning_component("panda_arm")
logger.info("MoveItPy instance created")
```

`panda_arm` 변수로 표현되는 플래닝 컴포넌트를 사용하여 모션 플래닝을 수행할 수 있습니다.
또한 모션을 계획하고 실행하기 위한 도우미 함수를 정의합니다:

```python
def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """모션을 계획하고 실행하는 도우미 함수."""
        # 목표에 대한 계획
        logger.info("Planning trajectory")
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
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")
```

## 단일 파이프라인 플래닝 - 기본 구성

우리는 미리 정의된 로봇 구성(srdf 파일에 정의됨)으로 계획하는 단일 플래닝 파이프라인을 실행하여 `moveit_py` 모션 플래닝 API 탐색을 시작합니다:

```python
# 미리 정의된 상태를 사용하여 계획 시작 상태 설정
panda_arm.set_start_state(configuration_name="ready")

# 미리 정의된 상태를 사용하여 포즈 목표 설정
panda_arm.set_goal_state(configuration_name="extended")

# 목표에 대한 계획
plan_and_execute(panda, panda_arm, logger)
```

## 단일 파이프라인 플래닝 - 로봇 상태

다음으로 우리는 로봇 상태로 계획할 것입니다.
이러한 방법은 우리가 원하는 대로 로봇 상태 구성을 변경할 수 있으므로(예: 조인트 값 설정을 통해) 매우 유연합니다.
여기서는 `set_start_state_to_current_state` 메서드를 사용하여 로봇의 시작 상태를 현재 상태로 설정하고 `set_goal_state` 메서드를 사용하여 목표 상태를 무작위 구성으로 설정합니다.
그런 다음 목표 상태에 대한 계획을 세우고 실행합니다:

```python
# 현재 로봇 모델을 사용하여 RobotState 인스턴스 생성
robot_model = panda.get_robot_model()
robot_state = RobotState(robot_model)

# 로봇 상태 무작위화
robot_state.set_to_random_positions()

# 현재 상태를 계획 시작 상태로 설정
panda_arm.set_start_state_to_current_state()

# 초기화된 로봇 상태를 목표 상태로 설정
logger.info("Set goal state to the initialized robot state")
panda_arm.set_goal_state(robot_state=robot_state)

# 목표에 대한 계획
plan_and_execute(panda, panda_arm, logger)
```

## 단일 파이프라인 플래닝 - 포즈 목표

목표 상태를 지정하는 또 다른 일반적인 방법은 포즈 목표를 나타내는 ROS 메시지를 통하는 것입니다.
여기서는 로봇의 엔드 이펙터에 대한 포즈 목표를 설정하는 방법을 보여줍니다:

```python
# 현재 상태를 계획 시작 상태로 설정
panda_arm.set_start_state_to_current_state()

# PoseStamped 메시지로 포즈 목표 설정
pose_goal = PoseStamped()
pose_goal.header.frame_id = "panda_link0"
pose_goal.pose.orientation.w = 1.0
pose_goal.pose.position.x = 0.28
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.5
panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")

# 목표에 대한 계획
plan_and_execute(panda, panda_arm, logger)
```

## 단일 파이프라인 플래닝 - 사용자정의 제약 조건

사용자 정의 제약 조건을 통해 모션 플래닝의 출력을 제어할 수도 있습니다.
여기서는 관절 제약 조건 집합을 만족하는 구성으로 계획하는 방법을 보여줍니다:

```python
# 현재 상태를 계획 시작 상태로 설정
panda_arm.set_start_state_to_current_state()

# 제약 조건 메시지 설정
joint_values = {
        "panda_joint1": -1.0,
        "panda_joint2": 0.7,
        "panda_joint3": 0.7,
        "panda_joint4": -1.5,
        "panda_joint5": -0.7,
        "panda_joint6": 2.0,
        "panda_joint7": 0.0,
}
robot_state.joint_positions = joint_values
joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=panda.get_robot_model().get_joint_model_group("panda_arm"),
)
panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

# 목표에 대한 계획
plan_and_execute(panda, panda_arm, logger)
```

## 다중 파이프라인 플래닝

`moveit_cpp`와 `moveit_py`의 최근 추가 기능은 여러 플래닝 파이프라인을 병렬로 실행하고 생성된 모든 모션 계획 중에서 작업 요구 사항을 가장 잘 만족하는 결과 모션 계획을 선택하는 기능입니다.
이전 섹션에서 우리는 일련의 플래닝 파이프라인을 정의했습니다.
여기서는 이러한 파이프라인 중 몇 개를 병렬로 계획하는 방법을 살펴보겠습니다:

```python
# 현재 상태를 계획 시작 상태로 설정
panda_arm.set_start_state_to_current_state()

# PoseStamped 메시지로 포즈 목표 설정
panda_arm.set_goal_state(configuration_name="ready")

# 다중 파이프라인 계획 요청 매개변수 초기화
multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        panda, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
)

# 목표에 대한 계획
plan_and_execute(
        panda,
        panda_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
)

# 계획 실행
if plan_result:
        logger.info("Executing plan")
        panda_arm.execute()
```

## 플래닝 Scene 사용하기

이 섹션의 코드를 실행하려면 다른 Python 파일을 실행해야 합니다. 다음과 같이 지정할 수 있습니다:

```bash
ros2 launch moveit2_tutorials motion_planning_python_api_tutorial.launch.py example_file:=motion_planning_python_api_planning_scene.py
```

플래닝 scene과 상호 작용하려면 플래닝 scene 모니터를 생성해야 합니다:

```python
panda = MoveItPy(node_name="moveit_py_planning_scene")
panda_arm = panda.get_planning_component("panda_arm")
planning_scene_monitor = panda.get_planning_scene_monitor()
```

그런 다음 플래닝 scene 모니터의 `read_write` 컨텍스트를 사용하여 플래닝 scene에 충돌 객체를 추가할 수 있습니다:

```python
with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.id = "boxes"

        box_pose = Pose()
        box_pose.position.x = 0.15
        box_pose.position.y = 0.1
        box_pose.position.z = 0.6

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # scene이 업데이트되도록 하는 것이 중요합니다
```

`CollisionObject.REMOVE` 작업을 사용하거나 scene에서 모든 객체를 제거하여 비슷한 방식으로 객체를 제거할 수 있습니다:

```python
with planning_scene_monitor.read_write() as scene:
        scene.remove_all_collision_objects()
        scene.current_state.update()
```

또한 충돌 확인과 같이 scene을 수정할 필요가 없는 작업에 대해 플래닝 scene 모니터의 `read_only` 컨텍스트를 사용할 수 있습니다.
예를 들면 다음과 같습니다:

```python
with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        original_joint_positions = robot_state.get_joint_group_positions("panda_arm")

        # 포즈 목표 설정
        pose_goal = Pose()
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.25
        pose_goal.position.z = 0.5
        pose_goal.orientation.w = 1.0

        # 로봇 상태 설정 및 충돌 확인
        robot_state.set_from_ik("panda_arm", pose_goal, "panda_hand")
        robot_state.update()  # 변환을 업데이트하는 데 필요
        robot_collision_status = scene.is_state_colliding(
                robot_state=robot_state, joint_model_group_name="panda_arm", verbose=True
        )
        logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

        # 원래 상태로 복원
        robot_state.set_joint_group_positions(
                "panda_arm",
                original_joint_positions,
        )
        robot_state.update()  # 변환을 업데이트하는 데 필요
```

이것으로 `moveit_py` 모션 플래닝 API의 기본 사항을 다루는 튜토리얼을 마칩니다.
MoveIt 2와 Python을 사용한 모션 플래닝에 대해 더 알아보려면 MoveIt 2 문서와 예제를 참조하는 것이 좋습니다.
