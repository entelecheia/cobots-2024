# MoveIt 작업 생성기(Task Constructor)

MoveIt 작업 생성기(이하 MTC)는 복잡한 작업 계획을 여러 개의 상호 의존적인 하위 작업으로 분할하는데 도움을 주는 프레임워크입니다. MTC는 MoveIt을 사용하여 하위 작업들을 해결하고, 하위 작업들로부터의 정보는 InterfaceState 객체를 통해 전달됩니다.

![MTC 작업 개요](./figs/mtc_task.png)

## MTC 단계(Stages)

MTC 단계는 작업 실행 파이프라인의 구성 요소 또는 단계를 나타냅니다. 단계들은 임의의 순서로 배열될 수 있으며, 그 계층 구조는 개별 단계 유형에 의해서만 제한됩니다. 단계들이 배열될 수 있는 순서는 결과가 전달되는 방향에 의해 제한됩니다.

결과 흐름과 관련하여 세 가지 가능한 단계가 있습니다:

- 생성기(Generators)
- 전파기(Propagators)
- 연결기(Connectors)

### 생성기(Generator) 단계

![생성기 단계](./figs/generating_stage.png)

생성기 단계는 인접한 단계로부터 입력을 받지 않습니다. 그들은 결과를 계산하고 그것을 앞뒤 양방향으로 전달합니다. MTC 작업의 실행은 생성기 단계에서 시작됩니다. 가장 중요한 생성기 단계는 `CurrentState`로, 계획 파이프라인의 시작점으로 현재 로봇 상태를 가져옵니다.

모니터링 생성기는 다른 단계(인접하지 않은)의 솔루션을 모니터링하여 계획에 사용하는 단계입니다. 모니터링 생성기의 예로는 `GeneratePose`가 있습니다. 이는 보통 `CurrentState` 또는 `ModifyPlanningScene` 단계를 모니터링합니다. `CurrentState`의 솔루션을 모니터링함으로써, `GeneratePose` 단계는 자세를 생성해야 하는 객체나 프레임을 찾을 수 있습니다.

### 전파기(Propagating) 단계

![전파기 단계](./figs/propagating_stage.png)

전파기는 인접한 상태의 솔루션을 받아, 문제를 해결하고, 그 결과를 반대쪽 이웃에게 전파합니다. 구현에 따라, 이 단계는 솔루션을 앞으로, 뒤로, 또는 양방향으로 전달할 수 있습니다. 전파기 단계의 예로는 자세에 대한 `Move Relative`가 있습니다. 이는 일반적으로 물체를 집기 위해 가까이 접근할 때 사용됩니다.

### 연결기(Connecting) 단계

![연결기 단계](./figs/connecting_stage.png)

연결기는 결과를 전파하지 않지만, 인접한 단계에서 제공하는 시작과 목표 입력을 연결하려 시도합니다. 연결 단계는 종종 시작 상태와 목표 상태 사이의 실행 가능한 궤적을 찾습니다.

### 래퍼(Wrapper)

래퍼는 결과를 수정하거나 필터링하기 위해 다른 단계를 캡슐화합니다. 래퍼의 예로는 `Generate Grasp Pose` 단계를 위한 `Compute IK`가 있습니다. `Generate Grasp Pose` 단계는 카테시안 자세 솔루션을 생성할 것입니다. `Generate Pose` 단계 주위에 `Compute IK` 단계를 래핑함으로써, `Generate Pose` 단계의 카테시안 자세 솔루션을 사용하여 IK 솔루션(즉, 자세에 도달하기 위한 로봇의 조인트 상태 구성)을 생성할 수 있습니다.

## MTC 컨테이너(Containers)

MTC 프레임워크는 컨테이너를 사용하여 단계의 계층적 구성을 가능하게 하며, 순차적 구성뿐만 아니라 병렬 구성도 허용합니다. MTC 컨테이너는 단계의 실행 순서를 구성하는 데 도움을 줍니다. 프로그래밍 측면에서, 컨테이너 내에 다른 컨테이너를 추가하는 것이 가능합니다.

현재 사용 가능한 컨테이너:

- Serial
- Parallel

### Serial 컨테이너

Serial 컨테이너는 단계를 선형으로 구성하며, 종단 간 솔루션만을 결과로 고려합니다. MTC 작업은 기본적으로 Serial 컨테이너로 저장됩니다.

### Parallel 컨테이너

Parallel 컨테이너는 대체 솔루션 계획을 허용하기 위해 단계들의 집합을 결합합니다.

## MTC 작업 초기화하기

최상위 계획 문제는 MTC 작업으로 지정되고, 단계에 의해 지정된 하위 문제들은 MTC 작업 객체에 추가됩니다.

```cpp
auto node = std::make_shared<rclcpp::Node>();
auto task = std::make_unique<moveit::task_constructor::Task>();
task->loadRobotModel(node);
// 로봇 모션을 실행하는데 사용되는 컨트롤러를 설정합니다. 설정되지 않은 경우, MoveIt은 컨트롤러 탐색 로직을 사용합니다.
task->setProperty("trajectory_execution_info", "joint_trajectory_controller gripper_controller");
```

## MTC 작업에 컨테이너와 단계 추가하기

MTC 작업에 단계 추가하기:

```cpp
auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
task->add(std::move(current_state));
```

컨테이너는 Stage에서 파생되므로, 컨테이너도 유사하게 MTC 작업에 추가할 수 있습니다.

```cpp
auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick Object");
// TODO: 컨테이너를 MTC 작업에 추가하기 전에 컨테이너에 단계를 추가합니다.
task->add(std::move(container));
```

## 계획 솔버 설정하기

모션 계획을 수행하는 단계는 솔버 정보가 필요합니다.

MTC에서 사용 가능한 솔버:

- `PipelinePlanner` - MoveIt의 계획 파이프라인을 사용
- `JointInterpolation` - 시작 및 목표 조인트 상태 사이를 보간합니다. 복잡한 모션은 지원하지 않습니다.
- `CartesianPath` - 엔드 이펙터를 카테시안 공간에서 직선으로 이동시킵니다.

솔버 초기화 방법에 대한 코드 예제:

```cpp
const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
    node, "ompl", "RRTConnectkConfigDefault");
const auto mtc_joint_interpolation_planner =
    std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
```

이러한 솔버들은 `MoveTo`, `MoveRelative`, `Connect`와 같은 단계에 전달될 것입니다.

## 속성 설정하기

각 MTC 단계에는 구성 가능한 속성이 있습니다. 예를 들어, 계획 그룹, 시간 초과, 목표 상태 등이 있습니다. 다양한 유형의 속성은 다음 함수를 사용하여 설정할 수 있습니다.

```cpp
void setProperty(const std::string& name, const boost::any& value);
```

자식 단계는 부모로부터 속성을 쉽게 상속받을 수 있으므로, 구성에 대한 오버헤드를 줄일 수 있습니다.

## 단계의 비용 계산기

CostTerm은 MTC 단계의 솔루션에 대한 비용을 계산하기 위한 기본 인터페이스입니다.

MTC에서 사용 가능한 CostTerm 구현:

- `Constant` - 각 솔루션에 상수 비용을 추가
- `PathLength` - 서로 다른 조인트에 대한 가중치를 선택적으로 사용하여 궤적 길이에 따라 비용이 결정
- `TrajectoryDuration` - 전체 궤적의 실행 시간에 따라 비용이 결정
- `TrajectoryCostTerm` - SubTrajectory 솔루션에서만 작동하는 비용 항목
- `LambdaCostTerm` - 비용을 계산하기 위해 lambda 표현식을 전달
- `DistanceToReference` - 기준점까지의 가중 조인트 공간 거리에 따라 비용이 결정
- `LinkMotion` - 링크의 카테시안 궤적 길이에 따라 비용이 결정
- `Clearance` - 충돌까지의 거리에 반비례하여 비용이 결정

`LambdaCostTerm`을 사용하여 CostTerm을 설정하는 방법에 대한 예제 코드:

```cpp
stage->setCostTerm(moveit::task_constructor::LambdaCostTerm(
      [](const moveit::task_constructor::SubTrajectory& traj) { return 100 * traj.cost(); }));
```

MTC에서 제공하는 모든 단계에는 기본 비용 항목이 있습니다. 솔루션으로 궤적을 생성하는 단계는 보통 경로 길이를 사용하여 비용을 계산합니다.

## MTC 작업 계획 및 실행하기

MTC 작업의 계획은 `MoveItErrorCode`를 반환할 것입니다. 계획이 성공하면 plan 함수가 `moveit_msgs::msg::MoveItErrorCodes::SUCCESS`를 반환하는 것을 기대할 수 있습니다.

```cpp
auto error_code = task.plan()
```

계획 후에, 첫 번째 성공한 솔루션을 추출하여 실행 함수에 전달합니다. 이는 `execute_task_solution` 액션 클라이언트를 생성할 것입니다. 액션 서버는 MTC에서 제공하는 `execute_task_solution_capability` 플러그인에 있습니다. 이 플러그인은 `MoveGroupCapability`를 확장합니다. 이는 MTC 솔루션에서 `MotionPlanRequest`를 구성하고 MoveIt의 `PlanExecution`을 사용하여 로봇을 작동시킵니다.

```cpp
auto result = task.execute(*task.solutions().front());
```
