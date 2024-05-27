# `move_group` 노드

아래 그림은 MoveIt에서 제공하는 핵심 노드인 `move_group`의 고수준 시스템 아키텍처를 보여줍니다. 이 노드는 통합자 역할을 하며, 모든 개별 컴포넌트를 함께 묶어 사용자가 사용할 수 있는 일련의 ROS 액션과 서비스를 제공합니다.

![move_group 아키텍처](./figs/move_group.png)

## 사용자 인터페이스

사용자는 두 가지 방법으로 `move_group`에서 제공하는 액션과 서비스에 액세스할 수 있습니다:

- **C++에서** - `move_group_interface` 패키지를 사용하면 `move_group`에 쉽게 설정할 수 있는 C++ 인터페이스를 제공합니다.
- **GUI를 통해** - ROS 시각화 도구인 Rviz의 Motion Planning 플러그인을 사용합니다.

## 설정

`move_group`은 ROS 노드입니다. ROS 파라미터 서버를 사용하여 세 가지 종류의 정보를 얻습니다:

1. URDF (Unified Robot Description Format)

   - `move_group`은 `robot_description` 파라미터를 찾아 로봇의 URDF를 가져옵니다.

2. SRDF (Semantic Robot Description Format)

   - `move_group`은 `robot_description_semantic` 파라미터를 찾아 로봇의 SRDF를 가져옵니다. SRDF는 일반적으로 MoveIt Setup Assistant를 사용하는 사용자가 한 번만 생성합니다.

3. MoveIt 설정
   - `move_group`은 관절 제한, 기구학, 모션 플래닝, 인식 및 기타 정보를 포함하여 MoveIt에 특정한 다른 설정을 찾습니다. 이러한 컴포넌트의 설정 파일은 MoveIt Setup Assistant에 의해 자동으로 생성되어 해당 로봇의 MoveIt 설정 패키지의 `config` 디렉토리에 저장됩니다.

## 로봇 인터페이스

`move_group`은 ROS 토픽과 액션을 통해 로봇과 통신합니다. 로봇과 통신하여 현재 상태 정보(관절 위치 등)를 얻고, 로봇 센서에서 포인트 클라우드 및 기타 센서 데이터를 얻고, 로봇의 컨트롤러와 통신합니다.

## 관절 상태 정보

`move_group`은 `/joint_states` 토픽을 수신하여 현재 상태 정보, 즉 로봇의 각 관절이 어디에 있는지 결정합니다. `move_group`은 이 토픽에 대해 여러 발행자의 수신이 가능하며, 로봇 상태에 대한 부분 정보만 발행하는 경우에도 가능합니다(예: 로봇의 팔과 이동 베이스에 대해 별도의 발행자가 사용될 수 있음). `move_group`은 자체 관절 상태 발행자를 설정하지 않습니다. 이는 각 로봇에서 구현해야 하는 부분입니다.

## 좌표 변환 정보

`move_group`은 ROS TF 라이브러리를 사용하여 좌표 변환 정보를 모니터링합니다. 이를 통해 노드는 로봇의 전역 자세에 대한 정보(다른 것들 중)를 얻을 수 있습니다. 예를 들어 ROS 네비게이션 스택은 맵 프레임과 로봇의 베이스 프레임 사이의 변환을 TF에 발행합니다. `move_group`은 TF를 사용하여 이 변환을 내부적으로 파악할 수 있습니다. `move_group`은 TF를 수신하기만 합니다. 로봇에서 TF 정보를 발행하려면 `robot_state_publisher` 노드를 로봇에서 실행해야 합니다.

## 컨트롤러 인터페이스

`move_group`은 FollowJointTrajectoryAction 인터페이스를 사용하여 로봇의 컨트롤러와 통신합니다. 이는 ROS 액션 인터페이스입니다. 로봇의 서버는 이 액션을 처리해야 하며, 이 서버는 `move_group` 자체에서 제공되지 않습니다. `move_group`은 로봇의 이 컨트롤러 액션 서버와 통신하기 위한 클라이언트만 인스턴스화합니다.

## 플래닝 Scene

`move_group`은 Planning Scene Monitor를 사용하여 **플래닝 Scene**을 유지 관리합니다. 플래닝 Scene은 세계와 로봇의 현재 상태를 표현한 것입니다. 로봇 상태에는 로봇에 부착된(로봇이 운반하는) 객체가 포함될 수 있으며, 이는 로봇에 견고하게 부착된 것으로 간주됩니다. **플래닝 Scene**을 유지 관리하고 업데이트하기 위한 아키텍처에 대한 자세한 내용은 아래의 플래닝 Scene 섹션에 설명되어 있습니다.

## 확장 가능한 기능

`move_group`은 쉽게 확장할 수 있도록 구성되어 있습니다. 픽앤플레이스, 기구학, 모션 플래닝과 같은 개별 기능은 실제로 공통 기본 클래스를 가진 별도의 플러그인으로 구현됩니다. 플러그인은 ROS를 통해 일련의 ROS YAML 매개변수와 ROS pluginlib 라이브러리를 사용하여 구성할 수 있습니다. 대부분의 사용자는 MoveIt Setup Assistant에서 생성된 실행 파일에 자동으로 구성되어 있으므로 move_group 플러그인을 구성할 필요가 없습니다.
