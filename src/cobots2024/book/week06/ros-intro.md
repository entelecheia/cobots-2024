# ROS 소개

## Why ROS 2?

Open Robotics에서는 2019년 5월에 "Why ROS 2"라는 문서를 공개하였습니다. 이 문서는 ROS 2를 사용해야 하는 이유를 설명하고 있으며, 개발자가 아닌 사용자 관점에서 작성되었습니다. ROS 2 TSC (Technical Steering Committee) 멤버들이 ROS 1 사용자들에게 보내는 메시지라고 볼 수 있습니다.

이 문서는 Open Robotics의 CEO인 Brian Gerkey가 초안을 작성하였고, ROS 2를 개발하고 사용하는 실제 사용자인 ROS 2 TSC 멤버들의 의견을 수렴하여 최종 작성되었습니다. ROS 2의 활성화를 염두에 둔 문서라고 할 수 있겠습니다.

## 왜 ROS 2인가?

"Why ROS 2" 문서의 내용을 요약하면 다음과 같습니다:

1. **시장 출시 시간 단축**: ROS 2는 로봇 애플리케이션 개발에 필요한 도구, 라이브러리, 기능을 제공하므로 개발 시간을 단축할 수 있습니다.

2. **생산을 위한 설계**: ROS 2는 산업용 수준으로 개발되었으며, 높은 신뢰성과 안전성을 중시합니다.

3. **멀티 플랫폼**: ROS 2는 Linux, Windows, macOS에서 지원 및 테스트되고 있습니다.

4. **다중 도메인**: ROS 2는 실내외, 가정, 자동차, 수중, 우주 등 다양한 로봇 응용 분야에서 사용할 수 있습니다.

5. **벤더 선택 가능**: ROS 2는 오픈소스 솔루션과 독점 솔루션을 모두 지원합니다.

6. **공개 표준 기반**: ROS 2는 IDL, DDS, DDS-RTPS 등의 산업 표준을 사용합니다.

7. **넓은 범위의 오픈소스 라이선스**: ROS 2 코드는 Apache 2.0 라이선스를 기본으로 사용하여 자유롭게 활용할 수 있습니다.

8. **글로벌 커뮤니티**: ROS 2는 ROS 커뮤니티에 의해 개발되었습니다.

9. **산업 지원**: 많은 글로벌 기업들이 ROS 2를 지원하고 있습니다.

10. **ROS 1과의 상호 운용성**: ROS 2는 ROS 1과의 브리지를 통해 양방향 통신이 가능합니다.

## ROS 2의 특징

### 플랫폼 (Platforms)

ROS 2는 Linux, Windows, macOS를 모두 지원합니다. 이는 바이너리 파일로 설치가 가능하다는 의미이기도 합니다. ROS 2 Foxy Fitzroy 기준으로 Linux는 Ubuntu 20.04, Windows는 Windows 10, macOS는 Mojave (10.14)를 지원하고 있습니다.

- 플랫폼 (Platform): 특정 프로세스나 프로그램이 실행되는 하드웨어 및 소프트웨어 환경을 의미합니다. 운영체제, CPU 아키텍처 등이 플랫폼에 해당합니다.

### 실시간성 (Real-time)

ROS 2는 실시간성을 지원합니다. 단, 이를 위해서는 선별된 하드웨어 사용, 실시간 운영체제 사용, DDS의 RTPS (Real-time Publish-Subscribe Protocol) 같은 통신 프로토콜 사용, 잘 짜여진 실시간 코드 사용 등이 전제되어야 합니다.

- 실시간성 (Real-time): 정해진 시간 내에 반드시 어떤 일이 수행됨을 보장하는 것을 의미합니다. 딱딱한 실시간성 (Hard real-time)은 정해진 시간을 엄격히 지켜야 하며, 부드러운 실시간성 (Soft real-time)은 어느 정도 융통성이 있습니다.

### 보안 (Security)

ROS 1에서는 보안이 취약한 부분이 있었습니다. 하지만 ROS 2에서는 디자인 설계 단계부터 보안을 고려하였습니다. DDS-Security라는 DDS 보안 사양을 적용하여 통신 단계에서부터 보안 이슈를 해결하였고, SROS 2 (Secure Robot Operating System 2)라는 툴을 개발하여 보안 관련 프로그래밍을 쉽게 할 수 있도록 하였습니다.

### 통신 (Communication)

ROS 1에서는 자체 개발한 TCPROS와 같은 통신 라이브러리를 사용한 반면, ROS 2는 DDS (Data Distribution Service)를 사용합니다. DDS는 OMG (Object Management Group)에서 표준화를 진행 중이며, 상업적인 용도에도 적합한 것으로 평가받고 있습니다.

DDS에서는 IDL (Interface Description Language)를 사용하여 메시지 정의 및 직렬화를 더 쉽고 포괄적으로 다룰 수 있습니다. 또한 RTPS (Real Time Publish Subscribe) 프로토콜을 사용하여 실시간 데이터 전송을 보장하고 임베디드 시스템에도 사용할 수 있습니다.

- 미들웨어 (Middleware): 분산 컴퓨팅 환경에서 서비스와 서비스, 서비스와 어플리케이션 간의 통신을 관리하는 소프트웨어 층입니다.
- IDL (Interface Description Language): 프로그래밍 언어에 독립적인 인터페이스를 정의하기 위한 언어입니다.

### 미들웨어 인터페이스 (Middleware Interface)

ROS 2에서는 여러 DDS 구현체를 지원하기 위해 RMW (ROS Middleware) 인터페이스를 제공합니다. 이를 통해 사용자는 특정 DDS 벤더에 종속되지 않고 원하는 DDS 구현체를 선택하여 사용할 수 있습니다.

현재 ROS 2를 지원하는 DDS 벤더는 eProsima (Fast DDS), RTI (Connext DDS), Eclipse Foundation (Cyclone DDS), Gurum Networks (GurumDDS), ADLINK (Vortex OpenSplice)가 있습니다.

### 노드 관리 (Node Manager, Discovery)

ROS 1에서는 roscore라는 마스터 노드가 각 노드들의 정보를 관리하고 노드 간 연결을 담당했습니다. 반면 ROS 2에서는 DDS의 Discovery 기능을 활용하여 노드 간 자동 검색 및 연결이 이루어집니다. 따라서 ROS 2에서는 roscore가 더 이상 필요하지 않습니다.

- Discovery: DDS에서 데이터를 주고받는 Publisher와 Subscriber가 서로를 찾아 자동으로 연결되는 과정을 말합니다.

### 프로그래밍 언어 (Languages)

ROS 2에서도 C++과 Python이 주요 프로그래밍 언어로 지원됩니다. ROS 1에 비해 최신 버전의 언어 사양을 지원하는데, C++의 경우 C++14를, Python의 경우 Python 3를 기본으로 합니다.

- ROS 1: C++03, Python 2.7
- ROS 2: C++14 (C++17), Python 3 (3.5+)

### 빌드 시스템 (Build System)

ROS 2에서는 colcon이라는 새로운 빌드 시스템을 사용합니다. colcon은 ROS 1의 catkin 빌드 시스템의 발전된 형태로, Python 패키지 빌드를 catkin보다 훨씬 유연하게 지원합니다. 또한 빌드 시 symlink-install 옵션을 제공하여 빌드 결과물이 설치 단계 없이 바로 테스트 가능하도록 합니다.

- 빌드 시스템 (Build system): 소스 코드를 컴파일하고 실행 가능한 프로그램이나 라이브러리로 만드는 과정을 자동화해주는 시스템입니다.

### 버전 관리 (Version Control)

과거 ROS 1에서는 wstool을 사용하여 다양한 ROS 패키지의 저장소를 관리했습니다. ROS 2에서는 wstool 대신 vcstool을 사용하며, 저장소 설정 파일의 확장자도 .rosinstall에서 .repos로 변경되었습니다.

- VCS (Version Control System): 파일의 변경 사항을 시간에 따라 기록하여 과거 특정 시점의 버전을 다시 불러올 수 있는 시스템입니다.

### 클라이언트 라이브러리 (Client Library)

ROS 2에서는 RCL (ROS Client Library)이라는 이름으로 각 프로그래밍 언어별 클라이언트 라이브러리를 제공합니다. C++의 경우 rclcpp, Python의 경우 rclpy라는 이름을 사용합니다. ROS 1에 비해 더 많은 언어를 지원하며, ROS 2의 새로운 기능들을 사용할 수 있는 인터페이스를 제공합니다.

### 라이프사이클 (Life Cycle)

ROS 2에서는 각 노드의 상태를 관리할 수 있는 라이프사이클 기능을 도입했습니다. 이를 통해 노드의 현재 상태를 모니터링하고, 상태 변경을 제어할 수 있습니다. 이는 로봇 시스템의 안정성과 관리 용이성을 높이는 데 도움이 됩니다.

- 라이프사이클 (Lifecycle): 노드가 생성되어 소멸되기까지의 전체 과정을 말합니다. ROS 2에서는 노드의 상태를 Unconfigured, Inactive, Active, Finalized 등으로 정의하고 있습니다.

### 멀티 노드 (Multiple Nodes)

ROS 2에서는 하나의 프로세스 내에서 여러 개의 노드를 실행할 수 있습니다. 이를 컴포넌트 (Components)라고 부릅니다. 이는 ROS 1의 Nodelet과 유사한 개념으로, 프로세스 내 통신 (Intra-process Communication)을 활용하여 노드 간 통신 오버헤드를 줄일 수 있습니다.

### 실행 모델 (Executor)

ROS 2에서는 실행 모델이 더욱 유연해졌습니다. 단일 스레드 실행, 멀티 스레드 실행 등을 선택할 수 있으며, 사용자 정의 실행 모델도 만들 수 있습니다. 이를 통해 시스템의 요구사항에 맞는 실행 모델을 구현할 수 있습니다.

- 실행 모델 (Executor): 노드의 콜백 함수들을 실행하는 주체입니다. ROS 2에서는 Single-threaded Executor, Multi-threaded Executor, Static Single-threaded Executor 등을 기본으로 제공합니다.

### 메시지 (Messages)

ROS 2에서도 메시지는 ROS 1과 유사한 형태를 가지고 있습니다. 단, ROS 2에서는 IDL을 사용하여 메시지를 정의하며, DDS 표준에 맞추어 메시지 타입이 조정되었습니다. Topic, Service, Action 등에서 사용되는 메시지의 기본 개념은 ROS 1과 동일합니다.

- 메시지 (Messages): 노드 간에 주고받는 데이터의 타입을 정의한 것입니다. ROS는 표준 메시지 타입을 제공하며, 사용자가 직접 메시지 타입을 정의할 수도 있습니다.

그 외에도 ROS 2는 Launch 파일, 빌드 도구, 임베디드 시스템 지원 등 많은 부분에서 ROS 1과 차이를 보입니다. 하지만 가장 중요한 변화는 통신 미들웨어를 DDS로 교체한 것과, 이를 통해 ROS가 산업용으로 한 단계 도약했다는 점입니다.

### 명령행 인터페이스 (Command-Line Interface)

ROS 2의 명령행 인터페이스는 ROS 1과 매우 유사합니다. 약간의 명령어 변경과 옵션 사용법만 익히면 큰 어려움 없이 사용할 수 있습니다.

- ROS 1: `rostopic list`
- ROS 2: `ros2 topic list`

### roslaunch

ROS 1에서는 roslaunch 명령을 사용하여 여러 노드를 한 번에 실행할 수 있었습니다. ROS 2에서도 유사한 기능을 제공하지만, XML 형식 외에 Python 형식의 실행 파일도 지원합니다. 이를 통해 조건문이나 반복문 등 더 복잡한 실행 시나리오를 구성할 수 있게 되었습니다.

### 그래프 API (Graph API)

ROS 1의 `rqt_graph` 툴은 시스템의 노드와 토픽 간 연결 관계를 시각화해주는 유용한 도구였습니다. ROS 2에서는 이런 기능을 확장하여, 실행 중인 노드와 토픽의 동적인 변화를 실시간으로 표현할 수 있는 Graph API를 제공할 예정입니다. 아직 구현은 되지 않았지만, 큰 기대를 모으고 있는 기능입니다.

### 임베디드 시스템 (Embedded Systems)

로봇 하드웨어를 제어하는 임베디드 시스템은 로봇의 눈과 귀, 그리고 팔다리와 같은 역할을 합니다. ROS 1에서는 rosserial이라는 프로토콜을 통해 마이크로컨트롤러와 통신했지만, ROS 2에서는 좀 더 다양한 방식의 임베디드 시스템 지원을 계획하고 있습니다.

- micro-ROS: ROS 2를 마이크로컨트롤러에서 직접 구동하기 위한 프로젝트입니다.
- DDS-XRCE: 경량화된 DDS 프로토콜로, 제한된 자원을 가진 임베디드 시스템에서의 활용을 위해 고안되었습니다.
- ros2arduino: ROS 2와 아두이노 간의 통신을 위한 패키지입니다.

이처럼 ROS 2는 로봇 개발의 전 분야를 아우르는 폭넓은 지원을 목표로 하고 있습니다.

지금까지 ROS 2의 주요 특징과 ROS 1과의 차이점을 살펴보았습니다. ROS 2는 ROS 1의 장점은 계승하면서도, 현대적인 로봇 시스템이 요구하는 새로운 기능들을 대거 도입하였습니다. 특히 DDS라는 검증된 미들웨어를 통해 시스템의 신뢰성과 실시간성을 높인 점, 그리고 라이프사이클과 실행 모델 등을 통해 노드 단위의 시스템 관리 기능을 강화한 점 등은 ROS 2만의 강점이라고 할 수 있겠습니다.

물론 ROS 2는 아직 발전 과정에 있는 플랫폼이며, 현재도 활발한 개발이 진행되고 있습니다. 앞으로도 ROS 커뮤니티의 집단지성을 통해 더욱 완성도 높은 로봇 소프트웨어 플랫폼으로 진화해 나갈 것으로 기대됩니다. 여러분도 ROS 2와 함께 로봇 개발의 미래를 그려보시기 바랍니다.
