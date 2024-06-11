# `turtlesim`, `ros2`, 그리고 `rqt` 사용하기

## 배경

Turtlesim은 ROS 2 학습을 위한 경량 시뮬레이터입니다. ROS 2가 가장 기본적인 수준에서 수행하는 작업을 보여주어 나중에 실제 로봇이나 로봇 시뮬레이션에서 수행할 작업에 대한 아이디어를 제공합니다.

ros2 도구는 사용자가 ROS 시스템을 관리, 검사 및 상호 작용하는 방법입니다. 시스템의 다양한 측면과 작동을 대상으로 하는 여러 명령을 지원합니다. 노드 시작, 매개변수 설정, 주제 수신 등 많은 작업에 사용할 수 있습니다. ros2 도구는 핵심 ROS 2 설치의 일부입니다.

rqt는 ROS 2용 그래픽 사용자 인터페이스(GUI) 도구입니다. rqt에서 수행되는 모든 작업은 명령줄에서 수행할 수 있지만, rqt는 ROS 2 요소를 조작하는 더 사용자 친화적인 방법을 제공합니다.

이 튜토리얼에서는 노드, 주제 및 서비스와 같은 핵심 ROS 2 개념을 다룹니다. 이러한 모든 개념은 이후 튜토리얼에서 자세히 설명될 것입니다. 지금은 도구를 설정하고 사용 감각을 익히는 것이 전부입니다.

## 작업

### 1. turtlesim 설치

항상 그렇듯이 이전 튜토리얼에 설명된 대로 새 터미널에서 설정 파일을 소싱하는 것으로 시작합니다.

ROS 2 배포판용 turtlesim 패키지를 설치합니다:

````{tab} Linux
```console
sudo apt update

sudo apt install ros-{DISTRO}-turtlesim
```
````

```{tab} macOS
ROS 2를 설치한 아카이브에 `ros_tutorials` 저장소가 포함되어 있다면 turtlesim이 이미 설치되어 있어야 합니다.
```

```{tab} Windows
ROS 2를 설치한 아카이브에 `ros_tutorials` 저장소가 포함되어 있다면 turtlesim이 이미 설치되어 있어야 합니다.
```

패키지가 설치되었는지 확인합니다:

```console
ros2 pkg executables turtlesim
```

위 명령은 turtlesim의 실행 파일 목록을 반환해야 합니다:

```console
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### 2. turtlesim 시작

turtlesim을 시작하려면 터미널에 다음 명령을 입력합니다:

```console
ros2 run turtlesim turtlesim_node
```

시뮬레이터 창이 나타나고 가운데에 랜덤한 거북이가 보일 것입니다.

![](figs/turtlesim.png)

터미널에서 명령 아래에 노드의 메시지가 표시됩니다:

```console
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

여기에서 기본 거북이의 이름과 생성되는 좌표를 볼 수 있습니다.

### 3. turtlesim 사용

새 터미널을 열고 ROS 2를 다시 소싱합니다.

이제 첫 번째 노드의 거북이를 제어할 새 노드를 실행합니다:

```console
ros2 run turtlesim turtle_teleop_key
```

이 시점에서 `turtlesim_node`를 실행하는 터미널, `turtle_teleop_key`를 실행하는 터미널, turtlesim 창 등 세 개의 창이 열려 있어야 합니다. turtlesim 창을 볼 수 있지만 `turtle_teleop_key`를 실행하는 터미널도 활성화하여 turtlesim에서 거북이를 제어할 수 있도록 이러한 창을 정렬하세요.

키보드의 화살표 키를 사용하여 거북이를 제어하세요. 거북이가 화면 주위를 움직이며 연결된 "펜"을 사용하여 지금까지 이동한 경로를 그릴 것입니다.

.. note::
화살표 키를 누르면 거북이가 짧은 거리만 이동하고 멈춥니다. 이는 실제로 운영자가 로봇과의 연결을 잃은 경우와 같이 로봇이 계속 명령을 수행하는 것을 원하지 않기 때문입니다.

각 명령의 `list` 하위 명령을 사용하여 노드 및 관련 주제, 서비스, 액션을 볼 수 있습니다:

```console
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

다가오는 튜토리얼에서 이러한 개념에 대해 자세히 배우게 될 것입니다. 이 튜토리얼의 목표는 turtlesim에 대한 일반적인 개요를 제공하는 것뿐이므로 rqt를 사용하여 일부 turtlesim 서비스를 호출하고 `turtlesim_node`와 상호 작용할 것입니다.

### 4. rqt 설치

`rqt` 및 해당 플러그인을 설치하려면 새 터미널을 엽니다:

````{tab} Ubuntu
```console
sudo apt update

sudo apt install ~nros-{DISTRO}-rqt*
```
````

````{tab} RHEL
```console
sudo dnf install ros-{DISTRO}-rqt*
```
````

```{tab} macOS
macOS용 ROS 2 설치 표준 아카이브에는 `rqt` 및 해당 플러그인이 포함되어 있으므로 `rqt`가 이미 설치되어 있어야 합니다.
```

```{tab} Windows
Windows용 ROS 2 설치 표준 아카이브에는 `rqt` 및 해당 플러그인이 포함되어 있으므로 `rqt`가 이미 설치되어 있어야 합니다.
```

rqt를 실행하려면:

```console
rqt
```

### 5. rqt 사용

rqt를 처음 실행하면 창이 비어 있습니다. 걱정하지 마세요. 상단 메뉴 모음에서 **Plugins** > **Services** > **Service Caller**를 선택하기만 하면 됩니다.

.. note::
rqt가 모든 플러그인을 찾는 데 시간이 걸릴 수 있습니다. **Plugins**를 클릭했지만 **Services** 또는 다른 옵션이 보이지 않는 경우 rqt를 닫고 터미널에 `rqt --force-discover` 명령을 입력해야 합니다.

![](figs/rqt.png)

turtlesim 노드의 모든 서비스를 사용할 수 있는지 확인하려면 **Service** 드롭다운 목록 왼쪽에 있는 새로 고침 버튼을 사용하세요.

**Service** 드롭다운 목록을 클릭하여 turtlesim의 서비스를 확인하고 `/spawn` 서비스를 선택하세요.

#### 5.1 spawn 서비스 사용해 보기

rqt를 사용하여 `/spawn` 서비스를 호출해 보겠습니다. `/spawn`의 이름에서 유추할 수 있듯이 `/spawn`은 turtlesim 창에 또 다른 거북이를 생성합니다.

**Expression** 열의 빈 작은따옴표 사이를 두 번 클릭하여 새 거북이에 `turtle2`와 같은 고유한 이름을 지정하세요. 이 표현식이 **name**의 값에 해당하고 **string** 유형임을 알 수 있습니다.

그 다음 새 거북이를 생성할 유효한 좌표를 입력하세요(예: `x = 1.0`, `y = 1.0`).

![](figs/spawn.png)

.. note::
기본 `turtle1`과 같은 기존 거북이와 동일한 이름으로 새 거북이를 생성하려고 하면 `turtlesim_node`를 실행하는 터미널에 다음과 같은 오류 메시지가 표시됩니다:

```console
[ERROR] [turtlesim]: A turtle named [turtle1] already exists
```

`turtle2`를 생성하려면 rqt 창 오른쪽 상단에 있는 **Call** 버튼을 클릭하여 서비스를 호출해야 합니다.

서비스 호출이 성공하면 **x**와 **y**에 입력한 좌표에서 새 거북이(다시 무작위 디자인)가 생성되는 것을 볼 수 있습니다.

rqt에서 서비스 목록을 새로 고치면 `/turtle1/...` 외에 새 거북이 `/turtle2/...`와 관련된 서비스도 있음을 알 수 있습니다.

#### 5.2 set_pen 서비스 사용해 보기

이제 `/set_pen` 서비스를 사용하여 `turtle1`에 고유한 펜을 제공해 보겠습니다:

![](figs/set_pen.png)

0에서 255 사이의 **r**, **g**, **b** 값은 `turtle1`이 그리는 펜의 색을 설정하고 **width**는 선의 두께를 설정합니다.

`turtle1`이 뚜렷한 빨간색 선으로 그리게 하려면 **r** 값을 255로, **width** 값을 5로 변경하세요. 값을 업데이트한 후 서비스를 호출하는 것을 잊지 마세요.

`turtle_teleop_key`가 실행 중인 터미널로 돌아가 화살표 키를 누르면 `turtle1`의 펜이 변경된 것을 볼 수 있습니다.

![](figs/new_pen.png)

`turtle2`를 움직일 방법이 없다는 것도 아마 알아차렸을 것입니다. 그것은 `turtle2`에 대한 teleop 노드가 없기 때문입니다.

### 6. 리맵핑

`turtle2`를 제어하려면 두 번째 teleop 노드가 필요합니다. 그러나 이전과 동일한 명령을 실행하려고 하면 이 명령도 `turtle1`을 제어한다는 것을 알 수 있습니다. 이 동작을 변경하는 방법은 `cmd_vel` 주제를 리맵핑하는 것입니다.

새 터미널에서 ROS 2를 소싱하고 다음을 실행합니다:

```console
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

이제 이 터미널이 활성화되면 `turtle2`를 움직일 수 있고, `turtle_teleop_key`를 실행하는 다른 터미널이 활성화되면 `turtle1`을 움직일 수 있습니다.

![](figs/remap.png)

### 7. turtlesim 닫기

시뮬레이션을 중지하려면 `turtlesim_node` 터미널에서 `Ctrl + C`를 입력하고 `turtle_teleop_key` 터미널에서 `q`를 입력할 수 있습니다.

## 요약

turtlesim과 rqt를 사용하는 것은 ROS 2의 핵심 개념을 배우는 좋은 방법입니다.
