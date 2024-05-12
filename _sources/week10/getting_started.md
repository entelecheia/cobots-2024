# MoveIt 2 시작하기

이 가이드에서는 MoveIt 2 튜토리얼을 원활하게 실행할 수 있도록 개발 환경을 설정하는 방법을 안내합니다. 여기에는 Colcon 작업 공간 생성, 최신 MoveIt 소스 코드 다운로드, 모든 것을 소스에서 빌드하는 과정이 포함됩니다. 이를 통해 최신 버그 수정 사항과 개선 사항이 적용된 MoveIt을 사용할 수 있습니다.

MoveIt의 모든 소스 코드를 빌드하는 데는 컴퓨터 성능에 따라 20-30분 정도 소요될 수 있습니다. 성능이 낮은 시스템을 사용하거나 더 빠르게 시작하고 싶다면 Docker를 사용하는 것도 좋은 방법입니다.

## ROS 2 및 Colcon 설치하기

먼저 ROS 2를 설치해야 합니다. ROS 2 설치 과정에서 누락되는 단계가 있을 수 있으니 주의 깊게 따라가시기 바랍니다. 특히 ROS 2 설치 자체를 소싱하는 것을 잊지 마세요.

```bash
source /opt/ros/{ROS 2 버전}/setup.bash
```

> **참고**: ROS 1과 달리 ROS 2에서는 설정 스크립트가 사용 중인 ROS 버전을 자동으로 전환하지 않습니다. 따라서 이전에 다른 버전의 ROS를 소싱한 경우 빌드 단계에서 오류가 발생할 수 있습니다. 이를 해결하려면 `.bashrc` 파일에서 소싱하는 내용을 변경하고 새 터미널을 시작하세요.

다음으로 `rosdep`을 설치하여 시스템 종속성을 설치합니다.

```bash
sudo apt install python3-rosdep
```

ROS 2를 설치한 후에는 최신 패키지로 업데이트합니다.

```bash
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

이제 ROS 2 빌드 시스템인 Colcon을 설치합니다. 이때 `colcon-mixin`도 함께 설치해야 합니다.

```bash
sudo apt install python3-colcon-common-extensions python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

마지막으로 `vcstool`을 설치합니다.

```bash
sudo apt install python3-vcstool
```

## Colcon 작업 공간 생성 및 소스 코드 다운로드

Colcon 작업 공간을 생성합니다.

```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
```

MoveIt 튜토리얼 소스 코드를 다운로드합니다.

```bash
git clone https://github.com/ros-planning/moveit2_tutorials -b humble
```

MoveIt의 나머지 소스 코드를 다운로드합니다.

```bash
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```

> **참고**: 소스 코드를 가져오는 과정에서 GitHub 자격 증명을 요청할 수 있습니다. 이때는 그냥 Enter 키를 누르면서 "Authentication failed" 오류를 무시하면 됩니다.

## MoveIt 빌드하기

먼저 작업 공간에 필요한 모든 종속성을 설치합니다.

```bash
cd ~/ws_moveit2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

이제 Colcon을 사용하여 MoveIt을 빌드합니다.

```bash
colcon build --mixin release
```

빌드 시간은 컴퓨터 성능에 따라 20분 이상 소요될 수 있습니다. 32GB 이상의 RAM을 권장합니다. 메모리가 부족하거나 빌드에 어려움이 있다면 `--parallel-workers 1` 옵션을 추가해보세요.

빌드가 성공적으로 완료되면 "finished" 메시지가 표시됩니다. 문제가 있다면 ROS 2 설치 과정을 다시 확인해 보시기 바랍니다.

## 빌드 결과 사용하기

빌드가 완료되면 다음 명령을 사용하여 빌드 결과를 소싱합니다.

```bash
source ~/ws_moveit2/install/setup.bash
```

편의를 위해 이 명령을 `.bashrc` 파일에 추가할 수도 있습니다.

```bash
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
```

> **참고**: `.bashrc`에서 자동으로 소싱하는 것은 필수는 아니지만, 여러 개의 Colcon 작업 공간을 사용하지 않는 경우에는 편리합니다.

## ROS 2 미들웨어 변경하기

현재 기본 ROS 2 미들웨어(RMW)에는 버그가 있습니다. 이를 해결하기 위해 Cyclone DDS를 사용합니다.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

> **참고**: Cyclone DDS를 사용하면 다른 미들웨어를 사용하는 노드와 호환되지 않습니다.

## 다음 단계

축하합니다! 이제 MoveIt 2 개발 환경 설정이 완료되었습니다. 다음 단계로 RViz에서 로봇을 시각화하고 모션 플래닝을 수행하는 방법을 살펴보겠습니다.
