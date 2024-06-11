# ROS 환경 설정

ROS (Robot Operating System)는 로봇 소프트웨어 개발을 위한 오픈 소스 프레임워크입니다. ROS 2는 ROS의 차세대 버전으로, 더 나은 성능과 확장성을 제공합니다. 이 글에서는 ROS 2 개발 환경을 설정하는 과정을 단계별로 설명하겠습니다.

## ROS 2 패키지 설치

먼저, ROS 2 패키지를 설치해야 합니다. 다음 명령어를 사용하여 필요한 패키지를 설치할 수 있습니다:

```bash
sudo apt update
sudo apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*
```

- `ros-foxy-desktop`: ROS 2 Foxy 버전의 데스크탑 패키지입니다. 기본적인 ROS 2 도구와 라이브러리가 포함되어 있습니다.
- `ros-foxy-rmw-fastrtps*`, `ros-foxy-rmw-cyclonedds*`: ROS 2의 미들웨어 구현체인 Fast-RTPS와 CycloneDDS에 대한 패키지입니다.

### ROS 2 패키지 설치 확인

설치가 완료되었다면, 두 개의 터미널 창에서 ROS 2의 기본 퍼블리셔(publisher)와 서브스크라이버(subscriber) 노드를 실행하여 설치를 확인할 수 있습니다.

터미널 1:

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

터미널 2:

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

`talker` 노드는 메시지를 퍼블리시하고, `listener` 노드는 해당 메시지를 수신하여 출력합니다. 두 노드가 정상적으로 동작하면 ROS 2 설치가 잘 된 것입니다.

## ROS 개발 도구 설치

ROS 2를 사용한 로봇 프로그래밍에 필요한 추가 도구들을 설치합니다. 다음 명령어를 사용하여 필수 소프트웨어를 한 번에 설치할 수 있습니다:

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
```

- `build-essential`, `cmake`, `git`: 빌드 도구, CMake, Git 버전 관리 시스템입니다.
- `libbullet-dev`: Bullet 물리 엔진 라이브러리입니다.
- `python3-colcon-common-extensions`, `python3-vcstool`, `python3-rosdep`, `python3-setuptools`: 빌드 도구 colcon, vcstool, rosdep 등을 위한 Python 패키지입니다.
- `python3-flake8`, `python3-pytest-cov`: Python 코드 스타일 체커와 테스트 도구입니다.

추가로 몇 가지 Python 패키지를 설치합니다:

```bash
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
```

마지막으로 C++ 라이브러리를 설치합니다:

```bash
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev
```

## ROS 2 빌드 테스트

ROS 2 설치를 마쳤으면 워크스페이스를 생성하고 빌드를 테스트해봅니다. 다음 명령어를 사용하여 `robot_ws`라는 이름의 워크스페이스를 생성합니다:

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/
colcon build --symlink-install
```

- `source /opt/ros/foxy/setup.bash`: ROS 2 Foxy 환경을 설정합니다.
- `mkdir -p ~/robot_ws/src`: 워크스페이스 디렉토리와 `src` 서브디렉토리를 생성합니다.
- `cd ~/robot_ws/`: 워크스페이스 디렉토리로 이동합니다.
- `colcon build --symlink-install`: colcon을 사용하여 워크스페이스를 빌드합니다. `--symlink-install` 옵션은 빌드 결과물을 심볼릭 링크로 설치합니다.

빌드가 성공하면 워크스페이스 디렉토리 아래에 `build`, `install`, `log` 서브디렉토리가 생성됩니다.

## Run Commands 설정

ROS 2 개발 과정에서 자주 사용하는 명령어들을 간편하게 실행할 수 있도록 셸의 `bashrc` 파일에 별칭(alias)과 환경 변수를 설정합니다. `nano` 편집기로 `bashrc` 파일을 열어 다음 내용을 추가합니다:

```bash
nano ~/.bashrc
```

```bash
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cb='cd ~/robot_ws && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'
```

- `source /opt/ros/foxy/setup.bash`, `source ~/robot_ws/install/local_setup.bash`: ROS 2 환경과 워크스페이스 환경을 설정합니다.
- `export ROS_DOMAIN_ID=7`, `export ROS_NAMESPACE=robot1`: ROS 도메인 ID와 네임스페이스를 설정합니다.
- `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`: ROS 2 미들웨어 구현체로 Fast-RTPS를 사용합니다.
- `export RCUTILS_*`: ROS 2 로깅 관련 환경 변수를 설정합니다.
- `alias cb`, `alias cbs`, `alias cbp`, `alias cbu`, `alias ct`, `alias ctp`, `alias ctr`: colcon 빌드와 테스트를 위한 별칭입니다.
- `alias rt`, `alias re`, `alias rn`: ROS 2 토픽과 노드 관련 명령어에 대한 별칭입니다.
- `alias killgazebo`: Gazebo 시뮬레이터 프로세스를 종료하는 명령어입니다.
- `alias testpub`, `alias testsub`: 테스트용 퍼블리셔와 서브스크라이버 노드를 실행하는 명령어입니다.
- `alias testpubimg`, `alias testsubimg`: 이미지 데이터를 주고받는 테스트용 퍼블리셔와 서브스크라이버 노드를 실행하는 명령어입니다.

## 통합 개발 환경(IDE) 설치

ROS 2 개발을 위해 추천하는 IDE는 Visual Studio Code (VS Code)입니다. VS Code는 다양한 운영 체제를 지원하는 경량 텍스트 에디터로, 풍부한 확장 기능을 제공합니다. 다음은 ROS 2 개발에 유용한 VS Code 확장 기능들입니다:

- C/C++, CMake, CMake Tools, Python: C/C++와 Python 언어 지원을 위한 확장 기능입니다.
- ROS, URDF, Colcon Tasks: ROS 2, URDF, colcon 빌드 시스템 지원을 위한 확장 기능입니다.
- XML Tools, YAML, Markdown All in One: XML, YAML, Markdown 형식 지원을 위한 확장 기능입니다.
- Highlight Trailing White Spaces, EOF Mark, Better Comments: 코드 가독성 향상을 위한 추가 확장 기능입니다.

### VS Code 환경 설정

VS Code에서 ROS 2 개발을 위한 환경 설정 파일들을 생성합니다.

- `~/.config/Code/User/settings.json`: VS Code 사용자 설정 파일입니다.
- `~/robot_ws/.vscode/c_cpp_properties.json`: C/C++ 프로젝트 설정 파일입니다.
- `~/robot_ws/.vscode/tasks.json`: 빌드 작업 설정 파일입니다.
- `~/robot_ws/.vscode/launch.json`: 디버깅 설정 파일입니다.

각 파일에 ROS 2 개발에 필요한 설정들을 추가합니다.

#### User settings 설정

`settings.json` 파일에는 VS Code의 사용자 설정을 정의합니다. 다음은 ROS 2 개발에 유용한 설정들의 예시입니다:

```json
{
  "cmake.configureOnOpen": false,
  "editor.minimap.enabled": false,
  "editor.mouseWheelZoom": true,
  "editor.renderControlCharacters": true,
  "editor.rulers": [100],
  "editor.tabSize": 2,
  "files.associations": {
    "_.repos": "yaml",
    "_.world": "xml",
    "*.xacro": "xml"
  },
  "files.insertFinalNewline": true,
  "files.trimTrailingWhitespace": true,
  "terminal.integrated.scrollback": 1000000,
  "workbench.iconTheme": "vscode-icons",
  "workbench.editor.pinnedTabSizing": "compact",
  "ros.distro": "foxy",
  "colcon.provideTasks": true
}
```

- `ros.distro`: ROS 배포판을 지정합니다. 여기서는 "foxy"로 설정되어 있습니다.
- `colcon.provideTasks`: colcon 빌드 작업을 사용하도록 설정합니다.
- `files.associations`: ROS 관련 파일 확장자에 대한 연결을 정의합니다.

#### C/C++ properties 설정

`c_cpp_properties.json` 파일에는 C/C++ 프로젝트의 설정을 정의합니다. 다음은 ROS 2 워크스페이스에 대한 설정 예시입니다:

```json
{
  "configurations": [
    {
      "name": "Linux",
      "includePath": [
        "${default}",
        "${workspaceFolder}/**",
        "/opt/ros/foxy/include/**"
      ],
      "defines": [],
      "compilerPath": "/usr/bin/g++",
      "cStandard": "c99",
      "cppStandard": "c++14",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}
```

- `includePath`: 헤더 파일의 검색 경로를 지정합니다. ROS 2 설치 경로와 워크스페이스 경로를 포함합니다.
- `compilerPath`: C++ 컴파일러 경로를 지정합니다.
- `cStandard`, `cppStandard`: C와 C++ 표준을 지정합니다.

#### Tasks 설정

`tasks.json` 파일에는 VS Code에서 실행할 수 있는 작업들을 정의합니다. 다음은 colcon 빌드와 테스트를 위한 작업 설정 예시입니다:

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "colcon: build",
      "type": "shell",
      "command": "colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'",
      "problemMatcher": [],
      "group": {
        "kind": "build",
        "isDefault": true
      }
    },
    {
      "label": "colcon: test",
      "type": "shell",
      "command": "colcon test && colcon test-result"
    },
    {
      "label": "colcon: clean",
      "type": "shell",
      "command": "rm -rf build install log"
    }
  ]
}
```

- `colcon: build`: colcon을 사용하여 워크스페이스를 빌드하는 작업입니다. 기본 빌드 작업으로 설정되어 있습니다.
- `colcon: test`: colcon을 사용하여 테스트를 실행하고 결과를 보여주는 작업입니다.
- `colcon: clean`: 빌드 결과물을 제거하는 작업입니다.

#### Launch 설정

`launch.json` 파일에는 디버깅을 위한 실행 설정을 정의합니다. 다음은 ROS 2 노드를 디버깅하기 위한 설정 예시입니다:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug-rclpy(debugpy)",
      "type": "python",
      "request": "launch",
      "program": "${file}",
      "console": "integratedTerminal"
    },
    {
      "name": "Debug-rclcpp(gbd)",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/install/${input:package}/lib/${input:package}/${input:node}",
      "args": [],
      "preLaunchTask": "colcon: build",
      "stopAtEntry": true,
      "cwd": "${workspaceFolder}",
      "externalConsole": false,
      "MIMode": "gdb",
      "setupCommands": [
        {
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }
      ]
    }
  ],
  "inputs": [
    {
      "id": "package",
      "type": "promptString",
      "description": "package name",
      "default": "topic_service_action_rclcpp_example"
    },
    {
      "id": "node",
      "type": "promptString",
      "description": "node name",
      "default": "argument"
    }
  ]
}
```

- `Debug-rclpy(debugpy)`: Python 노드를 디버깅하기 위한 설정입니다. `debugpy` 디버거를 사용합니다.
- `Debug-rclcpp(gbd)`: C++ 노드를 디버깅하기 위한 설정입니다. `gdb` 디버거를 사용하며, 디버깅 시작 전에 colcon 빌드 작업을 실행합니다.
- `inputs`: 디버깅할 패키지와 노드 이름을 입력받기 위한 설정입니다.

#### 빌드와 디버깅

- `Ctrl + Shift + B` 단축키로 빌드를 수행할 수 있습니다.
- `Ctrl + Shift + D` 단축키로 디버깅을 시작할 수 있습니다. C++과 Python에 대해 별도의 디버깅 설정이 필요합니다.

## ROS 2 삭제

필요한 경우 다음 명령어를 사용하여 ROS 2를 삭제할 수 있습니다:

```bash
sudo apt remove ros-foxy-* && sudo apt autoremove
```

이 명령어는 설치된 모든 ROS 2 Foxy 패키지를 삭제하고 불필요한 의존성 패키지를 정리합니다.

이상으로 ROS 2 개발 환경 설정 과정을 살펴보았습니다. ROS 2 패키지 설치, 개발 도구 설치, 워크스페이스 생성, VS Code 설정 등의 단계를 통해 ROS 2 프로젝트를 시작할 수 있는 기반을 마련할 수 있습니다.

각 단계에서 주요 설정 파일들의 예시를 확인하였습니다. `settings.json`에서는 VS Code의 사용자 설정을, `c_cpp_properties.json`에서는 C/C++ 프로젝트 설정을, `tasks.json`에서는 colcon 빌드와 테스트 작업을, 그리고 `launch.json`에서는 디버깅 설정을 정의하였습니다.

이제 여러분만의 ROS 2 프로젝트를 생성하고, VS Code에서 제공하는 강력한 개발 도구들을 활용하여 효과적으로 개발을 진행해보세요. ROS 2의 다양한 기능과 라이브러리를 사용하여 멋진 로봇 애플리케이션을 만들어보시기 바랍니다!
