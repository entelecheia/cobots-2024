# 패키지와 워크스페이스

## 패키지 (Package)

ROS에서 패키지는 소스 코드와 기타 리소스를 하나의 단위로 묶어 배포하는 것을 말합니다. 패키지는 ROS의 기본 구성 단위로, 노드, 라이브러리, 데이터셋, 설정 파일 등을 포함합니다.

- 배포 (Distribution): 소프트웨어를 사용자에게 전달하는 방식을 말합니다. ROS에서는 바이너리 패키지와 소스 코드 패키지, 두 가지 형태로 배포됩니다.

ROS 패키지는 `apt`를 통해 바이너리 형태로 설치할 수 있습니다. 패키지 이름은 `ros-<distro>-<package_name>`의 형식을 따르며, 언더스코어(`_`)는 대시(`-`)로 변경됩니다. 예를 들어 `ros-iron-rclpy`는 ROS 2 Iron 배포판의 `rclpy` 패키지를 의미합니다.

소스 코드 형태의 ROS 패키지는 직접 다운로드하여 빌드할 수 있습니다. 바이너리 패키지를 사용하는 것이 간편하지만, ROS 코드를 직접 작성하려면 패키지를 직접 만들어야 합니다.

### 패키지 구조

모든 ROS 패키지는 `package.xml`이라는 매니페스트 파일을 포함하고 있습니다. 이 파일은 XML 형식으로 작성되며, 패키지의 메타데이터를 기술합니다. `package.xml`의 전체 스펙은 [ROS REP 149](https://www.ros.org/reps/rep-0149.html)에 정의되어 있습니다.

`package.xml`에서 중요한 요소 중 하나는 `<export>`의 `<build_type>`입니다. 이는 패키지의 빌드 타입을 결정하는데, 다음과 같은 값을 가질 수 있습니다:

- `ament_python`: 순수 Python 패키지에 사용됩니다.
- `ament_cmake`: C++ 패키지나 커스텀 메시지, 서비스, 액션 등을 정의하는 패키지에 사용됩니다.
- `ament_cmake_python`: Python과 C++ 코드가 혼합된 패키지에 사용됩니다.

### package.xml

#### 필수 요소

- `<name>`: 패키지의 이름
- `<version>`: 패키지의 버전
- `<description>`: 패키지에 대한 설명
- `<maintainer>`: 패키지 관리자 정보 (이름과 이메일)
- `<license>`: 패키지의 라이선스 정보

#### 의존성 명시

패키지의 의존성은 다음과 같이 분류됩니다:

- `<exec_depend>`: 런타임에 필요한 패키지. Python 패키지에서 다른 패키지의 코드를 임포트하거나, 런치 파일에서 다른 패키지의 노드를 실행하는 경우 사용합니다. 가장 일반적인 의존성 타입입니다.
- `<build_depend>`: 빌드 타임에 필요한 패키지. Python 코드는 컴파일되지 않으므로 보통 필요하지 않습니다. 단, 다른 패키지의 메시지나 서비스에 의존하는 경우에는 사용합니다.
- `<depend>`: C++ 패키지에서 가장 일반적으로 사용되는 의존성 타입입니다. Python 패키지에서 `std_msgs`와 같은 커스텀 메시지에 의존하는 경우에도 사용합니다. `<depend>`를 사용하면 `<exec_depend>`와 `<build_depend>`가 자동으로 추가됩니다.

`rosdep`은 `package.xml`에 명시된 의존성을 분석하여 누락된 의존성 패키지를 자동으로 설치해주는 도구입니다.

### Ament Python 패키지

[Ament Python](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)은 순수 Python으로 작성된 ROS 2 패키지에 사용되는 빌드 타입입니다. 다음 명령으로 패키지 템플릿을 생성할 수 있습니다:

```bash
ros2 pkg create --build-type ament_python
```

Ament Python 패키지의 기본 디렉토리 구조는 다음과 같습니다:

```
pkg_name/
├── package.xml             # 패키지 매니페스트 파일
├── pkg_name/               # Python 패키지
│   ├── __init__.py         # 이 디렉토리가 Python 패키지임을 나타냄
│   └── module.py           # Python 모듈
├── launch/                 # 런치 파일 디렉토리
│   ├── file.launch.xml     # XML 형식의 런치 파일
│   ├── file.launch.py      # Python 형식의 런치 파일
│   └── file.launch.yaml    # YAML 형식의 런치 파일
├── config/                 # 설정 파일 디렉토리
│   ├── parameters.yaml     # 노드의 파라미터를 저장하는 파일
│   └── view.rviz           # RViz 설정 파일
├── resource/               # ament_index에 패키지를 등록하기 위한 디렉토리
│   └── package_name        # ament_index에 패키지를 등록하기 위한 빈 파일
├── setup.cfg               # 설치 경로 등을 설정하는 파일
├── setup.py                # 노드 진입점, 메타데이터, 기타 설치 파일 등을 정의
└── test/                   # 단위 테스트 디렉토리
    ├── test_copyright.py   # 저작권 표기를 검사하는 테스트
    ├── test_flake8.py      # Python 코드 스타일을 검사하는 테스트
    └── test_pep257.py      # Docstring 스타일을 검사하는 테스트
```

#### setup.py

`setup.py` 파일은 Python 패키지 도구인 `setuptools`를 기반으로 합니다. 이 파일에는 패키지의 메타데이터와 설치할 파일, 노드의 진입점 등을 정의합니다.

- `data_files`: 설치해야 할 비코드 파일들을 지정합니다. `[('install_dir1', ['file1', 'file2', ...]), ('install_dir2', ['file1', ...])]`와 같은 형태의 리스트로 표현됩니다.
- `entry_points`: 다양한 진입점을 정의하는 딕셔너리입니다. 노드는 `console_scripts` 리스트에 `'node_exec = pkg.module:func'` 형태로 등록됩니다.

#### setup.cfg

`setup.py`에서 사용할 기본 설치 경로를 지정합니다. 보통은 그대로 두면 됩니다.

### Ament CMake 패키지

[Ament CMake](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) 패키지는 주로 C++로 작성된 ROS 프로젝트에 사용됩니다. 다음 명령으로 패키지 템플릿을 생성할 수 있습니다:

```bash
ros2 pkg create --build-type ament_cmake
```

현재는 커스텀 인터페이스(메시지, 서비스, 액션 등)를 정의하려면 반드시 Ament CMake 패키지를 사용해야 합니다.

Ament CMake 패키지의 기본 디렉토리 구조는 다음과 같습니다:

```
pkg_name/
├── package.xml           # 패키지 매니페스트 파일
├── CMakeLists.txt        # CMake 빌드 설정 파일 (ament_* 매크로 사용)
├── msg/                  # 커스텀 메시지 타입 디렉토리
│   └── MessageType.msg   # 메시지 타입을 정의하는 ROS IDL 파일
├── srv/                  # 커스텀 서비스 타입 디렉토리
│   └── ServiceType.srv   # 서비스 타입을 정의하는 ROS IDL 파일
└── action/               # 커스텀 액션 타입 디렉토리
    └── ActionType.action # 액션 타입을 정의하는 ROS IDL 파일
```

## 커스텀 인터페이스 (Custom Interfaces)

### 커스텀 인터페이스 정의하기

1. 커스텀 인터페이스는 `ament_cmake` 타입의 패키지에서 만듭니다.
2. [ROS IDL](https://docs.ros.org/en/iron/Concepts/About-ROS-Interfaces.html)을 사용하여 인터페이스 파일을 작성합니다.
   - 메시지 파일은 `.msg` 확장자를 가지며, `<package_name>/msg` 디렉토리에 위치합니다.
   - 서비스 파일은 `.srv` 확장자를 가지며, `<package_name>/srv` 디렉토리에 위치합니다.
   - 액션 파일은 `.action` 확장자를 가지며, `<package_name>/action` 디렉토리에 위치합니다.
3. `CMakeLists.txt` 파일을 수정하여 인터페이스 코드 생성을 위한 CMake 코드를 추가합니다.
4. `package.xml` 파일을 수정합니다.
   - `rosidl_default_generators`에 대한 `<buildtool_depend>`를 추가합니다.
   - `rosidl_default_runtime`에 대한 `<exec_depend>`를 추가합니다.
   - `<member_of_group>` 태그를 사용하여 패키지를 `rosidl_interface_packages` 그룹에 추가합니다.
   - 사용한 타입이 정의된 패키지에 대한 `<exec_depend>`와 `<build_depend>`를 추가합니다.
5. 커스텀 인터페이스의 이름은 대문자로 시작해야 하며, 영문자와 숫자만 포함할 수 있습니다.

### 인터페이스 사용하기

1. 인터페이스를 정의한 패키지에 대한 `<exec_depend>`를 추가합니다.
2. Python에서는 `from <interface_package_name>.<msg|srv|action> import TypeName` 형태로 임포트합니다.
   - `<interface_package_name>`은 인터페이스를 정의한 패키지의 이름입니다.
   - `<msg|srv|action>`은 `msg`, `srv`, `action` 중 하나입니다.
   - `TypeName`은 메시지, 서비스, 액션 타입의 이름입니다.

## 워크스페이스 (Workspace)

### 구조

ROS [워크스페이스](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)는 `ws`라는 디렉토리로, 다음과 같은 구조를 가집니다:

1. `ws/src/<repo>`: ROS 패키지의 소스 코드가 위치하는 디렉토리입니다.
   - `package.xml` 파일을 포함하는 모든 디렉토리는 패키지로 간주됩니다.
   - 관례상 패키지는 `ws/src/<repo>` 아래에 위치하며, `<repo>`는 소스 코드 저장소의 이름입니다.
   - 하나의 저장소에는 여러 개의 패키지가 포함될 수 있습니다.
2. `ws/log`: 패키지 빌드 시 생성되는 로그 파일이 저장되는 디렉토리입니다. 최신 로그는 `ws/log/latest`에 위치합니다.
3. `ws/install`: 빌드된 패키지가 설치되는 디렉토리입니다.
4. ROS 워크스페이스를 사용하면 관련 패키지들을 묶어서 빌드할 수 있습니다.
5. 프로젝트마다 새로운 워크스페이스를 사용하는 것이 좋습니다. 이렇게 하면 현재 개발 중인 패키지와 관련 없는 문제로 인해 영향을 받지 않습니다.

### 워크스페이스 빌드하기

- 워크스페이스는 사용하기 전에 반드시 [빌드](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#build-the-workspace-with-colcon)해야 합니다.
- 워크스페이스 디렉토리(`ws`)에서 `colcon build` 명령을 실행하면 `ws/src` 아래의 모든 패키지가 빌드됩니다.
  - 빌드 결과물은 `ws/build`에 저장됩니다.
  - 최종 결과물은 `ws/install`에 설치되며, 실제 코드(Python 파일 포함)가 실행되는 위치입니다.
  - 따라서 워크스페이스를 사용하기 전에 반드시 `colcon build`를 최소한 한 번은 실행해야 합니다.
- `colcon build`의 주요 옵션은 다음과 같습니다:
  - `--symlink-install`: Python 파일을 수정한 후에도 `colcon build`를 다시 실행하지 않아도 되도록 합니다.
  - `--merge-install`: 기본적으로 각 패키지는 자체 디렉토리 구조로 설치됩니다. 이로 인해 경로가 매우 길어질 수 있으므로, 이 옵션을 사용하면 모든 패키지의 파일이 유형별로 공통 디렉토리에 설치됩니다.

#### 경고

- 현재 많은 ROS 2 패키지에서 `setuptools` 사용과 관련된 비권장 경고(deprecation warning)가 발생합니다.
- 이러한 경고를 보지 않으려면 `~/.bashrc`에 `export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources`를 추가하세요.
- 자세한 내용은 [colcon-core 이슈 #454](https://github.com/colcon/colcon-core/issues/454)를 참고하세요.
- 경고를 방치하는 것은 좋지 않은 습관입니다.
  - 경고가 문제가 되지 않는 이유를 신중히 고려한 후에만 경고를 억제해야 합니다 (이 경우에는 우리가 제어할 수 없는 경고입니다).
  - 일부 경고를 방치하면 모든 경고를 무시하는 상황이 발생할 수 있으며, 이는 종종 버그로 이어집니다.

### 워크스페이스 사용하기

- 빌드 후에는 `ws/install` 디렉토리(설치 공간)가 생성됩니다.
  - 여기에는 Python 파일과 기타 파일(설치되도록 `setup.py`에 지정한 경우)이 포함됩니다.
- 빌드 후에는 ROS에서 이 파일들을 사용할 수 있도록 설치 공간을 소싱(sourcing)해야 합니다.
- 워크스페이스를 소싱하려면 `source <path_to_install_space>/setup.bash` 명령을 실행합니다.
  - 예를 들어 현재 디렉토리가 워크스페이스의 루트(`ws/`)인 경우, `source install/setup.bash`를 실행합니다.
  - 기술적으로는 관련 워크스페이스를 소싱한 터미널 창에서 `colcon build`를 실행하면 안 됩니다.

## Colcon

- [`colcon`](https://colcon.readthedocs.io/en/released/)은 ROS 2의 빌드 도구로, 워크스페이스를 빌드하는 데 사용됩니다.
  - `python`으로 작성되었으며, 누구나 빌드 과정을 커스터마이즈할 수 있는 확장 기능으로 구현되어 있습니다.
  - 일반적인 확장 기능은 `python3-colcon-common-extensions` 패키지의 일부로 제공됩니다.
  - 다른 확장 기능은 보통 `python3-colcon-<extension-name>` 형태로 설치할 수 있습니다.
  - 누구나 colcon 확장 기능을 작성할 수 있으며, colcon 개발자가 이를 지원하거나 호스팅할 필요는 없습니다. 그러나 일반적으로 유용한 colcon 확장 기능의 소스 코드는 [Colcon Github Organization](https://github.com/colcon)에서 호스팅되며 유지 관리됩니다.
- 빌드 도구로서 colcon은 다양한 빌드 시스템(ament_python, ament_cmake, CMake, Catkin 등)을 사용하는 프로젝트를 빌드할 수 있습니다 ([Colcon 설계 근거](https://design.ros2.org/articles/build_tool.html) 참조).
- `colcon`은 서로 다른 프로그래밍 언어와 빌드 시스템을 사용하는 여러 ROS 패키지 간의 의존성을 관리합니다.
  - `package.xml`에 명시된 의존성을 기반으로 `colcon`이 패키지를 올바른 순서로 빌드합니다.
  - `<package A>`가 `<package B>`에 대한 `build_depend`를 가지고 있다면, `colcon`은 항상 `<package A>`보다 먼저 `<package B>`를 빌드합니다.
- 기본적으로 `colcon`은 패키지를 병렬로 빌드합니다.
  - 의존성이 간접적으로 명시되어 있더라도 운이 좋으면 코드가 여전히 컴파일될 수 있습니다. 예를 들어 `<package A>`가 `<package B>`에 의존하지만 의존성이 명시되어 있지 않은 경우, `colcon`이 두 패키지를 동시에 빌드하려고 시도할 수 있습니다. 이때 `<package B>`가 `<package A>` 프로세스에서 필요로 하기 전에 완료되면 빌드가 성공합니다.
  - 의존성이 항상 올바르게 빌드되는지 확인하려면 `colcon build --executor sequential` 옵션을 사용하여 각 패키지를 순차적으로 빌드하세요.
- [ROS 2 빌드 시스템에 대한 자세한 정보](https://docs.ros.org/en/iron/Concepts/About-Build-System.html)

### colcon_cd

- [colcon_cd](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes)를 사용하면 워크스페이스 디렉토리와 패키지 디렉토리 간에 빠르게 전환할 수 있습니다.
- `sudo apt install python3-colcon-cd` 명령으로 설치합니다.
- `source /usr/share/colcon_cd/function/colcon_cd.sh`를 추가합니다.
- 워크스페이스 디렉토리에서 `colcon_cd <package>`를 실행하면 해당 패키지로 이동합니다.
- `colcon_cd` 명령을 실행하면 워크스페이스 디렉토리로 돌아갑니다.

### colcon_clean

- [colcon-clean](https://github.com/colcon/colcon-clean)을 사용하면 워크스페이스에서 빌드 결과를 쉽게 제거할 수 있습니다.
- `sudo apt install python3-colcon-clean` 명령으로 설치합니다.
- `colcon clean workspace`를 실행하면 생성된 모든 파일이 정리됩니다.
- `colcon clean packages`를 사용하면 정리할 개별 패키지를 선택할 수 있습니다.

### colcon.pkg

1. `colcon.pkg` 파일은 패키지 디렉토리의 루트에 위치하며, 패키지 빌드 시 `colcon` 설정을 변경할 수 있게 해줍니다.
2. 이 파일의 한 가지 용도는 환경 훅(environment hook)을 설치하는 것입니다. 환경 훅은 `install/setup.bash`가 소싱될 때 환경 변수를 설정하는 파일입니다.
3. 환경 훅을 설정하려면 다음 단계를 따르세요:
   1. 패키지의 `env-hooks` 하위 디렉토리에 [dsv](https://colcon.readthedocs.io/en/released/developer/environment.html) 파일을 생성합니다.
      - 예를 들어 `prepend-non-duplicate;VAR_NAME;VAR_VALUE`는 중복되지 않는 경우 `VAR_NAME` 환경 변수 앞에 `VAR_VALUE`를 추가합니다.
   2. `colcon.pkg`에 훅을 추가합니다:
      ```json
      {
        "hooks": ["share/<PACKAGE_NAME>/env-hooks/<DSV_NAME>.dsv"]
      }
      ```
   3. 위 코드는 지정된 경로에 있는 훅을 추가합니다.
   4. `setup.py`에서 `.dsv` 파일이 `colcon.pkg`에서 참조한 경로에 설치되도록 해야 합니다.

## ROS 환경

- ROS는 설정을 제어하고 노드와 라이브러리를 찾기 위해 [환경 변수](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)에 의존합니다.
  - 환경 변수에 대한 자세한 내용은 [환경 변수](https://nu-msr.github.io/ros_notes/ros2/colcon.html../../hackathon/linux_interactive.html#org876ea6d)를 참조하세요.
- ROS 환경 변수는 언더레이(underlay)가 소싱될 때 설정됩니다.
  - 언더레이를 소싱하려면 `source /opt/ros/iron/setup.bash`를 실행합니다.
  - `ROS`를 설치할 때 `bash`가 열릴 때마다 위 명령이 자동으로 실행되도록 `~/.bashrc`에 추가했습니다.
  - ROS 명령행 도구와 시스템에 설치된 패키지에 접근하려면 언더레이를 소싱해야 합니다.
- 오버레이(overlay)를 소싱하여 다른 ROS 워크스페이스를 추가할 수 있습니다. 오버레이를 소싱하면 해당 오버레이에 설치된 패키지에 접근할 수 있습니다.
  - 워크스페이스를 빌드한 후 `source install/setup.bash`를 실행하면 오버레이를 추가하여 방금 빌드한 패키지에 접근할 수 있습니다.
- ROS 환경 변수를 보려면 `env | grep "AMENT\|CMAKE\|ROS\|COLCON"` 명령을 실행하세요.
  - 이 명령은 모든 환경 변수를 출력하고 `AMENT`, `CMAKE`, `ROS`를 포함하는 변수를 검색합니다.
  - `AMENT_PREFIX_PATH`는 런타임에 ROS가 패키지와 파일을 찾는 위치입니다.
  - `ROS_AUTOMATIC_DISCOVERY_RANGE`는 ROS 노드가 네트워크에서 자동으로 서로를 발견하는 위치에 대한 설정을 제공합니다.
    - 기본적으로 이 설정은 `SUBNET`으로, 동일한 서브넷에서 실행되는 모든 노드가 서로를 볼 수 있음을 의미합니다. 그러나 Northwestern의 네트워크는 자동 노드 검색에 필요한 트래픽을 차단합니다.

### 오버레이 (Overlays)

- 기술적으로는 `colcon build`를 사용할 때 오버레이를 소싱하면 안 됩니다. 이는 빌드와 ROS 명령을 실행하기 위해 별도의 창이 필요하다는 것을 의미합니다.
  - 빌드할 때 오버레이를 소싱하면 이전 빌드에서 빌드된 패키지를 사용할 수 있게 되어 의존성 문제를 야기하거나 숨길 수 있습니다.
  - 실제 일상적인 사용에서는 이 문제가 거의 중요하지 않지만, 이렇게 하는 것이 좋은 습관이며, 그렇지 않으면 언젠가는 많은 시간을 낭비하게 될 수 있습니다.
- 여러 ROS 워크스페이스를 서로 오버레이할 수 있으므로 여러 워크스페이스의 패키지를 사용하거나 특정 패키지를 오버라이드할 수 있습니다.
  - 일반적인 사용 사례는 소스 코드로만 제공되는 외부 패키지(예: interbotix 패키지)를 사용하는 경우입니다.
  - 또 다른 사용 사례는 시스템에 설치된 패키지를 제거하지 않고도 해당 패키지를 개발할 수 있도록 하는 것입니다.
- `install/setup.bash` 파일은 `colcon build`가 실행될 때 생성되며, 현재 ROS 환경을 캡처하고 여기에 현재 워크스페이스를 추가합니다.
  - 즉, `source install/setup.bash`는 빌드 당시의 환경에 현재 워크스페이스를 더한 환경을 설정합니다.
  - `install/local_setup.bash`는 빌드 당시의 환경과 관계없이 현재 워크스페이스를 기존 ROS 환경에 오버레이합니다.

이상으로 ROS의 패키지와 워크스페이스에 대해 알아보았습니다. 패키지는 ROS 코드와 리소스의 기본 단위이며, 워크스페이스는 관련 패키지들을 묶어 빌드하고 관리하는 데 사용됩니다.

ROS 패키지는 `package.xml` 매니페스트 파일에 의해 정의되며, 빌드 타입에 따라 `ament_python`, `ament_cmake`, `ament_cmake_python` 등으로 분류됩니다. 패키지는 `apt`를 통해 바이너리 형태로 설치하거나, 소스 코드를 직접 빌드할 수도 있습니다.

워크스페이스는 `src`, `build`, `install`, `log` 등의 하위 디렉토리로 구성되며, `colcon` 빌드 도구를 사용하여 빌드합니다. 빌드된 패키지는 `install` 디렉토리에 설치되며, `setup.bash`를 소싱하여 사용할 수 있습니다.

ROS는 다양한 환경 변수를 사용하여 설정을 제어하고 패키지와 노드를 찾습니다. 언더레이와 오버레이를 통해 여러 워크스페이스를 조합하여 사용할 수 있습니다.

이제 ROS 패키지와 워크스페이스의 구조와 작동 방식을 이해했으니, 실제로 패키지를 만들고 빌드하면서 ROS 개발 환경에 익숙해져 보시기 바랍니다. ROS의 강력한 도구와 생태계를 활용하여 멋진 로봇 애플리케이션을 만들어보세요!
