# ROS 2 환경 구성하기

## 배경

ROS 2는 쉘 환경을 사용하여 작업 공간을 결합하는 개념에 의존합니다. "작업 공간"은 ROS 2로 개발하는 시스템의 위치를 나타내는 ROS 용어입니다. 핵심 ROS 2 작업 공간을 언더레이(underlay)라고 합니다. 후속 로컬 작업 공간을 오버레이(overlay)라고 합니다. ROS 2로 개발할 때 일반적으로 여러 작업 공간이 동시에 활성화됩니다.

작업 공간을 결합하면 ROS 2의 다른 버전 또는 다른 패키지 세트에 대해 더 쉽게 개발할 수 있습니다. 또한 동일한 컴퓨터에 여러 ROS 2 배포판(예: Dashing과 Eloquent)을 설치하고 이들 간에 전환할 수 있습니다.

이는 새 쉘을 열 때마다 설정 파일을 소싱하거나 한 번 쉘 시작 스크립트에 소스 명령을 추가하여 수행됩니다. 설정 파일을 소싱하지 않으면 ROS 2 명령에 액세스하거나 ROS 2 패키지를 찾거나 사용할 수 없습니다. 즉, ROS 2를 사용할 수 없게 됩니다.

## 전제 조건

이 튜토리얼을 시작하기 전에 ROS 2 설치 페이지의 지침에 따라 ROS 2를 설치하세요.

이 튜토리얼에서 사용되는 명령은 운영 체제용 바이너리 패키지 설치 가이드(Linux용 Debian 패키지)를 따랐다고 가정합니다. 소스에서 빌드한 경우에도 계속 따라할 수 있지만 설정 파일 경로가 다를 수 있습니다. 또한 소스에서 설치하는 경우 `sudo apt install ros-<distro>-<package>` 명령(초급 튜토리얼에서 자주 사용됨)을 사용할 수 없습니다.

Linux 또는 macOS를 사용 중이지만 아직 쉘에 익숙하지 않은 경우 이 튜토리얼이 도움이 될 것입니다.

## 작업

### 1. 설정 파일 소싱하기

새로 여는 모든 쉘에서 ROS 2 명령에 액세스하려면 다음과 같이 이 명령을 실행해야 합니다:

````{tab} Linux
```bash
# Bash를 사용하지 않는 경우 ".bash"를 사용 중인 쉘로 바꾸세요
# 가능한 값: setup.bash, setup.sh, setup.zsh
source /opt/ros/{DISTRO}/setup.bash
```
````

````{tab} macOS
```console
. ~/ros2_install/ros2-osx/setup.bash
```
````

````{tab} Windows
```console
call C:\dev\ros2\local_setup.bat
```
````

정확한 명령은 ROS 2를 설치한 위치에 따라 다릅니다. 문제가 있는 경우 파일 경로가 설치 위치로 이어지는지 확인하세요.

### 2. 쉘 시작 스크립트에 소싱 추가하기

새 쉘을 열 때마다 설정 파일을 소싱하지 않으려면(작업 1 건너뛰기) 쉘 시작 스크립트에 명령을 추가할 수 있습니다:

````{tab} Linux
```console
echo "source /opt/ros/{DISTRO}/setup.bash" >> ~/.bashrc
```

이를 실행 취소하려면 시스템의 쉘 시작 스크립트를 찾아 추가된 소스 명령을 제거하세요.
````

````{tab} macOS
```console
echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile
```

이를 실행 취소하려면 시스템의 쉘 시작 스크립트를 찾아 추가된 소스 명령을 제거하세요.
````

````{tab} Windows
PowerShell 사용자의 경우에만 '내 문서'에 'WindowsPowerShell'이라는 폴더를 만드세요. 'WindowsPowerShell' 내에서 'Microsoft.PowerShell_profile.ps1' 파일을 만드세요. 파일 안에 다음을 붙여넣으세요:

```console
C:\dev\ros2_{DISTRO}\local_setup.ps1
```

PowerShell은 새 쉘이 열릴 때마다 이 스크립트를 실행할 수 있는 권한을 요청합니다. 이 문제를 피하려면 다음을 실행할 수 있습니다:

```console
Unblock-File C:\dev\ros2_{DISTRO}\local_setup.ps1
```

이를 실행 취소하려면 새 'Microsoft.PowerShell_profile.ps1' 파일을 제거하세요.
````

### 3. 환경 변수 확인하기

ROS 2 설정 파일을 소싱하면 ROS 2 작동에 필요한 여러 환경 변수가 설정됩니다. ROS 2 패키지를 찾거나 사용하는 데 문제가 있는 경우 다음 명령을 사용하여 환경이 제대로 설정되었는지 확인하세요:

````{tab} Linux
```console
printenv | grep -i ROS
```
````

````{tab} macOS
```console
printenv | grep -i ROS
```
````

````{tab} Windows
```console
set | findstr -i ROS
```
````

`ROS_DISTRO` 및 `ROS_VERSION`과 같은 변수가 설정되어 있는지 확인하세요.

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO={DISTRO}
```

환경 변수가 올바르게 설정되지 않은 경우 따랐던 설치 가이드의 ROS 2 패키지 설치 섹션으로 돌아가세요. 더 구체적인 도움이 필요한 경우(환경 설정 파일은 다른 곳에서 올 수 있으므로) 커뮤니티에서 답변을 얻을 수 있습니다.

#### 3.1 `ROS_DOMAIN_ID` 변수

ROS 도메인 ID에 대한 자세한 내용은 도메인 ID 문서를 참조하세요.

ROS 2 노드 그룹에 대해 고유한 정수를 결정한 후 다음 명령을 사용하여 환경 변수를 설정할 수 있습니다:

````{tab} Linux
```console
export ROS_DOMAIN_ID=<your_domain_id>
```

쉘 세션 간에 이 설정을 유지하려면 쉘 시작 스크립트에 명령을 추가할 수 있습니다:

```console
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
````

````{tab} macOS
```console
export ROS_DOMAIN_ID=<your_domain_id>
```

쉘 세션 간에 이 설정을 유지하려면 쉘 시작 스크립트에 명령을 추가할 수 있습니다:

```console
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile
```
````

````{tab} Windows
```console
set ROS_DOMAIN_ID=<your_domain_id>
```

쉘 세션 간에 이를 영구적으로 만들려면 다음도 실행하세요:

```console
setx ROS_DOMAIN_ID <your_domain_id>
```
````

#### 3.2 `ROS_AUTOMATIC_DISCOVERY_RANGE` 변수

기본적으로 ROS 2 통신은 로컬호스트로 제한되지 않습니다. `ROS_AUTOMATIC_DISCOVERY_RANGE` 환경 변수를 사용하면 ROS 2 검색 범위를 제한할 수 있습니다. `ROS_AUTOMATIC_DISCOVERY_RANGE`를 사용하면 여러 로봇이 동일한 주제에 게시하여 이상한 동작을 유발할 수 있는 교실과 같은 특정 설정에서 도움이 됩니다. 자세한 내용은 동적 검색 개선을 참조하세요.

## 요약

ROS 2 개발 환경은 사용 전에 올바르게 구성되어야 합니다. 이는 열리는 모든 새 쉘에서 설정 파일을 소싱하거나 시작 스크립트에 소스 명령을 추가하는 두 가지 방법으로 수행할 수 있습니다.

ROS 2로 패키지를 찾거나 사용하는 데 문제가 있는 경우 가장 먼저 해야 할 일은 환경 변수를 확인하고 의도한 버전 및 배포판으로 설정되어 있는지 확인하는 것입니다.
