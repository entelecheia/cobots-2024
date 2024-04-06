# 도커를 이용한 ROS2 실행환경 구축

여기서는 도커를 활용하여 ROS2 실행환경을 구축하는 방법에 대해 설명하겠습니다. [dotfiles](https://dotfiles.entelecheia.ai/)와 [ros2-container](https://github.com/entelecheia/ros2-container) 프로젝트를 이용하여 개발 환경을 설정하고, MobaXterm(Windows)과 XQuartz(macOS)를 사용하여 SSH와 X11 Forwarding을 통해 원격 컴퓨터에서 GUI 애플리케이션을 실행하는 방법도 알아보겠습니다.

## dotfiles를 이용한 ROS2 개발 환경 설정

[Dotfiles 프로젝트](https://dotfiles.entelecheia.ai/)는 개발 환경을 일관되게 유지하고 관리하는데 도움이 되는 도구입니다. 다음 단계를 따라 dotfiles를 설치하고 초기화할 수 있습니다.

### dotfiles 설치

`wget`이나 `curl` 명령어를 사용하여 설치 스크립트를 다운로드하고 실행합니다.

- `wget` 사용:

  ```sh
  sh -c "$(wget -qO- https://dotfiles.entelecheia.ai/install)"
  ```

- `curl` 사용:
  ```sh
  sh -c "$(curl -fsSL https://dotfiles.entelecheia.ai/install)"
  ```

설치 스크립트가 실행되면 dotfiles 저장소가 여러분의 컴퓨터에 복제되고 필요한 설정이 이루어집니다.

### dotfiles 초기화

설치가 완료되면 자동으로 초기화 과정이 시작됩니다. 이 과정에서 dotfiles에 정의된 설정이 시스템에 적용되어 일관된 개발 환경을 구성합니다.

만약 수동으로 다시 초기화해야 한다면 다음 명령어를 실행하세요:

```sh
dotu init
```

이 명령어는 dotfiles의 최신 설정으로 시스템을 업데이트하여 변경사항과 환경을 동기화된 상태로 유지합니다.

## ros2-container를 이용한 ROS2 실행환경 구축

[ros2-container](https://github.com/entelecheia/ros2-container) 프로젝트는 도커를 이용하여 ROS2 실행환경을 구축하는데 도움을 줍니다. 다음 단계를 따라 프로젝트를 클론하고 도커 이미지를 빌드한 후 컨테이너를 실행할 수 있습니다.

### ros2-container 레포지토리 클론

```bash
cd ~/workspace/projects
git clone https://github.com/entelecheia/ros2-container.git
```

### 도커 이미지 빌드 및 실행

1. 도커 이미지 빌드:

   ```bash
   make docker-build
   ```

   `docker.foxy-dsr.env` 파일에는 다양한 설정 옵션과 환경 변수가 포함되어 있습니다. `docker-compose.foxy-dsr.yaml` 파일은 이러한 변수를 사용하여 서비스의 동작을 커스터마이징합니다. 이는 개발, 테스트, 프로덕션 환경에 대해 서로 다른 설정을 지정하려는 경우에 일반적으로 사용되는 방법입니다. `Dockerfile.foxy-dsr` 파일은 도커 빌드 과정에서 이러한 변수를 사용하여 설정을 조정합니다. 이 파일들은 `copier copy` 명령을 실행할 때 Copier에 의해 자동으로 생성됩니다.

2. 도커 컨테이너 실행:

   ```bash
   make docker-run
   ```

   이 명령어는 이전 단계에서 빌드한 이미지를 사용하여 도커 컨테이너를 시작합니다. 컨테이너 내에서는 애플리케이션을 시작하는 bash 스크립트가 실행됩니다.

## Windows에서 MobaXterm을 이용한 SSH와 X11 Forwarding

[MobaXterm](https://mobaxterm.mobatek.net/download.html)은 다양한 기능을 갖춘 멋진 터미널 에뮬레이터입니다. Home Edition은 무료로 사용할 수 있습니다. MobaXterm은 다운로드 가능합니다.

설치 후 MobaXterm을 실행하세요. 처음 실행 시 약간 느릴 수 있습니다. 방화벽 액세스에 대한 프롬프트가 표시되면 액세스 허용을 선택하세요.

Session을 선택하여 새 세션을 시작하세요.

MobaXterm 창에서 SSH를 선택한 다음 연결하려는 컴퓨터를 Remote host 필드에 입력하세요. Specify username 체크박스를 선택하고 사용자 이름을 입력할 수도 있습니다. 포트는 22로 설정된 상태로 두세요.

Advanced SSH settings 탭을 선택한 다음 X11-Forwarding 체크박스가 선택되어 있는지 확인하세요. 나머지 설정은 기본값으로 둡니다. OK를 선택하세요.

MobaXterm은 탭에서 새 SSH 세션을 시작하고 암호를 입력하라는 메시지를 표시합니다. 인증이 완료되면 MobaXterm은 세션에 대한 정보와 연결된 컴퓨터의 정보를 표시합니다. 이제 명령을 실행하고 프로그램을 정상적으로 열 수 있습니다. MobaXterm은 X 세션에서 실행 중임을 나타내기 위해 팝업되는 모든 창의 제목 표시줄에 "@computername"을 추가합니다.

여기에서 저장된 사용자 세션을 빠르게 사용하여 다시 연결할 수 있습니다. 왼쪽의 탭에는 유용한 도구, 게임 등이 있습니다. SFTP 탭에는 SFTP 파일 브라우저도 있어 Michigan Tech 홈 드라이브와 MobaXterm을 실행하는 컴퓨터 간에 빠르고 쉽게 파일을 전송할 수 있습니다.

이제 MobaXterm을 사용하여 SSH와 X11 Forwarding을 통해 원격 컴퓨터에서 ROS2 GUI 애플리케이션을 실행할 수 있습니다. 다음 단계를 따라 MobaXterm에서 도커 컨테이너에 연결하고 X11 Forwarding을 사용하여 GUI 애플리케이션을 실행해 보세요.

1. MobaXterm에서 새 세션 시작:
   MobaXterm을 열고 Session을 선택하여 새 세션을 시작하세요. SSH를 선택하고 도커 컨테이너가 실행 중인 원격 호스트의 IP 주소나 호스트명을 입력하세요. 포트는 22로 설정된 상태로 두세요.

2. X11 Forwarding 설정:
   Advanced SSH settings 탭을 선택하고 X11-Forwarding 체크박스를 선택하세요. 나머지 설정은 기본값으로 둡니다. OK를 선택하세요.

3. SSH 세션 시작:
   MobaXterm이 새 SSH 세션을 시작하고 암호를 입력하라는 메시지를 표시합니다. 인증이 완료되면 원격 호스트의 터미널에 연결됩니다.

4. 도커 컨테이너 실행:
   SSH 세션에서 다음 명령어를 실행하여 도커 컨테이너를 시작하세요:

   ```bash
   docker run -it --rm \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     ghcr.io/entelecheia/ros2:latest-foxy-dsr \
     bash
   ```

   이 명령어는 DISPLAY 환경 변수와 X11 소켓을 매핑하여 도커 컨테이너를 실행합니다.

5. GUI 애플리케이션 실행:
   도커 컨테이너 내에서 GUI 애플리케이션을 실행하세요. 예를 들어, RViz를 실행하려면 다음 명령어를 입력하세요:

   ```bash
   rviz2
   ```

   RViz 창이 MobaXterm에 나타납니다. 제목 표시줄에는 "@computername"이 추가되어 X 세션에서 실행 중임을 나타냅니다.

## macOS에서 XQuartz를 이용한 SSH와 X11 Forwarding

macOS에서 원격 도커 컨테이너의 X11 Forwarding을 사용하려면 XQuartz를 설치하고 설정해야 합니다. 다음 단계를 따라 XQuartz를 설치하고 SSH와 X11 Forwarding을 사용하여 ROS2 GUI 애플리케이션을 실행해 보세요.

1. XQuartz 설치:
   XQuartz는 macOS를 위한 X11 서버를 제공합니다. https://www.xquartz.org/ 에서 XQuartz를 다운로드하고 설치하세요.

   ```bash
   brew install --cask xquartz
   ```

2. XQuartz 실행:
   설치 후 Applications 폴더에서 XQuartz를 실행하세요.

3. 네트워크 연결 허용:
   XQuartz 환경설정에서 Security 탭으로 이동하여 "Allow connections from network clients" 옵션을 체크하세요.

4. xhost 명령어 실행:
   터미널을 열고 다음 명령어를 실행하여 모든 호스트에서 X 서버로의 연결을 허용하세요:

   ```bash
   xhost +
   ```

   특정 IP 주소에서의 연결만 허용하려면 다음과 같이 실행하세요:

   ```bash
   xhost + [IP_ADDRESS]
   ```

5. SSH 세션 시작:
   터미널에서 다음 명령어를 실행하여 원격 호스트에 SSH로 연결하세요:

   ```bash
   ssh -X username@remote_host
   ```

   `-X` 옵션은 X11 Forwarding을 활성화합니다. `username`은 원격 호스트의 사용자 이름이고, `remote_host`는 원격 호스트의 IP 주소나 호스트명입니다.

6. 도커 컨테이너 실행:
   SSH 세션에서 다음 명령어를 실행하여 도커 컨테이너를 시작하세요:

   ```bash
   docker run -it --rm \
     -e DISPLAY=host.docker.internal:0 \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     ghcr.io/entelecheia/ros2:latest-foxy-dsr \
     bash
   ```

   `-e DISPLAY=host.docker.internal:0`는 컨테이너 내부의 DISPLAY 환경 변수가 호스트의 X11 서버를 가리키도록 설정합니다. `-v /tmp/.X11-unix:/tmp/.X11-unix`는 호스트의 X11 유닉스 소켓을 컨테이너에 매핑합니다.

7. GUI 애플리케이션 실행:
   도커 컨테이너 내에서 GUI 애플리케이션을 실행하세요. 예를 들어, RViz를 실행하려면 다음 명령어를 입력하세요:

   ```bash
   rviz2
   ```

   RViz 창이 macOS에 나타납니다.

이제 XQuartz를 사용하여 SSH와 X11 Forwarding을 통해 macOS에서 원격 도커 컨테이너의 ROS2 GUI 애플리케이션을 실행할 수 있습니다.

이상으로 도커를 이용한 ROS2 실행환경 구축과 Windows에서 MobaXterm, macOS에서 XQuartz를 사용한 SSH, X11 Forwarding 방법에 대해 알아보았습니다. dotfiles와 ros2-container 프로젝트를 활용하면 일관되고 편리한 개발 환경을 만들 수 있습니다. 이제 여러분만의 ROS2 프로젝트를 시작해 보세요!
