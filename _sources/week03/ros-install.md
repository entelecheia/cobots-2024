# ROS 설치하기

## ROS2 Humble 설치하기

설치 가이드: https://docs.ros.org/en/humble/Installation.html

1. **로케일 설정**
   - 로케일: 언어, 지역, 문자 인코딩 등을 정의하는 설정입니다.

```bash
locale # 현재 로케일 설정 확인
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale # 업데이트된 로케일 설정 확인
```

2. **소프트웨어 속성 및 Universe 저장소 설정**
   - Universe 저장소: Ubuntu에서 커뮤니티 유지 관리 자유-오픈 소스 소프트웨어를 포함하는 저장소입니다.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

3. **Curl 설치 및 ROS 키 설정**
   - Curl: 다양한 통신 프로토콜을 이용하여 데이터를 전송하기 위한 라이브러리와 명령 줄 도구입니다.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. **ROS2 소스 설정**

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. **업데이트 및 업그레이드**

```bash
sudo apt update
sudo apt upgrade
```

6. **ROS 데스크탑 설치**
   - ROS 데스크탑: ROS 개발에 필요한 패키지, 라이브러리, 도구 등을 포함하는 메타 패키지입니다.

```bash
sudo apt install ros-humble-desktop
```

7. **ROS 환경 설정**
   - 이 명령어는 ROS 환경 변수를 설정하여 ROS 명령어를 사용할 수 있게 해줍니다.

```bash
source /opt/ros/humble/setup.bash
```

8. **ROS2 명령어 확인**

```bash
ros2
```

위의 내용은 Ubuntu에서 ROS2 Humble 버전을 설치하는 과정을 자세히 설명한 것입니다. 주요 용어에 대한 설명도 포함되어 있으며, 최신 정보를 바탕으로 작성되었습니다. 이제 ROS2를 사용할 준비가 되었습니다!

```{admonition} 이슈 - ros2 명령어를 찾을 수 없음
`ros2` 명령어를 실행할 때 흔히 발생하는 오류입니다.

**해결 방법: Setup.bash 파일 소싱하기**
  - 소싱(Sourcing): 스크립트 파일의 내용을 현재 셸 환경에 적용하는 것을 의미합니다.

**Bash 셸의 경우:**

1. 명령어 방식: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` 명령어를 실행하여 자동으로 소싱합니다. 이렇게 하면 `ros2` 명령어를 사용할 수 있게 됩니다.
   - ~/.bashrc: Bash 셸 시작 시 자동으로 실행되는 스크립트 파일입니다.

2. 에티터 방식: VS Code에서 `.bashrc` 파일을 열고 `source /opt/ros/humble/setup.bash` 라인을 수동으로 추가합니다.

**Zsh 셸의 경우:**

1. 명령어 방식: `echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc` 명령어를 실행하여 자동으로 소싱합니다. 이렇게 하면 `ros2` 명령어를 사용할 수 있게 됩니다.
   - ~/.zshrc: Zsh 셸 시작 시 자동으로 실행되는 스크립트 파일입니다.

2. 에디터 방식: VS Code에서 `.zshrc` 파일을 열고 `source /opt/ros/humble/setup.bash` 라인을 수동으로 추가합니다.
```
