# ROS 설치환경 구축하기

## WSL에서 VS Code 사용하기

### WSL 설치하기

웹사이트: https://learn.microsoft.com/ko-kr/windows/wsl/install

1. 관리자 모드로 PowerShell을 엽니다.

2. `wsl --install` 명령어를 실행합니다.

3. 재시작합니다.

4. `wsl --list --online` 명령어를 실행하여 사용 가능한 Linux 배포판을 확인합니다.

   - WSL(Windows Subsystem for Linux): Windows에서 Linux 환경을 사용할 수 있게 해주는 기능입니다.

5. Ubuntu-22.04 배포판을 설치합니다: `wsl --install Ubuntu-22.04`

   - Ubuntu: 가장 인기 있는 Linux 배포판 중 하나입니다.

6. `username`과 `password`를 생성합니다.

7. `wsl -l -v` 명령어를 실행하여 버전을 확인합니다. 버전 2가 표시되어야 합니다.

### WSL 확장 설치하기

1. VS Code 확장에서 WSL을 설치합니다.

### VS Code에서 WSL 열기

1. Ubuntu 앱을 엽니다 (`Windows 키 > Ubuntu`).

2. `code .` 명령어를 실행합니다.

3. 탐색기를 오른쪽 클릭하여 폴더 경로를 확인하고 저장합니다.

## WSL에서 GitHub 설정하기

1. 자격 증명 설정하기

```bash
git config --global user.name "사용자이름"
git config --global user.email "이메일@gmail.com"
```

2. SSH 키 생성 및 복사하기
   - SSH(Secure Shell): 네트워크 상의 다른 컴퓨터에 로그인하거나 원격 시스템에서 명령을 실행할 때 사용되는 프로토콜입니다.

```bash
ssh-keygen -t rsa -b 4096 -C "이메일@gmail.com"
cat ~/.ssh/id_rsa.pub
```

3. GitHub의 `설정 > SSH 및 GPG 키 > 새 SSH 키`에 키를 붙여넣습니다.

4. 저장소를 클론하고 시작할 준비가 되었습니다!
