# 202021024 - 로봇팔을 이용한 덤벨 및 원판 정리

## 프로젝트 개요

이 프로젝트의 목표는 ROS2를 이용하여 헬스장에 정리가 안된 덤벨이나, 원판들을 인식하고 있어야 하는 위치로 자동으로 정리해주는 것이 목표이다.
로봇은 덤벨과 같은 물체를 인식하고, 무게를 측정하며 지정된 위치로 이동시키는 역활을 수행한다.

## 프로젝트 목표

- 카메라를 이용해 옮겨도 되는 물건인지 판별하는 시스템 구현
- 카메라를 이용해 덤벨과 원판과 같은 물건들을 구별하고, 지정된 위치로 옮기는 작업을 구현
- 로봇팔을 이용한 무게측정을 최대한 분별력 있는 방법으로 구현
- 지정된 위치가 포화 상태일 시 다른 적합한 위치로 정리

## 방법론

- ROS2를 이용한 로봇 프로그레밍
- 카메라와 같은 센서를 사용하여 물체 인식
- 지정된 위치에 알맞은 물체를 구분하기 위한 센서 통합
- 관리 및 모니터링을 위한 애플리케이션 개발

## 필요 자원

- 하드웨어: 두산로보틱스사의 로봇팔, 카메라
- 소프트웨어: ROS2, OpenCV 컴퓨터 비전 라이브러리

## 프로젝트 참여자

- 학번: 202021024
- 이름: 윤효찬
- 이메일: 202021024@chu.ac.kr

## 예상 예산

| 품목        | 가격(원)   |
| ----------- | ---------- |
| 웹캠(1개)   | 29,100     |
| **총 예산** | **29,100** |

## 예상 일정

| 주차   | 활동                                  |
| ------ | ------------------------------------- |
| 주 1-2 | 프로젝트 계획 및 초기 설계            |
| 주 3-4 | 프로그래밍 및 초기 테스트             |
| 주 5-6 | 물체를 인식 및 분류를 위한 센서 통합  |
| 주 7   | 사용자 인터페이스 개발 및 변수 테스트 |
| 주 8   | 최종 발표, 프로젝트 마무리 및 문서화  |

## 위험 관리

- 부족한 하드웨어 및 소프트웨에 관한 필요한 정보들을 습득하고, 각 한계점들을 파악하여 위험을 최소화

## 숙제

- ![스크린샷 2024-05-28 094325](https://github.com/chu-aie/cobots-2024/assets/162118894/c26b0bbf-e9c1-4c2c-8332-fafa15bf0b78)

## 기말 과제

터틀심을 이용한 땅따먹기 게임

- 팬더로봇은 구동이 안되고 있고, 전 숙제를 통하여 터틀심이 정상적으로 작동하는 것을 확인하였다. 그래서 터틀심을 이용하여 땅따먹기 게임을 제작해보기로 했다.
- 땅따먹기 게임 코드

  #!/usr/bin/env python3

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import tty
import termios
import threading

class TurtleGame(Node):

    def __init__(self, turtle_name):
        super().__init__('turtle_game_' + turtle_name)
        self.publisher_ = self.create_publisher(Twist, '/' + turtle_name + '/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/' + turtle_name + '/pose', self.pose_callback, 10)
        self.current_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_turtle(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.publisher_.publish(msg)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def player1_control(turtle_game):
    while True:
        key = getch()
        if key == '\x1b[A':  # Up arrow
            turtle_game.move_turtle(2.0, 0.0)
        elif key == '\x1b[B':  # Down arrow
            turtle_game.move_turtle(-2.0, 0.0)
        elif key == '\x1b[C':  # Right arrow
            turtle_game.move_turtle(0.0, -1.5)
        elif key == '\x1b[D':  # Left arrow
            turtle_game.move_turtle(0.0, 1.5)

def player2_control(turtle_game):
    while True:
        key = getch()
        if key == 'w' or key == 'W':
            turtle_game.move_turtle(2.0, 0.0)
        elif key == 's' or key == 'S':
            turtle_game.move_turtle(-2.0, 0.0)
        elif key == 'd' or key == 'D':
            turtle_game.move_turtle(0.0, -1.5)
        elif key == 'a' or key == 'A':
            turtle_game.move_turtle(0.0, 1.5)

def main(args=None):
    rclpy.init(args=args)

    turtle1_game = TurtleGame('turtle1')
    turtle2_game = TurtleGame('turtle2')

    # Create threads for each player's control
    thread_player1 = threading.Thread(target=player1_control, args=(turtle1_game,))
    thread_player2 = threading.Thread(target=player2_control, args=(turtle2_game,))

    thread_player1.start()
    thread_player2.start()

    try:
        rclpy.spin(turtle1_game)
        rclpy.spin(turtle2_game)
    except KeyboardInterrupt:
        pass
    finally:
        thread_player1.join()
        thread_player2.join()

        turtle1_game.destroy_node()
        turtle2_game.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- turtle_game_node.py 파일을 생성하고 코드를 안에 저장 후 구동을 하려했지만 오류가 계속 발생하여 구동하지 못하였다.

```sh
  cd ~/ros2_ws
  colcon build --packages-select my_turtle_game
  source install/setup.bash
  ros2 run my_turtle_game turtle_game_node

  Starting >>> my_turtle_game
  Finished <<< my_turtle_game [0.39s]

  Summary: 1 package finished [0.52s]
  not found: "/home/yhc/ros2_ws/local_setup.bash"
  Package 'my_turtle_game' not found
```

- ros2 버전 확인과 파일명 확인 등 여러 방면으로 해결해보려 했으나 구동에 실패했다.
