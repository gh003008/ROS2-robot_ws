# ROS_tutorials

## 실습1(Turtlesim을 활용한 ROS 기초 프로그래밍)

###  실습과제 목표

![rosgraph_turtle_hw_1](https://user-images.githubusercontent.com/88312299/149097045-bfe4751f-4571-43c3-9017-4cce57715ede.png)


위의 그림과 같은 프로그램을 만들어야 한다.
RVD데이터 (토픽)을 받아서 turtlesim 이 움직이도록 하기 위해 RVD를 subscribe 하게 한후 이를 /turtle1/cmd_vel 형태로 바꾸어주고 (Twist)
이를 publish 해줄 pub_sub node를 만든다. 



### 1. msg_srv_action_interface 패키지 생성

먼저 RVD 메세지 타입을 정의하기 위해 인터페이스로만 구성된 패키지인 "msg_srv_action_interface" 을 생성한다.
입력받게 될 rvd 메세지 타입을 새롭게 정의한다.
```
cd ~/robot_ws/src
ros2 pkg create  msg_srv_action_interface --build-type ament_cmake
cd msg_srv_action_interface
mkdir msg srv action
```
이후 [package.xml](msg_srv_action_interface/package.xml), [CMakeLists.txt](msg_srv_action_interface/CMakeLists.txt) 파일을 변경한 뒤, msg 폴더 안에 [RVD.msg](msg_srv_action_interface/msg/RVD.msg) 를 생성한다.


### 2. turtle_hw_1 패키지 생성

등속 원운동을 위한 패키지 "turtle_hw_1" 을 생성한다. 

```
cd ~/robot_ws/src
ros2 pkg create turtle_hw_1 --build-type ament_python --dependencies rclpy std_msgs
cd turtle_hw_1
```
 [package.xml](turtle_hw_1/package.xml), [setup.py](turtle_hw_1/setup.py) 파일을 설정에 따라 수정해 준다.
 turtle_hw_1 폴더 안에 [pub_sub.py](turtle_hw_1/turtle_hw_1/pub_sub.py) 를 생성해 준다. 

```
cd ~/robot_ws/src/turtle_hw_1/turtle_hw_1
touch pub_sub.py
code pub_sub.py
```

(위의 내용을 복붙)

### 3. launch 파일 

turtlesim_node 노드와 pub_sub 노드를 한 번에 실행시키기 위해 "ros2 launch" 를 이용한다.
```
cd ~/robot_ws/src/turtle_hw_1
mkdir launch
cd launch
```
 [turtle.launch.py](turtle_hw_1/launch/turtle.launch.py) 를 생성해 준다. 
```
touch turtle.launch.py
code turtle.launch.py
```

(위의 내용을 복붙)

### 4. 빌드

```
cd ~/robot_ws
colcon build --symlink-install --packages-select msg_srv_action_interface
colcon build --symlink-install --packages-select turtle_hw_1
. ~/robot_ws/install/local_setup.bash
```

### 5. 실행 
```
ros2 launch turtle_hw_1 turtle.launch.py
```

ros2 topic pub 명령어를 이용하여 토픽"/radius_velocity_direction"을 발행

```
ros2 topic pub --once /radius_velocity_direction msg_srv_action_interface/msg/RVD "{radius: 1.0, velocity: 2.0, direction: True}"
```

시계반대방향 direction : True
시계방향 direction : False

끝-
