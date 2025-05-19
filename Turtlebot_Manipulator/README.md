# 📦 Box Sorter Package — AMR + Manipulator 기반 박스 이동 자동화

## 🎥 프로젝트 데모 영상

- [Turtlebot_Manipulator 데모 영상 보기](https://youtu.be/ksJAOrboxLM)
- [unity demo](https://youtu.be/3gOX1T0tEus)
  
<table>
<tr>
<td><img src="./docs/스크린샷_2025-05-20_08-52-09.png" width="300"></td>
<td><img src="./docs/스크린샷_2025-05-20_08-52-40.png" width="300"></td>
</tr>
</table>


---

## 📌 프로젝트 개요

본 프로젝트는 AMR(자율이동로봇)과 매니퓰레이터를 활용해 박스 이동 및 적재 작업을 자동화하는 시스템입니다.  
ArUco 마커 인식과 캘리브레이션 기반 색상 감지를 이용해 작업 위치를 탐색하고, GUI 기반 Task 입력을 통해 목적지 및 작업 유형을 지정하여 AMR과 매니퓰레이터가 협력하여 박스를 컨베이어 벨트에 올리고, 최종 위치로 이동시킨 뒤 원위치 복귀하는 작업을 수행합니다.

---

## 🛠 사용 기술 스택

- ROS 2 Humble: AMR 및 매니퓰레이터 제어, GUI 통신  
- ArUco 마커 인식: 위치 탐색 및 정렬  
- 캘리브레이션 기반 박스 감지: 빨강/파랑 색상 인식  
- PySide2 GUI: Task 입력 및 실행 모니터링  
- MoveIt: 로봇 팔 경로 계획 및 제어

---

### 🔹 TurtleBot3 Manipulation Packages
- `turtlebot3_manipulation_bringup`
- `turtlebot3_manipulation_cartographer`
- `turtlebot3_manipulation_description`
- `turtlebot3_manipulation_hardware`
- `turtlebot3_manipulation_moveit_config`
- `turtlebot3_manipulation_navigation2`
- `turtlebot3_manipulation_teleop`

### 🔹 Additional Packages
- `turtlebot_cosmo_interface`
- `turtlebot_moveit`

---

## 🚀 실행 방법 (How to Run)

### 🖥️ 1. TurtleBot3에서 실행 (SSH 접속 필요)

#### 1.1 TurtleBot3 Bringup 실행
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```
- TurtleBot3의 하드웨어 제어 노드 실행

#### 1.2 YOLO & ArUco Detector 실행
```bash
ros2 launch aruco_yolo aruco_yolo.launch.py
```
- **YOLO 노드**: Red, Blue, Purple 박스 클래스 및 중심 검출
- **ArUco 노드**: ArUco 마커 종류 및 거리 데이터 측정

---

### 💻 2. PC에서 실행

#### 2.1 경로 설정 노드 실행
```bash
ros2 launch box_sorter_manipulator moveit_core.launch.py
```
- MoveIt과 연동하여 Arm 및 Gripper의 경로를 설정

#### 2.2 Arm Controller 실행
```bash
ros2 run turtlebot_moveit turtlebot_arm_controller
```
- Arm Manipulator의 조인트 이동 명령 실행

#### 2.3 Manager 노드 실행
```bash
ros2 run box_sorter simple_manager_node
```
- 상태(status)에 따른 실행 방법 정의

#### 2.4 GUI & Conveyor 노드 실행
```bash
ros2 launch box_sorter GUI_conveyor.launch.py
```

##### ✅ GUI 노드 실행
```bash
ros2 run box_sorter job_publisher
```
- 이동 명령 실행 및 상태 출력
- **Publisher**:
  - `/job_topic` (`std_msgs/String`)
  - `/conveyor/control` (`std_msgs/String`)
- **Subscriber**:
  - `/yolo/compressed` (`sensor_msgs/CompressedImage`)
  - `/conveyor/status` (`std_msgs/String`)

##### ✅ Conveyor 노드 실행
```bash
ros2 run box_sorter conveyor
```
- 컨베이어 벨트 상태를 퍼블리시함
- **Publisher**: 
  - `/conveyor/control` (`std_msgs/String`)
  - `/conveyor/status` (`std_msgs/String`)
- 아두이노와 연결하여 컨베이어 벨트를 제어함

---

## 📜 주요 노드 및 메시지
- /job_topic (std_msgs/String): 작업 명령 퍼블리시
- /conveyor/control (std_msgs/String): 컨베이어 벨트 제어
- /conveyor/status (std_msgs/String): 컨베이어 상태 모니터링
- /yolo/compressed (sensor_msgs/CompressedImage): YOLO 영상 피드
