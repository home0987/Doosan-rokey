# Rescue & Drone Simulation Packages

본 프로젝트는 **TurtleBot3** 및 **드론**을 활용한 시뮬레이션 환경을 구축하고, 이를 ROS 2 기반으로 제어하는 패키지 모음입니다.

---
## 🚀 실행 방법 
  ```sh
  ros2 launch sjtu_drone_bringup project_sjtu_drone_bringup.launch.py
  ```
  - 드론 **스폰, Gazebo, RViz2, Teleop 실행**

  ```sh
  ros2 launch rescue_turtlebot3_bringup map_world_nogazebo.launch.py
  ```
  - **드론과 함께 실행**하는 모드 (Gazebo 미포함)

  ```sh
  ros2 launch rescue_control rescue_control_launch.launch.py
  ```
  - camera_openCV
  - GUI

  ```sh
  ros2 launch rescue_turtlebot3_bringup map_world_nogazebo.launch.py
  ```
  - **드론과 함께 실행**하는 모드 (Gazebo 미포함)

  ```sh
  ros2 run rescue_turtlebot3_bringup spawn_ran
  ```
  - 목표물 랜덤위치 spawn


  ```sh
  ros2 run sjtu_drone_control drone_obj_tracker
  ```
  - 목표뮬 발견 시 드론 출발

  ```sh
  ros2 run rescue_control send_waypoint
  ```
  - waypoint 순환

## 📁 패키지 개요

### 1. sample_pkgs  
- 강사님 제공 패키지 (참고용)

### 2. rescue_control  
- **역할:** 각종 메시지를 수신하고 컨트롤 타워 역할 수행  
- **주요 실행 명령어:**  
  ```sh
  ros2 run rescue_control send_waypoint
  ```
  - `send_waypoint` : TurtleBot3의 웨이포인트 지정
 
  ```sh
  ros2 run rescue_control GUI
  ```
  - `GUI` : 관제 시스템 실행

### 3. rescue_turtlebot3_bringup  
- **역할:** TurtleBot3의 Gazebo 및 RViz2 실행, 스폰 관련 기능 제공  
- **주요 실행 명령어:**  
  ```sh
  ros2 launch rescue_turtlebot3_bringup map_world.launch.py
  ```
  - Gazebo, RViz2에서 **TurtleBot3 단독 실행**  

  ```sh
  ros2 launch rescue_turtlebot3_bringup map_world_nogazebo.launch.py
  ```
  - **드론과 함께 실행**하는 모드 (Gazebo 미포함)  

  ```sh
  ros2 run rescue_turtlebot3_bringup spawn_ran
  ```
  - **더미(random) 스폰 실행**

### 4. sjtu_drone_description  
- **역할:** 드론의 URDF, CAD 파일, World 파일 제공  

### 5. sjtu_drone_bringup  
- **역할:** 드론의 Gazebo 및 RViz2 실행, 스폰 기능 제공  
- **주요 실행 명령어:**  
  ```sh
  ros2 launch sjtu_drone_bringup project_sjtu_drone_bringup.launch.py
  ```
  - 드론 **스폰, Gazebo, RViz2, Teleop 실행**

### 6. sjtu_drone_control  
- **역할:** 드론의 Teleop 및 제어 기능 수행  
- **주요 실행 명령어:**  
  ```sh
  ros2 run sjtu_drone_control drone_obj_tracker
  ```
  - **목표물 발견 시 드론 출발**하는 기능 실행
  
  ```sh
  ros2 run sjtu_drone_control drone_amcl_follower
  ```
  - **드론 turtlebot3 실시간 추적**하는 기능 실행  

---



msgs
노드 네임스페이스 및 토픽 정리
==================================
1. **/simple_drone/simple_drone**
   - **Publish:**
     - `/simple_drone/cmd_vel` → [geometry_msgs/msg/Twist]
   - **Subscribe:**
     - `/simple_drone/gt_pose` → [geometry_msgs/msg/Pose]
     - `/simple_drone/gt_vel` → [geometry_msgs/msg/Twist]
2. **/simple_drone/teleop_node**
   - **Publish:**
     - `/simple_drone/cmd_vel` → [geometry_msgs/msg/Twist]
   - **Subscribe:** (없음)
3. **/simple_drone/bottom & front**
   - **Publish:**
     - `/simple_drone/camera_bottom/image_raw` → [sensor_msgs/msg/Image]
     - `/simple_drone/camera_bottom`
       `/simple_drone/bottom/camera_info` → [sensor_msgs/msg/CameraInfo]
   - **Subscribe:** (없음)
