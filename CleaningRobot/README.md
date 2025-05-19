# 🧹 Cleaning Robot Navigation — 자동 매핑 및 경로 생성 시스템

본 프로젝트는 ROS2 환경에서 자동 매핑 및 청소 경로 생성을 목표로 하는  
Cleaning Robot Navigation 시스템입니다.  
프론티어 탐색 알고리즘을 활용하여 미탐지 구역을 효과적으로 탐색하고,  
SLAM으로 생성된 맵을 기반으로 최적의 청소 경로를 자동으로 계획합니다.


## 🎥 프로젝트 데모 영상

[Cleaning Robot Navigation](https://youtu.be/ksJAOrboxLM)

---

## 📌 주요 기능 및 구성

- **Frontier 기반 탐색 (frontier.py)**  
  - SLAM으로 생성된 맵을 분석하여 미탐색 지역(Frontier)을 탐색  
  - BFS 및 Flood Fill 알고리즘으로 도달 가능한 영역만 식별  
  - 가장 적합한 목표 지점을 선택해 자율 이동 목표 설정  

- **맵 및 위치 변환 (camera_map_tf.py)**  
  - 지도 좌표와 로봇 좌표계 간 변환 관리  
  - 로봇 위치와 미탐색 지점 시각화용 Marker 발행  
  - TF 변환을 통해 카메라 좌표에서 로봇 기준 좌표로 변환  

- **이미지 기반 특징점 매칭 (image_matcher.py, image_matcher_camera.py)**  
  - SIFT 특징점 검출 및 매칭을 통한 영상 내 참조 이미지 검출  
  - OpenCV와 CvBridge를 활용해 ROS 이미지 메시지를 처리  
  - 3D 좌표 추정 및 ROS Transform 메시지로 변환 후 퍼블리시  

---

## 🚀 실행 방법

1. SLAM 및 맵 생성 및 Navigation2 실행  
```bash
ros2 launch turtlebot4_navigation slam.launch.py
ros2 launch turtlebot4_navigation nav2.launch.py
```
```bash
ros2 launch cleaning_robot cleaning_robot_nav2.launch.py
```

2. Cleaning Robot Navigation 노드 실행
```bash
ros2 run cleaning_robot frontier          # Frontier 기반 탐색
ros2 run cleaning_robot next_point_BFS    # BFS 기반 목표점 탐색
ros2 run cleaning_robot move_goal          # 목표점 위치 publish 및 네비게이션 목표 지정
```

3. image_matcher
```bash
ros2 run cleaning_robot image_matcher     # 영상 매칭 노드 실행
ros2 run cleaning_robot image_matcher_camera # 카메라 영상 매칭 테스트
```

---

## 📂 주요 코드 파일 설명

| 파일명                    | 역할                                                    |
|--------------------------|---------------------------------------------------------|
| `frontier.py`            | 자동 탐색 가능한 영역 식별 및 Frontier 기반 목표 선정       |
| `camera_map_tf.py`       | TF 변환 및 로봇 위치 좌표계 변환, 미탐색 지역 Marker 발행    |
| `image_matcher.py`       | SIFT 특징점 기반 이미지 매칭 및 3D 위치 추정               |
| `image_matcher_camera.py`| 웹캠 영상 기반 실시간 영상 매칭 테스트                      |
| `lib_tf.py`              | 좌표 변환, 사원수 및 행렬 변환 함수들 포함                  |
| `lib_img.py`             | 시각화용 Occupancy Grid 처리 및 Marker 생성 함수 포함       |

