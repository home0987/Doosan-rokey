# 🛒 Shopping Pet — Autonomous Navigation & Object Tracking Robot

이 프로젝트는 대형마트 및 유통기업용 **자율 주행 쇼핑 도우미 로봇(AMR)** 시스템입니다.  
YOLOv8 기반 객체 탐지와 OpenCV 색상 필터링, ROS2 내비게이션,  
그리고 사용자 인터페이스를 통한 로봇 제어 및 실시간 위치 알림 기능을 통합하여  
고객 편의성과 매장 내 로봇 활용도를 극대화하는 솔루션입니다.

---

## 📌 프로젝트 개요 (Solution Overview)

- **비즈니스 목표**: 자율 이동 로봇 카트 시스템 개발 및 판매/임대  
- **주요 기능**:  
  - 고객 자율 주행 및 추적 (YOLOv8 + OpenCV 색상 필터링)  
  - 장애물 회피 및 자동 반납  
  - 고객 신뢰도가 가장 높은 객체 선택 및 거리 유지  
  - 실시간 로봇 위치 알림 및 사용자 제어 인터페이스 제공  
- **시스템 통신**: PC와 AMR 간 ROS2 기반 통신
- 
---

## ⚠ 주요 도전과제 및 해결 방법

| 문제점                            | 해결책 및 핵심 기술                                |
|----------------------------------|--------------------------------------------------|
| 학습용 데이터 라벨링의 어려움    | YOLOv8 사전학습 모델로 객체 바운딩 박스 자동 생성 |
| 트래킹 정확성 문제                | OpenCV K-Means 클러스터링으로 빨간색 객체 필터링 |
| 다수 객체 탐지 시 불안정한 트래킹 | 신뢰도 최고 객체 선택 및 중심 좌표 기반 이동 제어 |
| 거리 유지 문제                    | 삼각측량 공식으로 거리 계산 후 적정 거리 유지    |
| 객체 미탐지 시 오작동 위험       | 3초 이상 미탐지 시 정지 및 위치 퍼블리시          |
| ROS2 통신 및 속도 변동           | 명령 퍼블리시 빈도 최적화로 속도 안정화          |
| 초기 위치로 원활한 복귀 어려움   | 웨이포인트 순차 이동 구현으로 안전한 복귀 지원    |

---

## 📈 모델 성능

- Precision: 0.9493  
- Recall: 0.9889  
- mAP@50: 0.9924  
- mAP@50-95: 0.9711  
- Fitness Score: 0.9733

---

## 🛠 개발환경 및 사용 기술

- **운영체제**: Ubuntu 22.04  
- **프로그래밍 언어**: Python 3.10  
- **로봇 프레임워크**: ROS2 Humble  
- **객체 탐지**: YOLOv8 (Ultralytics)  
- **내비게이션**: ROS2 Nav2 패키지, FollowWaypoints 액션  
- **카메라 및 영상 처리**: OpenCV  
- **기타 도구**: PySide2, MySQL, matplotlib (시각화)  

---

## 📁 주요 실행 파일

- `focal_length_measurement.py` : 카메라 초점 거리 측정  
- `camera_tracking_node.py` : YOLOv8 객체 추적 및 로봇 이동 명령 퍼블리시  
- `nav.py` : ROS2 내비게이션 웨이포인트 액션 및 GUI  
- `amr.py` : AMR 관련 제어 기능  
- `camera_detection.py` : OpenCV 기반 색상 필터링

---

## 🚀 실행 방법

### 1. 환경 준비

- ROS2 Humble, Python 3.10, OpenCV, PySide2, MySQL 설치  
- 필요 라이브러리는 `requirements.txt` 또는 개별 설치 필요  

### 2. 카메라 초점 거리 측정 (필요 시)

```bash
python3 focal_length_measurement.py
```
### 3. ROS2 내비게이션 노드 및 GUI 실행
```bash
ros2 run nav  # nav.py 내비게이션 노드 실행
```

### 4. 객체 추적 및 이동 명령 퍼블리시
```bash
ros2 run <your_package> camera_tracking_node
```
