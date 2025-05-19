# 🧃 Serving Robot System

이 프로젝트는 **식음료 서빙 로봇**을 위한 ROS2 기반 시뮬레이션 및 제어 시스템으로,  
TurtleBot3, PySide2 GUI, MySQL 데이터베이스, 그리고 데이터 시각화를 통합하여  
로봇의 자율주행부터 주문 관리, 매출 분석까지 종합적인 솔루션을 제공합니다.

---

## 🛠 개발환경 및 사용 기술

- **운영체제**: Ubuntu 22.04  
- **개발 도구**: VSCode, Terminator  
- **프로그래밍 언어**: Python 3.10  
- **로봇 프레임워크**: ROS2 Humble  
- **데이터베이스**: MySQL 8.0.40  
- **GUI 프레임워크**: PySide2  
- **시각화**: matplotlib (NanumGothicCoding 한글 폰트 적용)  

---

## 📦 프로젝트 구성

| 파일/모듈명         | 역할 및 기능                                                   |
|---------------------|---------------------------------------------------------------|
| `db_connection.py`  | MySQL 데이터베이스 접속 함수                                    |
| `order.py`          | 주문 접수 GUI 및 주문정보 DB 저장, ROS2 주문 메시지 발행        |
| `subscribe.py`      | ROS2 주문 토픽 구독 및 주문 내역 표시 GUI                       |
| `gui_test.py`       | 로봇 경로 지정, 내비게이션 제어 GUI 및 ROS2 노드 구현           |
| `table_1.py`        | 테이블 선택 GUI 및 ROS2 주문 메시지 발행                       |
| `visualization.py`  | DB 주문 데이터 시각화 (일일 매출, 메뉴별 매출, 포장/매장 비교)  |

---

## 🚀 실행 방법

### 1. 환경 준비

- ROS2 Humble, Python 3.10, MySQL 8.0, PySide2 설치  
- `db_connection.py`의 DB 접속 정보는 환경에 맞게 수정 필요  

---

### 2. ROS2 시뮬레이션 및 GUI 실행 예

```bash
# 터틀봇 시뮬레이션 실행
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

# GUI 실행 예
python3 table_1.py     # 테이블 선택 및 주문 시작
python3 order.py       # 주문 GUI
python3 subscribe.py   # 주문 모니터링 GUI
python3 gui_test.py    # 로봇 내비게이션 GUI
```

### 데이터베이스 초기 설정
```bash
# MySQL 접속
sudo mysql -u root -p
```
```sql
-- 사용자 생성
CREATE USER 'cafe_admin'@'%' IDENTIFIED BY 'cafe';
```
```sql
-- 테이블 생성
mysql> source $HOME/setup.sql;
```
> ⚠ db_connection.py의 계정 정보와 일치시켜야 하며, root 계정 사용 시 권한 이슈 발생 가능

### 데이터 시각화
```bash
# img/ 폴더 내 일일 매출, 메뉴별 매출, 포장 vs 매장 매출 그래프 이미지 생성
python3 visualization.py
```

### 기타
- matplotlib 한글 폰트 설정 🔗  
  [velog.io 한글 폰트 설정 가이드](https://velog.io/@redgreen/Linux-linux%EC%97%90%EC%84%9C-Matplotlib-%ED%95%9C%EA%B8%80%ED%8F%B0%ED%8A%B8-%EC%84%A4%EC%A0%95%ED%95%98%EA%B8%B0)
  
---

## 🧑‍💻 주요 기능 상세 설명
- 주문 접수: PySide2 GUI에서 음료 및 디저트 선택 후, 주문을 DB에 저장하고 ROS2 토픽에 메시지 발행
- 주문 모니터링: 실시간으로 ROS2 주문 토픽 구독, 최근 주문 내역 확인 및 주문 순서 표시
- 로봇 제어: GUI 기반 로봇 경로 설정 및 내비게이션 액션 수행
- 데이터 시각화: 판매 데이터 기반 매출 현황 그래프 자동 생성

---
