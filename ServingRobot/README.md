# 🧃 Serving Robot System

이 프로젝트는 **식음료 서빙 로봇**을 위한 ROS2 기반 시뮬레이션 및 제어 시스템입니다.  
TurtleBot3와 GUI, MySQL을 활용해 자율 주행, 테이블 배정, 데이터 시각화까지 통합된 관제 기능을 구현했습니다.

---

## 🛠 개발환경 및 사용 기술

- **운영체제**: Ubuntu 22.04
- **IDE & 터미널**: VSCode, Terminator
- **프로그래밍 언어**: Python 3.10
- **로봇 프레임워크**: ROS2 Humble
- **데이터베이스**: MySQL 8.0.40
- **시각화**: matplotlib

---

## 🚀 실행 방법

### 🧭 ROS2 시뮬레이션 실행

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
ros2 run turtlebot3_gui gui
```

### 데이터베이스 초기 설정
```bash
#MySQL 접속
sudo mysql -u root -p
```
```sql
#사용자 생성
CREATE USER 'cafe_admin'@'%' IDENTIFIED BY 'cafe';
```
```sql
#테이블 생성
mysql> source $HOME/setup.sql;
```
#⚠ db_connection.py에서 root 계정 사용 시 에러 발생 가능

### GUI 및 구독 실행
```bash
python3 table_1.py
```
```bash
python3 subscribe.py
```

### 데이터 시각화
```bash
#img/ 폴더에 시각화 이미지가 자동 생성
python3 visualization.py
```

### 기타
- matplotlib 한글 폰트 설정 : https://velog.io/@redgreen/Linux-linux%EC%97%90%EC%84%9C-Matplotlib-%ED%95%9C%EA%B8%80%ED%8F%B0%ED%8A%B8-%EC%84%A4%EC%A0%95%ED%95%98%EA%B8%B0 
