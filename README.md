# Cleaning Robot Navigation

## 실행 명령어

### 1. Frontier 기반 Canny Path Finding
```bash
ros2 run cleaning_robot frontier
```

### 2. BFS 기반 Path Finding - 패기
```bash
ros2 run cleaning_robot next_point_BFS
```

### 3. 자신 위치 기준 Grid Search
```bash
ros2 run cleaning_robot next_point_BFS
```

### 4. `/nearest_unknown`을 받아 `/goal_pose`(nav2) 발신
```bash
ros2 run cleaning_robot move_goal
```

## Navigation 및 SLAM 실행
### TurtleBot4 Navigation 및 SLAM 실행
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

### TurtleBot4 Navigation 실행
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

### Cleaning Robot Navigation 실행
```bash
ros2 launch cleaning_robot cleaning_robot_nav2.launch.py
```

