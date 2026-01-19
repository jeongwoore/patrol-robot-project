# 🚓 TurtleBot3 Patrol Robot Project

## 1. 프로젝트 개요

본 프로젝트는 **ROS2 기반 TurtleBot3 자율 순찰 로봇**을 구현하는 것을 목표로 합니다.
로봇은 사용자가 지정한 **출발점(2D Pose Estimate)** 과 **목표점(2D Goal Pose)** 사이를 **왕복 순찰**하며, 주행 중 **사람 추종**, **신호등 인식**, **동적 장애물 회피** 기능을 수행합니다.

주요 특징은 다음과 같습니다.

* 🗺️ **D* Lite 알고리즘**을 이용한 동적 경로 재계획
* 🚗 **Pure Pursuit 제어기** 기반 경로 추종
* 🧍 **YOLOv8 기반 사람 인식 및 추종**
* 🚦 **신호등(RED / GREEN) 인식에 따른 주행 제어**
* 🧱 **LaserScan 기반 동적 장애물 회피**

---

## 2. 시스템 구성

### 🔧 하드웨어

* TurtleBot3 (Waffle Pi)
* LiDAR (기본 장착)
* USB Camera / Webcam

### 💻 소프트웨어

* Ubuntu 22.04
* ROS2 Humble
* OpenCV
* YOLOv8 (Ultralytics)
* Nav2 (Localization용)

---

## 3. 전체 시스템 구조

```text
RViz (2D Pose / Goal)
        ↓
D* Lite Path Planner
        ↓
Global Path (Grid)
        ↓
Pure Pursuit Controller
        ↓
/cmd_vel
        ↓
TurtleBot3
```

추가 인식 모듈:

* Camera → YOLOv8 → 사람 / 신호등 인식
* LiDAR → LaserScan → 동적 장애물 반영 → D* Lite 재계획

---

## 4. 주요 기능 설명

### 4.1 D* Lite 기반 경로 생성

* OccupancyGrid 맵을 기반으로 최단 경로 생성
* 새로운 장애물 발견 시 **실시간 재계획 가능**
* 장애물 Inflation 적용으로 안전 거리 확보

### 4.2 Pure Pursuit 주행 제어

* 경로 상 Lookahead Point 추적
* 각도 오차 기반 선속도 / 각속도 제어
* 부드러운 곡선 주행 가능

### 4.3 왕복 순찰 로직

* 출발점 ↔ 목표점 도달 시 자동 전환
* 무한 반복 순찰 구조

### 4.4 사람 추종 기능

* YOLOv8을 이용한 사람(person) 객체 검출
* 화면 중심 오차 기반 회전 제어
* Bounding Box 크기 기반 거리 유지

### 4.5 신호등 인식

* RED 신호 → 정지
* GREEN 신호 → 주행 재개

### 4.6 동적 장애물 회피

* LaserScan 데이터 실시간 수신
* 새로운 장애물 발견 시 맵 업데이트
* D* Lite 경로 재계산

---

## 5. 실행 방법

### 5.1 TurtleBot3 실행

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### 5.2 Localization 실행

```bash
ros2 launch nav2_bringup localization_launch.py map:=/절대경로/map.yaml
```

### 5.3 RViz 실행

```bash
ros2 run rviz2 rviz2 -d patrol.rviz
```

### 5.4 Patrol Node 실행

```bash
ros2 run patrol_pkg dstar_lite
```

---

## 6. 프로젝트 구조

```text
patrol-robot-project/
 ├── patrol_pkg/
 │   ├── dstar_lite.py   # D* Lite + Pure Pursuit
 │   ├── yolo_human.py      # 사람 추종
 │   ├── yolo_light.py     # 신호등 인식
 │   └── yolo_detect.py
 ├── launch/
 ├── rviz/
 ├── map/
 ├── README.md
 └── .gitignore
```

---

## 7. 기대 효과

* 실내 환경에서 자율 순찰 로봇 구현 가능
* 동적 환경에서도 안정적인 경로 재계획
* AI 비전 + 로봇 내비게이션 융합 프로젝트

---

## 8. 향후 확장 아이디어

* State Machine 기반 행동 전환
* Twist Mux를 이용한 주행 명령 통합
* 다중 목표 지점 순찰
* 범죄자 특정 객체 추적

---

## 9. 개발자

* 이름: Jeongwoore
* 전공/분야: AI · Robotics
* GitHub: [https://github.com/jeongwoore](https://github.com/jeongwoore)
