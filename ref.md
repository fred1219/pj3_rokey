# TurtleAutonomy: 차선 기반 자율주행 + ArUco 작업 수행 서비스 로봇

**ROS 2 + OpenCV 기반으로, 차선 인식 → 자율주행 → ArUco 마커 감지 후 작업 수행까지 연결되는 모바일 매니퓰레이터 시스템을 구현했습니다.**
**비정형 환경에서도 견고한 주행을 위해 전처리 기반 차선 인식 개선과 상태 기반 주행 복구 제어를 적용하였고, 미션 기반 이동 후 ArUco 마커를 인식하여 후속 작업을 수행하도록 구현했습니다.**

---

### 📑 프로젝트 개요

* **주제**: TurtleAutonomy (모바일 매니퓰레이터 서비스 로봇)
* **목표**: 차선 기반 자율주행 + ArUco 기반 물체 인식 및 작업 수행 자동화
* **형태**: TurtleBot3 Waffle + 매니퓰레이터 + 카메라 결합 모바일 로봇

<br>

#### 🧑‍🤝‍🧑 팀원 역할 분담

| 이름           | 역할                                        |
| ------------ | ----------------------------------------- |
| **이세현(작성자)** | **이미지 전처리, 차선 주행 알고리즘 구현, 시뮬 환경 구성, GUI 제작** |
| 강인우          | 시뮬레이션 제어 태스크, 이미지 전처리, 횡단보도 정지 알고리즘 구현  |
| 이형연          | ArUco 마커 인식, 매니퓰레이터 제어 로직 및 표지판 인식 학습     |

<br>

#### 📅 작업 일정

| 기간           | 작업 내용                        |
| ------------ | ---------------------------- |
| 6/09         | 기획 및 기능 설계                   |
| 6/10 \~ 6/13 | 시뮬레이션 테스트 및 차선 인식/주행 구현      |
| 6/14 \~ 6/18 | ArUco 인식 및 로봇팔 제어 연동, 통합 테스트 |
| 6/19         | GUI 구성, 영상 시연 촬영 및 보고서 작성    |

<br>

#### 💡 본인 주요 기여

* 조명과 반사에 강한 이미지 전처리 알고리즘 구현 (HSV 마스킹, 히스토그램 스트레칭, BEV 등)
* 차선 인식 기반 상태 머신(FSM) 주행 복구 및 PD 제어 로직 개발
* TurtleBot3 시뮬레이션 환경 구성 및 GUI 개발

---

#### 🛠 기본 구성 실행 순서

   - **로봇 bringup 및 MoveIt 실행**
   
      ```bash
      ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
      ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
      ros2 run turtlebot_moveit turtlebot_arm_controller
      ```
   
   - **카메라 및 전처리 노드 실행**
   
      ```bash
      ros2 run turtlebot3_autorace_camera img_publish
      ros2 run turtlebot3_autorace_camera image_preprocesser
      ros2 run turtlebot3_autorace_camera image_compensation
      ```
   
   - **라인 검출 및 주행 제어**
   
      ```bash
      ros2 run turtlebot3_autorace_detect detect_lane
      ros2 run turtlebot3_autorace_detect detect_stop_line
      ros2 run turtlebot3_autorace_driving control_lane
      ```
   
   - **AR 마커 기반 픽앤플레이스 동작 (선택적 실행)**
   
      ```bash
      ros2 run aruco_yolo aruco_detector
      ros2 run aruco_yolo pick_and_place --ros-args -p markerid:=1
      ```

#### 🧭 ROS 통신 구조도(rqt_graph)
<img width="942" height="208" alt="image" src="https://github.com/user-attachments/assets/b83627b8-9fc2-4d81-9697-682b2972ce9a" />

---

## 1. 개발 배경 및 기획

최근 물류, 안내, 청소 등 자율 이동 로봇의 활용이 증가하고 있으나, 조명변화로 인한 차선 인식 실패 및 마커 기반 작업의 자동화 문제는 여전히 과제로 남아 있습니다.

**TurtleAutonomy 프로젝트**는 이 두 문제를 통합 해결하는 TurtleBot3 기반 자율주행 로봇을 목표로 설계되었습니다.

### 🔧 핵심 기능

1. 조명·반사 대응 전처리 기반 차선 인식
2. 상태 기반 주행 복구 로직
3. ArUco 마커 인식 기반 후속 작업 자동화

---

## 2. 사용 장비 및 기술 스택

| 분야     | 기술                                |
| ------ | --------------------------------- |
| 언어     | Python                            |
| 로봇 플랫폼 | ROS 2 (Humble), TurtleBot3 Waffle |
| 비전 인식  | OpenCV (CLAHE, HSV, BEV 등)        |
| 태그 인식  | ArUco Marker (4×4‑50)             |
| 이미지 전송 | image\_transport (compressed)     |
| 제어 구조  | State Machine + PD 제어             |



---

## 3. 주요 기능 구현 과정

### 3-1. 🛠 이미지 전처리 기반 차선 인식 고도화

* **문제점**
  조명 변화(밝기, 그림자, 반사광 등)로 인한 마스크 추출 실패

* **해결 전략**: 다단계 전처리 파이프라인 구현

| 단계 | 기법                                   | 목적                   |
| -- | ------------------------------------ | -------------------- |
| ①  | CLAHE                                | 국소 대비 향상 및 반사광 보정    |
| ②  | convertScaleAbs                      | 전체 밝기 안정화 및 대비 조정    |
| ③  | 선형 히스토그램 스트레칭                        | 대비 향상 + 노이즈 억제       |
| ④  | HSV 마스킹 + 트랙바 튜닝                     | 실시간 색상 마스크 조정        |
| ⑤  | Contour 필터링 (`keep_largest_contour`) | 가장 큰 윤곽만 유지 (노이즈 제거) |
| ⑥  | BEV (Bird Eye View) 변환               | 시점 왜곡 보정 후 라인 직선화    |

**keep\_largest\_contour 함수**는 작은 반사 노이즈를 필터링하고 차선만 유지합니다.

---

### 3-2. 🤖 상태 기반 자율주행 제어 (`ControlLane`)

* **문제점**
  차선 일부 상실 시 오작동, 전체 상실 시 시스템 복귀 미비

* **해결 전략**: State 기반 FSM 구조로 복구 자동화

| 이전 상태 → 현재 | 대응 행동                    |
| ---------- | ------------------------ |
| 1→0 or 3→0 | 마지막 유효 center 기준으로 조향 유지 |
| 2→0        | 즉시 정지 (안전 확보)            |
| 0→2        | 라인 복귀 시 자동 주행 재개         |

**PD 제어 로직 예시**:

```python
Kp = 0.0025; Kd = 0.007  
angular_z = Kp * error + Kd * (error - last_error)
```

* 중심 오차 작을수록 직진, 클수록 회전과 감속 증가
* FSM 구조를 통해 이탈 후 복귀, 정지, 재시작 논리를 자동화

---

### 3-3. 🎯 ArUco 마커 기반 작업 수행

* 주행 도중 ArUco 마커를 인식할 경우 ID 기반으로 작업 수행 로직 분기
* 예: ID 12 → 정지 후 작업팔 수행, ID 14 → 회전 후 대기
* 자세 추정을 위해 `estimatePoseSingleMarkers` 활용

```python
corners, ids, _ = aruco.detectMarkers(...)
if id == 12:
    execute_grasp()
```

---

## 4. 핵심 코드 구현

### img\_publish & image\_compensation (turtlebot3\_autorace\_camera)

**역할**: 카메라 압축 이미지 토픽 발행<br>
**기능**:

* `img_publish`: 원본 카메라 영상 압축 이미지 토픽 ROS2 퍼블리시

---

### imaeg\_preprocesser & image\_compensation (turtlebot3\_autorace\_camera)

**역할**: 이미지 전처리 및 HSV 마스킹 + 트랙바 튜닝<br>
**기능**:

* `image_preprocesser`: 압축 이미지 토픽을 받고 CLAHE, 대비/밝기 조정 후 ROS2 퍼블리시
* `image_compensation`: 선형 히스토그램 스트레칭, HSV 마스킹, 트랙바 기반 실시간 파라미터 튜닝, Contour 필터링을 통한 차선 추출

---


### detect\_lane & detect\_stop\_line (turtlebot3\_autorace\_detect)

**역할**: BEV 변환 기반 차선과 횡단보도(정지선) 검출<br>
**기능**:

* `detect_lane`: BEV 적용 및 차선 중심점 산출 후 `/lane_center` 토픽 발행
* `detect_stop_line`: 횡단보도 영역 검출 및 조건 만족 시 `/stop_line` 토픽 발행

---

### control\_lane (turtlebot3\_autorace\_driving)

**역할**: FSM 기반 상태 제어 및 PD 제어를 통한 주행 명령 생성<br>
**기능**:

* 차선 인식 상태에 따른 주행 복구, 정지, 재개 로직 구현
* 차선 중심 오차 기반 PD 조향 제어 및 속도 조절 명령 발행

---

### aruco\_detector (aruco\_yolo)

**역할**: 카메라 영상 내 ArUco 마커 인식 및 자세 추정<br>
**기능**:

* `detectMarkers()`로 마커 코너 및 ID 추출
* ID 분기별 작업 명령 발행
* `estimatePoseSingleMarkers()`로 3D 위치와 자세 추정

---

### pick\_and\_place (aruco\_yolo)

**역할**: 특정 ArUco 마커 인식 시 로봇 팔 픽앤플레이스 수행<br>
**기능**:

* 마커 ID에 따라 매니퓰레이터 목표 위치 계산 및 이동
* MoveIt과 연동해 픽업 및 위치 이동 동작 실행
* 작업 완료 후 복귀 및 대기

---

## 5. 도전과 해결

| 문제                   | 해결 전략                                       |
| -------------------- | ------------------------------------------- |
| 조명·반사 등으로 인한 마스크 불안정 | CLAHE + 선형 히스토그램 스트레칭 + HSV 튜닝 기반 견고 마스크 구축 |
| 차선 외 다른 노이즈 필터링 | 이미지 컨투어 기법을 사용하여 노이즈 오인식 방지 |
| 차선 일부 상실 시 주행 흐름 불안정 | FSM 기반 자동 복구 로직 및 PD 조향 안정화                 |
| 마커 인식 중 반복 오작동 발생    | ID 필터링 및 동작 간 시간 간격 조건 추가로 오작동 방지           |

---

## 6. 협업 내용 및 흐름

* 영상 전처리 → 차선 중심점 추출 → FSM 제어 → ArUco 마커 인식 → 후속 작업 수행까지 각 모듈 토픽과 파라미터 공유
* HSV 트랙바 및 BEV 파라미터 실시간 조정으로 팀원 간 동기화 및 테스트 용이
* 다양한 시뮬레이션 및 실제 환경 테스트, 영상 기록 및 피드백 통한 지속적 개선

---

## 7. 성과 및 결과물

* 조명 변화 환경에서도 견고한 차선 인식 성능 확보
* FSM 기반 제어로 차선 이탈 상황에서 자동 회복 및 정지 흐름 확보
* ArUco ID 기반 후속 작업 연계 기능 성공 구현

---

## 8. 프로젝트 결과 요약

* 차선 기반 주행 → 인식 → 작업 수행이 연속적으로 자동화된 ROS 기반 시스템 완성
* 조명과 반사 등 환경 변화에도 안정적 주행 가능한 TurtleBot3 시스템 구현
* 확장 가능한 모듈 구조 설계 및 개발

---

## 9. 개인적 성찰 및 배운 점

* 실시간 영상처리와 ROS 제어 구조 설계 실전 경험
* FSM 기반 복구 제어 설계 및 적용 능력 향상
* ArUco 인식과 작업 연계 구조 설계 실전 경험

---

## 10. 개선 및 확장 아이디어

* HSV 파라미터 자동 튜닝 기능 추가
* ArUco 인식 시 음성 안내 및 GUI 피드백 적용
* 작업팔, 리프팅 모듈과 추가 연동 실험
