# TurtoBot: 차선 기반 자율주행 + ArUco 작업 수행 서비스 로봇

**ROS 2 + OpenCV 기반으로, 차선 인식 → 자율주행 → ArUco 마커 감지 후 작업 수행까지 연결되는 모바일 매니퓰레이터 시스템을 구현했습니다.**
**조명 변화 등 비정형 환경에서도 견고하게 주행할 수 있도록 전처리 기반 차선 인식 개선과 상태 기반 복구 제어를 적용했으며, 미션 기반 주행 후 ArUco 마커 인식을 통해 작업 수행 시스템을 구축하였습니다.**

* 📅 **개발 기간**: 2025.06.09 \~ 06.19 (11일)
* 🧑‍🤝‍🧑 **인원 구성**: 3명<br>
  **(본인 역할: 시뮬레이션 환경 제작, GUI 설계, 차선 인식 알고리즘 및 BEV 변환, 이미지 전처리 성능 개선)**
* 🛠 **사용 기술**: ROS 2, OpenCV, ArUco, MoveIt, RViz, Python, Ubuntu 22.04
* ✅ **주요 기능**: 차선 인식 기반 자율주행, ArUco 마커 인식, 매니퓰레이터 제어, GUI 인터페이스

### 🎥 시연 영상

[![시연 영상 보기](https://img.youtube.com/vi/BHgXm7nOlXM/0.jpg)](https://youtu.be/BHgXm7nOlXM)

📄 [PDF 소개 자료](Turtobot.pdf)  

---

### 📑 프로젝트 개요

* **주제**: TurtoBot (모바일 매니퓰레이터 서비스 로봇)
* **목표**: 차선 기반 자율주행 + ArUco 기반 물체 인식 및 작업 수행 자동화
* **형태**: TurtleBot3 Waffle + 매니퓰레이터 + 카메라 결합 모바일 로봇

<br>

#### 🧑‍🤝‍🧑 팀원 역할 분담

> | 이름           | 역할                                        |
> | ------------ | ----------------------------------------- |
> | **이세현(작성자)** | **차선 인식 및 이미지 전처리, 시뮬 GUI 및 환경 구성** |
> | 강인우          | 시뮬레이션 제어 태스크, 이미지 전처리 보조, 횡단보도 정지선 인식 로직 구현  |
> | 이형연          | Aruco 마커 인식, 매니퓰레이터 제어 로직 및 표지판 인식 학습     |

<br>

#### 📅 작업 일정

> | 기간           | 작업 내용                        |
> | ------------ | ---------------------------- |
> | 6/09         | 시뮬레이션 기획 및 기능 설계 시작                   |
> | 6/10 \~ 6/13 | 시뮬레이션 테스트 및 차선 인식/주행 구현      |
> | 6/14         | 실주행 적용 및 기능 설계 시작 |
> | 6/14 \~ 6/17 | 이미지 전처리 테스트 및 차선 인식/주행 구현      |
> | 6/17 \~ 6/18 | ArUco 인식 및 로봇팔 제어 연동, 통합 테스트 |
> | 6/19         | GUI 구성, 영상 시연 촬영 및 보고서 작성    |

<br>

#### 💡 본인 주요 기여

* 조명과 반사에 강한 이미지 전처리 알고리즘 구현 (HSV 마스킹, 히스토그램 스트레칭, BEV 등)
* 차선 인식 기반 상태 머신(FSM) 주행 알고리즘 구현
* TurtleBot3 시뮬레이션 GUI 및 환경 구성

> ##### 🎨 시뮬레이션 GUI
<img width="517" height="309" alt="터토봇2" src="https://github.com/user-attachments/assets/215fc240-f57c-43cf-87e1-d35bf40f4608" />

---

#### 🛠 기본 구성 실행 순서

   - **로봇 bringup 및 MoveIt 실행**
   
      ```bash
      ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
      ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
      ros2 run turtlebot_moveit turtlebot_arm_controller
      ```
   
   - **카메라 및 전처리 코드 실행**
   
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

---

## 1. 개발 배경 및 기획

차선 인식 기반 자율주행은 단순한 라인트레이서 이상의 응용 가능성을 가지며, 이를 모바일 매니퓰레이터와 결합할 경우 단일 로봇으로 `이동 + 작업`이 가능한 복합 서비스가 가능해집니다.

**TurtoBot 프로젝트**는 실내 환경에서 AGV 역할을 하는 TurtleBot3에 로봇팔을 탑재하고, ArUco 마커 기반 작업 수행 기능을 더해 다양한 현장 시나리오에서 대응 가능한 서비스 로봇 시스템을 구현하는 것이 목표였습니다.

### 🔧 핵심 기능

1. 차선(가이드라인) 인식 기반 자율주행 (라인 트래킹)
2. 상태 기반 주행 복구 로직
3. ArUco 마커 인식 기반 위치 추정 및 작업 지시


### 🧭 프로세스(차선 인식 기반 자율주행)

<img width="1189" height="265" alt="image" src="https://github.com/user-attachments/assets/ee93e934-9d7e-4ccc-8efc-04e7c0d414e3" />

---

## 2. 사용 장비 및 기술 스택

| 분류     | 기술                                     |
| ------ | -------------------------------------- |
| 언어     | Python                                 |
| 로봇 플랫폼 | TurtleBot3 Waffle + 매니퓰레이터             |
| OS     | Ubuntu 22.04                           |
| 주행 제어  | ROS 2, RViz, Gazebo, /cmd\_vel, MoveIt |
| 비전 인식  | OpenCV + ArUco + 카메라 (C270)            |
| 차선 인식  | CLAHE + Histogram Stretch + BEV 변환     |
| GUI    | PyQt 기반 이미지 디스플레이                      |



---

## 3. 주요 기능 구현 과정

### 3-1. 🛠 이미지 전처리 기반 차선 인식 고도화

* **문제점**
  조명 변화(밝기, 그림자, 반사광 등)로 인한 마스크 추출 실패

* **해결 전략**: 다단계 전처리 파이프라인 구현

| 단계 | 기법                                   | 목적                   |
| -- | ------------------------------------ | -------------------- |
| ①  | CLAHE (명암 향상용 히스토그램 균일화)  | 국소 대비 향상 + 반사광 보정    |
| ②  | convertScaleAbs                      | 밝기/대비 조정    |
| ③  | 히스토그램 기반 선형 스트레칭       | 전역 대비 향상 + 노이즈 억제       |
| ④  | HSV 색상 마스크 추출 + 실시간 트랙바 튜닝   | 실시간 색상 마스크 조정        |
| ⑤  | Contour 필터링 (`keep_largest_contour`) | 가장 큰 윤곽만 유지 (노이즈 제거) |
| ⑥  | BEV (Bird Eye View) 변환               | 시점 왜곡 보정 후 라인 직선화    |

**keep\_largest\_contour 함수**는 작은 반사 노이즈를 필터링하고 차선만 유지합니다.<br>
<br>

### 📷 이미지 전처리 결과

<img width="1009" height="301" alt="image" src="https://github.com/user-attachments/assets/f5d9fbff-5f7b-47ac-8d54-3593dd6536bd" />

                        **< ① ~ ③ 단계 적용 결과 >**

<br>

<img width="1121" height="429" alt="image" src="https://github.com/user-attachments/assets/8feccc15-7802-4f3d-9ad1-7c55307f70b8" />

                        **< ① ~ ⑥ 단계 적용 결과 >**

---

### 3-2. 📏 차선 인식 및 중심선 시각화

**역할**

* 차선 라인(노란선, 흰선) 검출 후 중앙선 추정
* 인식 결과를 토픽 및 시각화 이미지로 발행

#### ✅ 주요 처리 흐름

| 단계 | 내용                                                                         |
| -- | -------------------------------------------------------------------------- |
| ①  | `/image_compensated/compressed`, `/white_mask`, `/yellow_mask` 세 이미지 동기 수신 |
| ②  | 차선 영역을 감싸는 ROI(Region of Interest) 시각화                                     |
| ③  | 원근 제거를 위한 BEV(Bird Eye View) 변환 수행                                         |
| ④  | 각 마스크 이미지에서 **non-zero 픽셀 좌표** 추출                                          |
| ⑤  | 기존 피팅 정보 기준으로 `±300px` 내의 픽셀만 선별하여 이차함수로 피팅 (`np.polyfit`)                 |
| ⑥  | 좌/우 차선의 존재 여부 판단 후 중심선(center line) 추정                                     |
| ⑦  | 중심선, 차선 영역, 좌/우 차선을 색상 기반으로 시각화                                            |
| ⑧  | `/lane_center`, `/lane_state`, `/image_lane_detected` 토픽 발행                |

#### 🎯 상태 판단 기준 및 시각화

| 항목     | 조건                      | 색상 (BGR)      |
| ------ | ----------------------- | ------------- |
| 노란선 인식 | yellow mask 픽셀 수 > 3000 | (255, 255, 0) |
| 흰선 인식  | white mask 픽셀 수 > 3000  | (0, 0, 255)   |
| 중앙선    | 좌/우 차선 평균 or 추정 위치      | (0, 255, 255) |
| 차선 영역  | 좌/우 차선 사이 영역            | 초록색           |


* 상태값 예시:

  * `lane_state == 2`: 양쪽 차선 모두 인식
  * `lane_state == 1`: 노란선만
  * `lane_state == 3`: 흰선만


#### 🧩 코드 기반 로직 예시

```python
# 이전 피팅 기반 차선 주변 픽셀 선택 후, 새로운 피팅
lane_inds = ((nonzerox > predicted_x - margin) & (nonzerox < predicted_x + margin))
new_fit = np.polyfit(y, x, 2)  # 이차 함수 피팅

# 중앙선 추정
if yellow_detected and white_detected:
    centerx = np.mean([left_fitx, right_fitx], axis=0)
elif yellow_detected:
    centerx = left_fitx + offset  # 기준 보정
elif white_detected:
    centerx = right_fitx - offset
```

#### 🖼️ 결과 이미지
<img width="732" height="593" alt="image" src="https://github.com/user-attachments/assets/838b6352-29ee-4f19-9520-26c8a87851c8"/>

* BEV 변환 → 라인 피팅 → 중심선 추정 → 시각화 → 역변환 후 합성 이미지 `/image_lane_detected`로 발행
* `cv2.fillPoly`로 차선 영역 색칠, `cv2.polylines`로 차선 라인 시각화

---
### 3-3. 🤖 상태 기반 자율주행 제어

**구현 코드**: `control_lane.py`

**핵심 로직**

* Finite State Machine(FSM) 기반
* 상태별 행동 정의:

  * `Normal`: 차선 따라 주행
  * `Lost`: 차선 상실 → 회복 또는 정지
  * `Stop`: 정지선 인식 시 정지

**제어 알고리즘**

* PD 제어 기반 조향 제어

  * 오차(`center - 이미지 중앙`) 기반
  * `steering = Kp * error + Kd * d(error)`
* 속도 조절: 차선 인식 상태에 따라 감속 또는 정지

---

### 3-4. 🎯 ArUco 마커 기반 작업 수행

* 주행 도중 ArUco 마커를 인식할 경우 ID 기반으로 작업 수행 로직 분기
* 예: ID 12 → 정지 후 작업팔 수행, ID 14 → 회전 후 대기
* 자세 추정을 위해 `estimatePoseSingleMarkers` 활용


#### ⚙️ 구현 내용
> MoveIt + ROS 2 비동기 서비스 요청 구조<br>
> cmd 값을 통해 로봇팔 동작/그리퍼 제어 분기<br>
1. 그리퍼 열기<br>
2. 마커 위로 이동<br>
3. 집을 수 있는 위치로 이동<br>
4. 그리퍼 닫기<br>
5. 위로 올리기<br>
6. 컨베이어 상단 위치 이동<br>
7. 적재 위치 이동<br>
8. 그리퍼 열기<br>
9. 복귀<br>

```python
corners, ids, _ = aruco.detectMarkers(...)
if id == 12:
    execute_grasp()
```

---

## 4. 핵심 코드 구현

### `img_publish.py`

**역할**: 카메라 원본 영상 압축 이미지로 퍼블리시<br>
**기능**:
* 실행 가능한 카메라 소스를 자동으로 탐색하여 순차적으로 테스트 후 적용
* 카메라에서 들어오는 원본 영상 프레임을 압축 이미지 토픽으로 ROS2 퍼블리시

---

### `image_preprocesser.py` & `image_compensation.py`

**역할**: 이미지 전처리 및 HSV 기반 색상 마스킹, 트랙바를 통한 파라미터 튜닝 제공<br>
**기능**:
 * **주요 전처리 기법**

| Python 파일                  | 주요 처리 내용                                          |
| ----------------------- | ------------------------------------------------- |
| `image_preprocess.py`   | CLAHE (HSV V채널, LAB L채널), 감마 보정, V채널 억제, 밝기·대비 조정 |
| `image_compensation.py` | 그레이스케일 히스토그램 누적 분포 분석 및 클리핑 기반 선형 스트레칭 적용         |

* CLAHE 적용 대상: HSV 색공간 V 채널, LAB 색공간 L 채널
* 감마 보정: `adjust_gamma(image, γ=1.6)`
* 밝기 및 대비 조정: OpenCV `convertScaleAbs` 함수 활용 (alpha, beta 파라미터 적용)
* 히스토그램 클리핑 기반 선형 스트레칭:

  * 누적 히스토그램 분석 후 임계값(`min_gray`, `max_gray`) 산출
  * 산출값으로 픽셀 값 선형 변환 수행 (alpha, beta 활용)
---


### `detect_lane.py`

**역할**: 이미지 동기화 및 BEV 변환 기반 차선<br>
**기능**:
  * `/image_compensated/compressed`, `/image_compensated/white_mask/compressed`, `/image_compensated/yellow_mask/compressed`<br>
    → 3개 이미지 토픽 동기화 수신(`message_filters.ApproximateTimeSynchronizer` 활용)
  * 보정 이미지와 색상 마스크 입력 → BEV 변환
  * BEV 변환 영역 ROI로 영역 시각화
  * 좌우 차선 픽셀 검출 후 2차 다항식 피팅으로 차선 곡선 산출
  * 차선 상태(양쪽, 한쪽, 없음) 판별 및 중앙 좌표 산출 → `/lane_center` 토픽 발행
  * 차선 시각화 이미지 생성 및 퍼블리시

---
### `detect_stop_line.py`

**역할**: 횡단보도(정지선) 검출<br>
**기능**:

  * 원본 영상 하단 ROI 추출 후 이진화 및 윤곽선 검출
  * 수직 띠 형태 및 흰색 픽셀 수 조건 만족 시 정지선 감지
  * 감지 여부 `/detect/stopline` Bool 메시지 발행


---
### `control_lane.py`

**역할**: FSM 기반 상태 제어 및 PD 제어를 통한 주행 명령 생성<br>
**기능**:
* `/lane_state` 구독하여 `이전 상태(prev_lane_state)`와 `현재 상태(curr_lane_state)` 비교 후 상태 전환 처리 로직 구현

| 이전 상태 → 현재 | 대응 행동                    |
| ---------- | ------------------------ |
| 1→0 or 3→0 | 마지막 유효 center 기준으로 조향 유지 |
| 2→0        | 즉시 정지 (안전 확보)            |
| 0→2        | 라인 복귀 시 자동 주행 재개         |

* `/lane_center` 구독하여차선 중심 오차 기반 PD 조향 제어 및 속도 조절 명령 발행
* `/detect/stopline` 구독하여 정지선 감지 시 차량 정지
* `/max_vel` 구독으로 최대 속도 반영, 조향 오차에 따라 선속도 제한
* 계산된 속도 및 조향값을 `/cmd_vel` 토픽에 Twist 메시지로 발행

---

### `aruco_detector.py`

**역할**: 카메라 영상 내 ArUco 마커 인식 및 3D 위치/자세 추정<br>
**기능**:

* OpenCV `detectMarkers()`를 통해 마커 코너와 ID 검출
* 사용자 정의 `estimatePoseSingleMarkers()`로 각 마커의 3D 위치(tvec) 및 회전벡터(rvec) 계산
* 회전행렬을 오일러 각도로 변환해 마커의 자세 표현
* 인식된 마커 ID 및 위치 정보를 MarkerArray 메시지로 발행
* 가장 가까운 마커 정보 로깅

---

### `pick_and_place.py`

**역할**: 특정 ArUco 마커 인식 시 로봇 팔 픽앤플레이스 수행<br>
**기능**:

* 감지된 마커 ID 기반 매니퓰레이터 목표 위치 계산
* MoveIt 연동을 통한 픽업 및 이동 동작 실행
* 작업 완료 후 초기 위치로 복귀 및 대기 상태 유지

---

## 5. 도전과 해결

| 문제                   | 해결 전략                                       |
| -------------------- | ------------------------------------------- |
| 조명·반사 등으로 인한 마스크 불안정 | CLAHE + 선형 히스토그램 스트레칭 + HSV 튜닝 기반 견고 마스크 구축 |
| 차선 외 다른 노이즈 필터링 | 이미지 컨투어 기법을 사용하여 노이즈 오인식 방지 |
| 차선 일부 상실 시 주행 흐름 불안정 | FSM 기반 자동 복구 로직 및 PD 조향 안정화                 |
| 프레임 동기화 문제        | ROS2 `message_filters.ApproximateTimeSynchronizer` 사용 |
| 로봇팔 동작 지연         | 비동기 서비스 구조 → 콜백 기반 단계별 실행 안정화                         |

---

## 6. 협업 내용 및 흐름
### 🤝 협업 내용
본 프로젝트는 이미지 전처리, 차선 주행, 마커 인식 및 매니퓰레이터 제어 등 모듈들이 유기적으로 연결된 구조로, **ROS 2 토픽 및 파라미터 공유**를 통해 통합 개발을 진행했습니다.

### 🔗 모듈 연동 구조

| 모듈                      | 발행 토픽 예시                             | 구독 모듈               |
| ----------------------- | ------------------------------------ | ------------------- |
| `image_preprocesser.py` | `/white_mask`, `/yellow_mask`        | `detect_lane.py`    |
| `image_compensation.py` | `/image_compensated/compressed`      | `detect_lane.py`    |
| `detect_lane.py`        | `/lane_center`, `/lane_state`        | `control_lane.py`   |
| `detect_stop_line.py`   | `/detect/stopline`                   | `control_lane.py`   |
| `aruco_detector.py`     | `/aruco/markers`, `/aruco/pose_info` | `pick_and_place.py` |

* 토픽 및 메시지 타입을 사전에 정의하여 **모듈 연동 시 오류 최소화**
* `rqt_image_view`, `rqt_graph`, `ros2 topic echo` 등을 활용한 **실시간 시각화 및 상태 디버깅**

---

### 🧑‍💻 협업 방식

* **역할별 모듈 독립 개발 → 추후 통합 편의성을 위해 노드, 토픽 구조 상시 공유**

  * 기능별 구현 후, 매일 오전 **토픽 연결 및 메시지 구조 점검**
  * **서브 노트 및 실제 로봇 환경**에서 통합 테스트 수행

* **HSV & BEV 파라미터 실시간 공유**

  * HSV 마스킹과 BEV 시점 변환 파라미터를 **트랙바 GUI**로 실시간 조정
  * 주요 설정값은 `.json`으로 저장 후 팀원 간 공유 → **재현성 확보**

* **FSM 상태 판단 기준 정합을 위한 디버깅**

  * `/lane_state`, `/detect/stopline` 상태를 **로그 출력 및 시각화**
  * 정지선 감지 기준(예: 흰색 픽셀 수 임계값)은 테스트를 통해 수치화하고, 하드 코딩하여 팀원 간 동작 판단 기준을 일치


---

## 7. 성과 및 결과물

* 조명 변화 환경에서도 실시간 차선 인식 → 자율주행 흐름 완성
* FSM 기반 제어로 차선 이탈 상황에서 자동 회복 및 정지 동작 구현
* 밝기/대비 변화에 강한 이미지 전처리 알고리즘 적용 경험
* 이미지 정합 기반 프레임 동기화 및 후처리 안정화
* GUI 인터페이스 구성으로 시각화 및 제어 가능
* ArUco 마커 기반 위치 추정 및 매니퓰레이터 제어 연동 성공

---

## 8. 프로젝트 결과 요약

* 차선 기반 주행 → 인식 → 작업 수행이 연속적으로 자동화된 ROS 기반 시스템 완성
* 조명과 반사 등 환경 변화에도 안정적 주행 가능한 TurtleBot3 시스템 구현
* 차선 중심 추출 및 BEV 기반 라인 검출 정확도 향상

---

## 9. 개인적 성찰 및 배운 점

* 실시간 영상처리와 ROS 제어 구조 설계 실전 경험
* FSM 기반 복구 제어 로직을 처음부터 설계하고 실제에 적용한 경험 축적
*  하나의 시스템을 동작시키기 위해 기능별로 역할 분담 → 통합 구조를 어떻게 설계해야 하는지 감각을 익힘

---

## 10. 개선 및 확장 아이디어

* 실환경 대응을 위한 외부 조도 센서 및 자동 HSV 파라미터 자동 튜닝 기능 추가
* ArUco 인식 시 음성 안내
* 작업팔, 리프팅 모듈과 추가 연동 실험
