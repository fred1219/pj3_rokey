# 4. 핵심 코드 구현

---

### `img_publish.py` (turtlebot3\_autorace\_camera)

**역할**:

* TurtleBot3 카메라 원본 영상 ROS 2 토픽으로 퍼블리시

**기능**:

* 카메라에서 실시간 영상 스트림 수신 및 `CompressedImage` 메시지로 변환 후 발행
* 하위 노드(전처리 노드)에서 원본 영상을 기반으로 전처리 수행 가능하도록 토픽 공급

---

### `image_preprocess.py` (turtlebot3\_autorace\_camera)

**역할**:

* 카메라 영상에 대한 초기 전처리 수행
* 원본 카메라 영상을 받아 HSV 색 공간 기반 V 채널에 CLAHE 적용 및 감마 보정, 밝기/대비 조정을 수행하여 전처리 영상 생성 및 퍼블리시

**기능**:

* HSV 변환 후 V 채널만 별도로 CLAHE 적용
* 감마 보정(γ=1.6)으로 빛 반사 억제
* V 채널 밝기 약화 (scale=0.85)로 고휘도 영역 약화
* cv2.convertScaleAbs로 추가 밝기/대비 조절 (alpha=0.9, beta=-50)
* 전처리된 이미지를 압축 JPEG 포맷으로 ROS 토픽에 퍼블리시

---

### `image_compensation.py` (turtlebot3\_autorace\_camera)

**역할**:

* 히스토그램 클리핑 기반 그레이스케일 영상 선형 스트레칭 적용

**기능**:

* 입력 영상의 그레이스케일 히스토그램 누적 분포 분석
* 클리핑 비율(clip\_hist\_percent)에 따른 최소/최대 밝기값 산출
* 이를 기반으로 선형 스트레칭(대비 조정) 수행
* calibration mode이면 트랙바 UI 실행되며 HSV, 클리핑 파라미터 실시간 튜닝 지원
* 밝기/대비 개선을 통한 차선 인식에 최적화된 영상 생성
* 처리 결과 영상 ROS 토픽으로 발행

---

### `detect_lane.py` (turtlebot3\_autorace\_detect)

**역할**:

* BEV (Bird Eye View) 변환 적용 후 차선 검출 및 중심선 좌표 산출

**기능**:

* 전처리된 영상 입력 받아 BEV 변환 수행
* 차선 마스크 추출 및 윤곽선(Contour) 분석
* 가장 큰 컨투어 필터링하여 주요 차선 확보
* 차선 중심 좌표 계산 후 `/lane_center` 토픽 발행
* 메시지 동기화를 위해 ApproximateTimeSynchronizer 사용

---

### `detect_stop_line.py` (turtlebot3\_autorace\_detect)

**역할**:

* 횡단보도(정지선) 검출

**기능**:

* 차선 이미지에서 횡단보도 패턴(흰색 띠) 추출
* 조건 기반(예: 흰색 픽셀 수, 수직 띠 개수) 정지선 감지
* 정지선 발견 시 `/stop_line` 토픽 발행

---

### `control_lane.py` (turtlebot3\_autorace\_driving)

**역할**:

* 상태 머신(FSM) 기반 차선 주행 제어 및 복구 동작 구현

**기능**:

* 차선 인식 상태(정상, 일부 상실, 완전 상실 등)에 따른 주행 행동 제어
* 차선 중심 오차 기준 PD 제어로 조향 각도 산출
* 주행 속도 및 방향 명령 생성 후 로봇 제어 명령 발행
* 차선 이탈 시 복귀 또는 정지 로직 포함

---

### `aruco_detector.py` (aruco\_yolo)

**역할**:

* 카메라 영상 내 ArUco 마커 인식 및 자세 추정

**기능**:

* OpenCV의 `detectMarkers()` 함수로 마커 코너 및 ID 추출
* ID별 분기하여 로봇 작업 명령 트리거
* `estimatePoseSingleMarkers()`를 활용해 3D 위치와 자세 계산
* 결과를 ROS 토픽으로 발행하여 후속 노드에서 활용

---

### `pick_and_place.py` (aruco\_yolo)

**역할**:

* 특정 ArUco 마커 인식 시 매니퓰레이터 픽앤플레이스 동작 수행

**기능**:

* 인식된 마커 ID에 따른 매니퓰레이터 목표 위치 산출
* MoveIt과 연동하여 픽업 → 이동 → 배치 동작 순차 실행
* 작업 완료 후 팔 원위치 복귀 및 대기 상태 전환

