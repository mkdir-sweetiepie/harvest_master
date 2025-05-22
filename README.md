
본 프로젝트는 광운대학교 로봇 동아리 BARAM에서 개발한 농작물 수확 로봇의 통합 관제 시스템입니다. Qt GUI를 기반으로 한 관제탑 역할을 하며, 비전 시스템, 모터 제어, 경로 계획 등 모든 하위 시스템을 통합 제어합니다.

## 주요 기능

### 🎯 메인 제어
- **수확 시퀀스 자동 실행**: 전체 수확 과정을 자동으로 관리
- **실시간 진행 상황 모니터링**: 현재 진행 단계 및 완료율 표시
- **긴급 정지 기능**: 안전을 위한 즉시 정지 기능
- **시스템 초기화**: 전체 시스템 상태 리셋

### 👁️ 비전 시스템
- **YOLOv12-X 기반 참외 감지**: 익은 참외(8개)와 덜 익은 참외(2개) 구분
- **FoundationPose 6D 추정**: 정밀한 참외 위치 및 방향 계산
- **Camera Calibration**: 내부/외부 파라미터 및 Hand-eye 캘리브레이션

### 🔧 모터 제어
- **6DOF Manipulator 제어**: 6축 로봇팔 정밀 제어
- **수동 위치 제어**: GUI를 통한 직접적인 위치 명령
- **DC 모터 + Dynamixel 통합**: 4개 DC 모터 + 2개 Dynamixel 서보 제어
- **실시간 상태 모니터링**: 모터 상태 및 위치 피드백

### 🗺️ 경로 계획
- **TSP 최적화**: Held-Karp 알고리즘을 통한 최적 수확 순서 결정
- **RRT* 경로 생성**: 장애물 회피를 고려한 안전한 경로 계획
- **Spline Interpolation**: 부드러운 궤적 생성

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    관제탑 GUI (Qt)                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐  │
│  │  메인 제어   │ │  비전 시스템 │ │  모터 제어   │ │ 경로계획 │  │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘  │
└─────────────────────────┬───────────────────────────────────┘
                         │ ROS 2 통신
        ┌────────────────┼────────────────┐
        │                │                │
   ┌─────────┐    ┌─────────────┐    ┌─────────┐
   │비전 노드 │    │ 경로계획 노드 │    │제어 노드 │
   └─────────┘    └─────────────┘    └─────────┘
        │                │                │
   ┌─────────┐    ┌─────────────┐    ┌─────────┐
   │YOLOv12-X│    │   TSP + RRT*  │    │하드웨어 │
   │Foundation│    │              │    │STM32+U2D2│
   └─────────┘    └─────────────┘    └─────────┘
```

## 설치 및 빌드

### 1. 시스템 요구사항
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Qt5 (5.15 이상)
- OpenCV 4.x
- Python 3.10+

### 2. 의존성 설치
```bash
# ROS 2 설치 (이미 설치된 경우 생략)
sudo apt update
sudo apt install ros-humble-desktop

# Qt5 개발 도구 설치
sudo apt install qtbase5-dev qttools5-dev-tools

# 추가 ROS 2 패키지
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-gazebo-ros-pkgs

# Python 의존성
pip3 install torch torchvision ultralytics
pip3 install opencv-python numpy
```

### 3. 워크스페이스 설정
```bash
# 워크스페이스 생성
mkdir -p ~/harvest_ws/src
cd ~/harvest_ws/src

# 소스 코드 클론
git clone <repository-url> harvest_master

# 빌드
cd ~/harvest_ws
colcon build --packages-select harvest_master

# 환경 설정
source install/setup.bash
```

## 실행 방법

### 1. 실제 로봇 시스템
```bash
# 전체 시스템 실행
ros2 launch harvest_master harvest_master_launch.py

# 개별 노드 실행 (테스트용)
ros2 run harvest_master harvest_master
```

### 2. 시뮬레이션 환경
```bash
# Gazebo 시뮬레이션 포함 실행
ros2 launch harvest_master simulation_launch.py
```

## 사용법

### 메인 제어 탭
1. **시스템 상태 확인**: 상단 상태 표시기에서 각 서브시스템 상태 확인
2. **수확 시작**: "수확 시작" 버튼 클릭으로 자동 수확 시퀀스 실행
3. **진행 모니터링**: 진행 바를 통해 실시간 진행 상황 확인
4. **긴급 상황**: "긴급 정지" 버튼으로 즉시 시스템 정지

### 비전 시스템 탭
1. **수동 감지**: "수동 감지 시작" 버튼으로 참외 감지 테스트
2. **결과 확인**: 감지된 참외 목록 및 좌표 정보 확인
3. **캘리브레이션**: 카메라 캘리브레이션 실행

### 모터 제어 탭
1. **수동 제어**: X, Y, Z 좌표 및 회전각 입력 후 "위치로 이동"
2. **테스트 기능**: 절단 도구 테스트 및 홈 포지션 이동
3. **상태 모니터링**: 실시간 모터 상태 확인

### 경로 계획 탭
1. **수확 순서**: TSP 알고리즘으로 계산된 최적 수확 순서 확인
2. **현재 목표**: 진행 중인 목표 참외 정보 표시
3. **경로 재계산**: 필요시 경로 재계산 실행

## 핵심 알고리즘

### 1. YOLOv12-X 참외 감지
```python
# Custom dataset: 10,000장 이미지
# 클래스: 익은 참외(8개), 덜 익은 참외(2개)
# Loss function: BCEWithLogitsLoss
# 평가 지표: mAP@0.75 (정밀 탐지)
```

### 2. TSP 최적화 (Held-Karp)
```python
# 동적 계획법 + 비트마스크
# 시간 복잡도: O(n²×2ⁿ)
# 8개 참외에 대한 최적 순회 경로 계산
dp[mask][i] = min(dp[mask][i], dp[prev_mask][j] + dist(j, i))
```

### 3. RRT* 경로 계획
```python
# 샘플링 기반 경로 계획
# 부모 노드 재선정 + 트리 재구성
# 장애물 회피 + 최적 경로 보장
```

### 4. FoundationPose 6D 추정
```python
# Model-based + Model-free 하이브리드
# Neural Object Modeling
# Pose hypothesis generation + selection
```

## 하드웨어 사양

### Manipulator 구성
- **DOF**: 6축
- **Reach**: 1046mm
- **Weight**: 9300g
- **모터**: 
  - Base: Faulhaber 3557K024CR + Gearheads (1:126)
  - Middle: PGM36-3657E-1280
  - End: Dynamixel MX-106R, AX-12A

### 센서 시스템
- **카메라**: Intel RealSense D435
- **제어보드**: STM32F429Z
- **모터 드라이버**: DDA3516P
- **통신**: U2D2 (Dynamixel), UART (STM32)

## 문제 해결

### 자주 발생하는 문제

1. **ROS 2 노드 연결 실패**
   ```bash
   # ROS_DOMAIN_ID 확인
   export ROS_DOMAIN_ID=0
   
   # 네트워크 확인
   ros2 topic list
   ```

2. **Qt GUI 실행 오류**
   ```bash
   # Qt 환경 변수 설정
   export QT_QPA_PLATFORM=xcb
   
   # 권한 확인
   xhost +local:root
   ```

3. **시리얼 포트 접근 권한**
   ```bash
   # 사용자를 dialout 그룹에 추가
   sudo usermod -a -G dialout $USER
   
   # 재로그인 후 확인
   groups $USER
   ```

4. **카메라 인식 실패**
   ```bash
   # USB 카메라 확인
   lsusb | grep Intel
   
   # 권한 설정
   sudo chmod 666 /dev/video*
   ```

5. **모터 제어 실패**
   ```bash
   # 포트 확인
   ls /dev/tty*
   
   # STM32 연결 확인
   dmesg | grep tty
   
   # Dynamixel U2D2 확인
   lsusb | grep ROBOTIS
   ```

## 개발팀 정보

### BARAM 로봇 동아리 (광운대학교)
- **팀장**: 임동균 (30기) - Manipulator 제어
- **팀원**: 오가현 (32기) - Manipulator 시뮬레이션
- **팀원**: 홍지현 (32기) - Computer Vision & SLAM
- **팀원**: 정우경 (33기) - Computer Vision (팀장)

### 연락처
- **Email**: baram@kw.ac.kr
- **Website**: baram.kw.ac.kr
- **GitHub**: [프로젝트 저장소 URL]

## 라이선스

MIT License

## 기여 방법

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 업데이트 로그

### v1.0.0 (2025-03-28)
- 초기 버전 릴리즈
- 기본 GUI 인터페이스 구현
- ROS 2 통신 구조 완성
- 주요 알고리즘 통합

### 향후 계획
- [ ] 실시간 3D 시각화 추가
- [ ] 웹 기반 원격 모니터링
- [ ] AI 기반 수확 품질 예측
- [ ] 다중 로봇 협업 시스템
- [ ] 클라우드 데이터 수집 및 분석

## 기술적 세부사항

### ROS 2 토픽 구조
```bash
# 발행 토픽
/system_state          (std_msgs/String)
/camera_trigger        (std_msgs/Bool)
/joint_velocity        (std_msgs/Float64MultiArray)
/path_planning_request (geometry_msgs/Pose)
/cutting_trigger       (std_msgs/Bool)
/system_command        (std_msgs/String)

# 구독 토픽
/detected_crops        (geometry_msgs/PoseArray)
/harvest_order         (std_msgs/Float64MultiArray)
/planned_path          (std_msgs/Float64MultiArray)
/crop_6d_pose          (geometry_msgs/Pose)
/robot_status          (std_msgs/String)
/vision_status         (std_msgs/String)
/motor_status          (std_msgs/String)
/planning_status       (std_msgs/String)
```

### 상태 머신
```cpp
enum class RobotState {
    INIT,              // 초기화
    DETECTION,         // 참외 감지
    HARVEST_PLANNING,  // 수확 계획
    APPROACHING,       // 접근중
    FOUNDATION_POSE,   // 줄기 추정
    CUTTING,          // 절단중
    NEXT_TARGET,      // 다음 목표
    COMPLETED,        // 완료
    ERROR             // 오류
};
```

### 통신 프로토콜

#### STM32 통신 (DC 모터)
```cpp
// 패킷 구조
struct MotorPacket {
    uint8_t header;      // 0xAA
    uint8_t length;      // 16
    float velocities[4]; // 4개 모터 속도
    uint8_t checksum;    // 체크섬
};
```

#### Dynamixel 통신
```cpp
// 프로토콜 2.0 사용
#define ADDR_TORQUE_ENABLE  64
#define ADDR_GOAL_POSITION  116
#define DXL_ID_1            5
#define DXL_ID_2            6
```

### 성능 지표

#### 비전 시스템
- **YOLOv12-X 정확도**: mAP@0.75 > 90%
- **FoundationPose 정밀도**: < 2mm 위치 오차
- **처리 속도**: 30 FPS (실시간 처리 불필요)

#### 모터 제어
- **위치 정확도**: ±1mm
- **반복 정확도**: ±0.5mm
- **최대 속도**: 관절별 차등 (1.0-2.0 rad/s)

#### 경로 계획
- **TSP 최적화**: 8개 노드 < 1ms
- **RRT* 계획**: 평균 2-5초
- **궤적 생성**: Cubic B-Spline 보간

## API 참조

### QNode 클래스 주요 메서드
```cpp
// 시스템 제어
void startHarvestSequence();
void stopHarvestSequence();
void emergencyStop();
void resetSystem();

// 모듈 제어
void triggerVisionDetection();
void moveToPosition(double x, double y, double z, double rx, double ry, double rz);
void setJointVelocities(const std::vector<double>& velocities);
void activateCuttingTool();

// 상태 조회
QString getCurrentState() const;
int getDetectedCropsCount() const;
int getCurrentTargetIndex() const;
```

### 시그널 및 슬롯
```cpp
// 주요 시그널
Q_SIGNALS:
    void stateChanged(QString state);
    void cropsDetected(int count);
    void harvestOrderUpdated(QStringList order);
    void progressUpdated(int current, int total);
    void errorOccurred(QString error);

// 주요 슬롯
private Q_SLOTS:
    void onStartHarvest();
    void onStopHarvest();
    void onEmergencyStop();
    void onResetSystem();
```

## 설정 파일

### 로봇 파라미터 설정
```yaml
# config/robot_params.yaml
robot:
  dof: 6
  joint_limits:
    - [-180.0, 180.0]  # Joint 1 (deg)
    - [-90.0, 90.0]    # Joint 2 (deg)
    - [-180.0, 180.0]  # Joint 3 (deg)
    - [-180.0, 180.0]  # Joint 4 (deg)
    - [-180.0, 180.0]  # Joint 5 (deg)
    - [-180.0, 180.0]  # Joint 6 (deg)
  max_velocity: [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]  # rad/s
  workspace:
    x_range: [-0.878, 0.873]  # m
    y_range: [-0.601, 0.601]  # m
    z_range: [-0.473, 1.278]  # m
```

### 비전 시스템 설정
```yaml
# config/vision_params.yaml
vision:
  yolo:
    model_path: "models/yolov12_crop.pt"
    confidence_threshold: 0.7
    nms_threshold: 0.45
    input_size: [640, 640]
    classes: ["ripe_crop", "unripe_crop"]
  
  foundation_pose:
    mesh_path: "models/crop_mesh.obj"
    confidence_threshold: 0.8
    max_iterations: 100
  
  camera:
    intrinsics: "config/camera_intrinsics.yaml"
    extrinsics: "config/camera_extrinsics.yaml"
```

### 통신 설정
```yaml
# config/communication.yaml
communication:
  stm32:
    port: "/dev/ttyUSB1"
    baudrate: 115200
    timeout: 1000
  
  dynamixel:
    port: "/dev/ttyACM0"
    baudrate: 115200
    protocol_version: 2.0
    ids: [5, 6]
  
  ros2:
    domain_id: 0
    qos_depth: 10
    update_rate: 30  # Hz
```

## 단위 테스트

### 테스트 실행
```bash
# 전체 테스트
cd ~/harvest_ws
colcon test --packages-select harvest_master

# 개별 테스트
ros2 run harvest_master test_qnode
ros2 run harvest_master test_communication
ros2 run harvest_master test_algorithms
```

### 테스트 케이스
1. **통신 테스트**: ROS 2 토픽 송수신 검증
2. **알고리즘 테스트**: TSP, RRT* 정확성 검증
3. **GUI 테스트**: 사용자 인터페이스 반응성 테스트
4. **하드웨어 테스트**: 모터 제어 명령 검증

## 성능 최적화

### 메모리 사용량 최적화
- **IPC 통신**: 동일 프로세스 내 노드 간 메모리 공유
- **이미지 압축**: JPEG 압축으로 네트워크 대역폭 절약
- **캐시 활용**: 계산 결과 캐싱으로 중복 계산 방지

### 실시간 성능
- **스레드 분리**: GUI 업데이트와 제어 로직 분리
- **우선순위 설정**: 실시간 제어 스레드 높은 우선순위
- **지연 최소화**: 직접 메모리 접근으로 통신 지연 감소

## 참고 문헌

1. Tian, Y., et al. "YOLOv12: Attention-centric real-time object detectors." arXiv preprint (2025)
2. Wen, B., et al. "FoundationPose: Unified 6D pose estimation and tracking." CVPR (2024)
3. Karaman, S., & Frazzoli, E. "Sampling-based algorithms for optimal motion planning." IJRR (2011)
4. Craig, J.J. "Introduction to Robotics: Mechanics and Control" 4th Edition (2017)

---

**Copyright (C) 2025. All rights reserved by Robotics Club BARAM, Kwangwoon University**
