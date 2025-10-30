# Steering Test Node - 사용 가이드

## 개요
조향각 명령의 응답성을 테스트하기 위한 노드입니다.
다양한 파형의 조향각 명령을 생성하여 실제 출력과 비교할 수 있습니다.

## 파형 종류

### 1. **Step (계단 함수)**
- 급격한 변화 테스트
- 지연 시간 측정에 최적
- 추천: frequency=0.5 (2초마다 전환)

### 2. **Sine (정현파)**
- 주파수 응답 테스트
- 부드러운 변화
- 추천: frequency=0.5~2.0

### 3. **Square (구형파)**
- Step과 유사하지만 더 규칙적
- 지연 시간 정확한 측정

### 4. **Triangle (삼각파)**
- 선형적인 변화
- Slew rate 테스트

### 5. **Chirp (주파수 증가)**
- 주파수가 시간에 따라 증가
- 전체 주파수 대역 스캔

## 사용 방법

### 1. 빌드
```bash
cd /home/ircv7/RACE/real_ws
colcon build --packages-select pure_pursuit
source install/setup.bash
```

### 2. Bringup 실행 (별도 터미널)
```bash
rlb  # 또는 ros2 launch f1tenth_stack bringup_launch.py
```

### 3. 테스트 실행

#### Step 입력 테스트 (0.5Hz, 2초마다 전환)
```bash
ros2 launch pure_pursuit steering_test.launch.py waveform:=step frequency:=0.5
```

#### Sine 파형 (1Hz)
```bash
ros2 launch pure_pursuit steering_test.launch.py waveform:=sine frequency:=1.0
```

#### High frequency test (2Hz)
```bash
ros2 launch pure_pursuit steering_test.launch.py waveform:=sine frequency:=2.0
```

#### 사용자 정의 설정
```bash
ros2 launch pure_pursuit steering_test.launch.py \
    waveform:=step \
    frequency:=0.5 \
    amplitude:=0.3 \
    speed:=2.0 \
    publish_rate:=100.0 \
    test_duration:=30.0
```

## PlotJuggler로 분석

### 1. PlotJuggler 실행
```bash
ros2 run plotjuggler plotjuggler
```

### 2. 토픽 구독
- Data Streaming → ROS2 Topic Subscriber
- 체크할 토픽:
  - `/drive/drive/steering_angle` (입력 명령)
  - `/ackermann_cmd/drive/steering_angle` (실제 출력)

### 3. 분석 항목
- **지연 시간**: 입력과 출력의 시간차
- **Slew rate**: 변화율 제한 여부
- **Overshoot**: 오버슈트 발생 여부
- **Settling time**: 안정화 시간

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `waveform` | step | 파형 종류 (step/sine/square/triangle/chirp) |
| `frequency` | 0.5 | 주파수 (Hz) |
| `amplitude` | 0.3 | 진폭 (rad, ±17도) |
| `speed` | 2.0 | 일정 속도 (m/s) |
| `publish_rate` | 100.0 | 발행 주파수 (Hz) |
| `test_duration` | 30.0 | 테스트 시간 (초, 0=무한) |

## 예상 결과

### 정상적인 경우:
- 입력과 출력이 거의 동일 (약간의 지연만 존재)
- 부드러운 곡선

### 문제가 있는 경우:
- 출력이 입력보다 크게 지연됨
- 출력이 slew rate에 의해 제한됨
- 출력이 입력과 다른 파형을 보임

## 트러블슈팅

### 토픽이 안 보이는 경우:
```bash
ros2 topic list | grep -E "drive|ackermann"
```

### QoS 문제:
- 노드는 BEST_EFFORT QoS 사용
- ackermann_mux와 호환됨

### 권한 문제:
```bash
chmod +x /home/ircv7/RACE/real_ws/src/app/scripts/steering_test_node.py
```
