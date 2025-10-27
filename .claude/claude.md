# RACE 프로젝트 - Claude 설정

## 언어 설정
**중요: 항상 한국어로 답변해주세요.**

모든 응답, 설명, 코드 주석은 한국어로 작성합니다.

## 프로젝트 개요
이 프로젝트는 F1TENTH 자율주행 레이싱 시스템입니다.

### 워크스페이스 구조
- `sim_ws/`: 시뮬레이션 워크스페이스
- `real_ws/`: 실제 차량 워크스페이스
- `map/`: 맵 파일 (1017.csv, 1017.pgm, 1017.yaml)
- `path/`: 경로 데이터 (1017/, 1017_for_mpcc/)
- `bound/`: 경계 데이터 저장용 폴더
- `trajectory_generator/`: 경로 생성 도구

### 맵/경로/경계 데이터 구조
각 워크스페이스는 독립적인 `map_global_path_bound` 폴더를 가지고 있습니다:

```
sim_ws/map_global_path_bound/
├── maps/           # 맵 파일 (.pgm, .yaml, .png)
├── global_paths/   # 레이스라인 CSV 파일
└── bounds/         # 트랙 경계 CSV 파일

real_ws/map_global_path_bound/
├── maps/           # 맵 파일 (.pgm, .yaml, .png)
├── global_paths/   # 레이스라인 CSV 파일
└── bounds/         # 트랙 경계 CSV 파일
```

### 주요 패키지
- **map_control**: 다중 레인 지원 및 TF 기반 위치 추정 MAP 컨트롤러
- **lane_selector**: 장애물 회피를 위한 레인 선택
- **obs_detect**: LiDAR 기반 장애물 감지
- **mpcc**: 모델 예측 윤곽 제어
- **f1tenth_gym_ros**: F1TENTH 시뮬레이션 환경

### 빌드 명령어
```bash
# sim_ws 빌드
cd ~/RACE/sim_ws
colcon build

# real_ws 빌드
cd ~/RACE/real_ws
colcon build
```

### 중요 설정 파일
- `sim_ws/src/map_control/config/map_controller_params.yaml`
- `sim_ws/src/lane_selector/config/lane_selector.yaml`
- `sim_ws/src/obs_detect/config/obs_detect.yaml`
- `real_ws/src/map/config/map_controller_params.yaml`
- `real_ws/src/app/config/pure_pursuit_config.yaml`

모든 경로는 `/home/ircv7/RACE/` 기준으로 절대 경로를 사용합니다.
