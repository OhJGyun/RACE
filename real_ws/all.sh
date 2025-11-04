#!/usr/bin/env bash
# all.sh — build & launch stack with ROS2 Humble
set -euo pipefail

# ===== 설정 =====
WS_DIR="${WS_DIR:-/home/ircv7/RACE/real_ws}"   # 필요 시 export WS_DIR=... 로 덮어쓰기
ROS_DISTRO="${ROS_DISTRO:-humble}"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

die(){ echo "[ERR] $*" >&2; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "필요한 명령어 없음: $1"; }

# --- 안전 소스: nounset(-u) 잠시 해제 + AMENT 변수 기본값 보정
safe_source() {
  set +u
  : "${AMENT_TRACE_SETUP_FILES:=}"
  source "$1"
  set -u
}

echo "[INFO] ROS2 distro: $ROS_DISTRO, workspace: $WS_DIR"
[ -d "$WS_DIR" ] || die "워크스페이스 폴더가 없음: $WS_DIR"
[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || die "/opt/ros/$ROS_DISTRO/setup.bash 없음"

# 1) ROS 기본 환경
safe_source "/opt/ros/$ROS_DISTRO/setup.bash"

# 2) 빌드 (install/setup.bash 없으면 빌드)
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[INFO] install 없음 → colcon build 시작"
  need colcon
  cd "$WS_DIR"
  colcon build --symlink-install
else
  echo "[INFO] install 존재 → 빌드 스킵"
fi

# 3) 워크스페이스 환경
safe_source "$WS_DIR/install/setup.bash"

# 4) 패키지 존재 확인(경고만) — BrokenPipe 방지: 'ros2 pkg prefix' 사용
MISSING=()
for pkg in obs_detect slam_nav map_control lane_selector; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[WARN] 패키지 '$pkg' 를 찾지 못했습니다(해당 launch 실패 가능)"
    MISSING+=("$pkg")
  fi
done

# 5) ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID="$DOMAIN_ID"
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 실행에 공통으로 쓸 커맨드(환경 로드 + launch)
cmd_env="set +u; : \"\${AMENT_TRACE_SETUP_FILES:=}\"; source /opt/ros/$ROS_DISTRO/setup.bash; source $WS_DIR/install/setup.bash; set -u;"

# 6) 실행: tmux가 있으면 tmux로, 없으면 백그라운드로
if command -v tmux >/dev/null 2>&1; then
  SESSION="race_stack"
  echo "[INFO] tmux로 실행 (세션: $SESSION)"
  tmux has-session -t "$SESSION" 2>/div/null && tmux kill-session -t "$SESSION"

  # (1) 빈 쉘로 세션/창 생성
  tmux new-session -d -s "$SESSION" -n main "bash"

  # (2) pane 분할 (총 5개)
  tmux split-window -v -t "$SESSION":0       # 0.0 (top) / 0.1 (bottom)
  tmux split-window -h -t "$SESSION":0.0     # 0.0 (top-left) / 0.2 (top-right)
  tmux split-window -h -t "$SESSION":0.1     # 0.1 (bot-left) / 0.3 (bot-right)
  tmux split-window -v -t "$SESSION":0.2     # 0.2 (top-right) / 0.4 (mid-right)

  # (3) 각 pane에 명령 투입
  tmux send-keys -t "$SESSION":0.0 "bash -lc '$cmd_env ros2 launch obs_detect ring_obs.launch.py'" C-m
  tmux send-keys -t "$SESSION":0.1 "bash -lc '$cmd_env echo \"[INFO] Pane 0.1 reserved for future use\"'" C-m
  tmux send-keys -t "$SESSION":0.2 "bash -lc '$cmd_env ros2 launch slam_nav localization_launch.py'" C-m
  tmux send-keys -t "$SESSION":0.3 "bash -lc '$cmd_env ros2 launch map_control map_controller.launch.py'" C-m
  tmux send-keys -t "$SESSION":0.4 "bash -lc '$cmd_env ros2 launch lane_selector lane_selector_launch.py'" C-m

  # (4) 타일 레이아웃
  tmux select-layout -t "$SESSION":0 tiled

  echo "[INFO] tmux attach -t $SESSION 로 확인하세요."
else
  echo "[INFO] tmux 미설치 → 백그라운드 실행(logs/ 저장)"
  mkdir -p "$WS_DIR/logs"
  ts=$(date +%Y%m%d_%H%M%S)

  run_bg () {
    local name="$1"; shift
    nohup bash -lc "$cmd_env $*" > "$WS_DIR/logs/${ts}_${name}.log" 2>&1 &
    echo "[BG] $name → PID $! (log: $WS_DIR/logs/${ts}_${name}.log)"
  }

  run_bg ring_obs     "ros2 launch obs_detect ring_obs.launch.py"
  run_bg localization "ros2 launch slam_nav localization_launch.py"
  run_bg map_control  "ros2 launch map_control map_controller.launch.py"
  run_bg lane_selector "ros2 launch lane_selector lane_selector_launch.py"

  echo "[INFO] 백그라운드 실행 완료. ps -ef | grep ros2 로 확인."
fi
