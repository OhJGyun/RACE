#!/bin/zsh
################################################################################
# F1TENTH Race Full Stack Launch Script for Terminator
#
# 터미네이터에서 5개의 런치 파일을 각각의 탭에서 실행합니다:
# 1. SLAM & Localization
# 2. MAP Controller
# 3. Lane Selector
# 4. Ring Obstacle Detection
# 5. Boundary Visualization
#
# Usage:
#   cd /home/ircv7/RACE/real_ws
#   ./launch_all.sh
################################################################################

# ROS2 환경 소싱
source /opt/ros/humble/setup.zsh
source /home/ircv7/RACE/real_ws/install/setup.zsh

# 터미네이터가 설치되어 있는지 확인
if ! command -v terminator &> /dev/null; then
    echo "❌ Terminator가 설치되어 있지 않습니다."
    echo "설치 방법: sudo apt install terminator"
    exit 1
fi

echo "🚀 F1TENTH Race Full Stack 시작..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5개의 런치 파일을 Terminator의 각 탭에서 실행합니다:"
echo "  1. SLAM & Localization"
echo "  2. MAP Controller"
echo "  3. Lane Selector"
echo "  4. Ring Obstacle Detection"
echo "  5. Boundary Visualization"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Terminator 설정 파일 생성 (임시)
TERMINATOR_CONFIG="/tmp/f1tenth_terminator_config"

cat > "$TERMINATOR_CONFIG" << 'EOF'
[global_config]
  title_hide_sizetext = True
  title_transmit_bg_color = "#d30102"

[keybindings]

[profiles]
  [[default]]
    background_darkness = 0.9
    background_type = transparent
    cursor_color = "#aaaaaa"
    font = Monospace 11
    scrollback_infinite = True

[layouts]
  [[default]]
    [[[child0]]]
      type = Window
      parent = ""
      order = 0
      position = 0:0
      size = 1920, 1080
      title = F1TENTH Race Stack
    [[[child1]]]
      type = Notebook
      parent = child0
      order = 0
      labels = SLAM & Localization, MAP Controller, Lane Selector, Ring Detection, Boundary Viz
    [[[terminal1]]]
      type = Terminal
      parent = child1
      order = 0
      profile = default
      command = zsh -c "cd /home/ircv7/RACE/real_ws && source /opt/ros/humble/setup.zsh && source install/setup.zsh && echo '🗺️  [1/5] Starting SLAM & Localization...' && ros2 launch slam_nav localization_launch.py; exec zsh"
    [[[terminal2]]]
      type = Terminal
      parent = child1
      order = 1
      profile = default
      command = zsh -c "cd /home/ircv7/RACE/real_ws && source /opt/ros/humble/setup.zsh && source install/setup.zsh && sleep 3 && echo '🛣️  [2/5] Starting MAP Controller...' && ros2 launch map_control map_controller.launch.py launch_rviz:=false; exec zsh"
    [[[terminal3]]]
      type = Terminal
      parent = child1
      order = 2
      profile = default
      command = zsh -c "cd /home/ircv7/RACE/real_ws && source /opt/ros/humble/setup.zsh && source install/setup.zsh && sleep 5 && echo '🚦 [3/5] Starting Lane Selector...' && ros2 launch lane_selector lane_selector_launch.py; exec zsh"
    [[[terminal4]]]
      type = Terminal
      parent = child1
      order = 3
      profile = default
      command = zsh -c "cd /home/ircv7/RACE/real_ws && source /opt/ros/humble/setup.zsh && source install/setup.zsh && sleep 7 && echo '⭕ [4/5] Starting Ring Obstacle Detection...' && ros2 launch obs_detect ring_viz.launch.py; exec zsh"
    [[[terminal5]]]
      type = Terminal
      parent = child1
      order = 4
      profile = default
      command = zsh -c "cd /home/ircv7/RACE/real_ws && source /opt/ros/humble/setup.zsh && source install/setup.zsh && sleep 9 && echo '📊 [5/5] Starting Boundary Visualization...' && ros2 launch obs_detect boundary_viz.launch.py launch_rviz:=false; exec zsh"

[plugins]
EOF

# Terminator 실행
echo "✅ Terminator 실행 중..."
terminator --config="$TERMINATOR_CONFIG" --layout=default &

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ 모든 노드가 Terminator 탭에서 시작되었습니다!"
echo ""
echo "💡 사용 팁:"
echo "  - Ctrl+Shift+T: 새 탭 열기"
echo "  - Ctrl+PageUp/PageDown: 탭 전환"
echo "  - Ctrl+Shift+W: 현재 탭 닫기"
echo "  - Ctrl+Shift+Q: Terminator 종료"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
