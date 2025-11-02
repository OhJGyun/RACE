# Boundary Editor GUI Tools

두 가지 인터랙티브 GUI 툴을 제공합니다:

## 1. 📝 Manual Editor (boundary_editor_gui.py)

수동으로 포인트를 클릭해서 선택/제거하는 툴

### 사용법:

```bash
cd /home/ojg/RACE/trajectory_generator

# 기본 사용
python3 boundary_editor_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv

# 출력 파일 지정
python3 boundary_editor_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv -o /home/ojg/RACE/bound/1031_1/outer_bound_manual.csv
```

### 컨트롤:

| 키/마우스 | 기능 |
|-----------|------|
| **LEFT CLICK** | 포인트 선택/제거 토글 |
| **a** | 모든 포인트 선택 |
| **c** | 모든 포인트 제거 |
| **i** | 선택 반전 |
| **r** | 원본으로 리셋 |
| **s** | 저장하고 종료 |
| **q** | 저장 없이 종료 |

### 특징:
- ✅ 완전 수동 컨트롤
- ✅ 포인트별 정밀 선택
- ✅ 녹색 = 선택됨, 빨간색 = 제거됨
- ✅ 실시간 통계 표시

---

## 2. 🤖 Auto Simplifier (boundary_simplifier_gui.py)

Douglas-Peucker 자동 단순화 + 수동 조정

### 사용법:

```bash
cd /home/ojg/RACE/trajectory_generator

# 기본 사용 (tolerance 0.10m)
python3 boundary_simplifier_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv

# Tolerance 지정
python3 boundary_simplifier_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv -t 0.15

# 출력 파일 지정
python3 boundary_simplifier_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv -o custom_output.csv
```

### 컨트롤:

| 컨트롤 | 기능 |
|--------|------|
| **Slider** | Tolerance 조정 (0.01 ~ 0.50m) |
| **LEFT CLICK** | 자동 단순화된 포인트 추가 제거 |
| **RESET 버튼** | 수동 편집 취소 |
| **SAVE 버튼** | 저장하고 종료 |
| **s 키** | 저장하고 종료 |
| **q 키** | 저장 없이 종료 |

### 특징:
- ✅ Douglas-Peucker 자동 단순화
- ✅ 실시간 tolerance 조정 슬라이더
- ✅ 자동 단순화 후 수동 미세 조정 가능
- ✅ 원본 포인트 반투명 표시 (비교용)
- ✅ 속도 향상 배율 실시간 표시

---

## 📊 어떤 툴을 사용해야 하나?

### Manual Editor 사용:
- 완전히 수동으로 컨트롤하고 싶을 때
- 특정 구간만 선택적으로 제거
- 소규모 편집

### Auto Simplifier 사용 (권장 ✅):
- 빠르게 대량의 포인트 제거
- Douglas-Peucker 알고리즘 활용
- Tolerance 값 실험하면서 조정
- 자동 단순화 + 수동 미세조정

---

## 🎯 추천 워크플로우

### 1단계: Auto Simplifier로 대략적 단순화

```bash
python3 boundary_simplifier_gui.py /home/ojg/RACE/bound/1031_1/outer_bound.csv
```

- Slider로 tolerance 조정 (0.10m 권장)
- 실시간으로 결과 확인
- 마음에 들면 SAVE

### 2단계 (선택): Manual Editor로 미세 조정

```bash
python3 boundary_editor_gui.py /home/ojg/RACE/bound/1031_1/outer_bound_simplified.csv
```

- 특정 포인트만 추가 제거
- 코너 부분 정밀 조정

---

## 📁 파일 출력

### 기본 출력 파일명:

**Manual Editor:**
```
input: outer_bound.csv
output: outer_bound_edited.csv
```

**Auto Simplifier:**
```
input: outer_bound.csv
output: outer_bound_simplified.csv
```

### 커스텀 출력:

```bash
python3 boundary_simplifier_gui.py outer_bound.csv -o my_custom_name.csv
```

---

## 🚀 실제 사용 예시

### 예시 1: Outer boundary 단순화

```bash
cd /home/ojg/RACE/trajectory_generator

# Auto simplifier 실행
python3 boundary_simplifier_gui.py \
    /home/ojg/RACE/bound/1031_1/outer_bound.csv \
    -t 0.10 \
    -o /home/ojg/RACE/bound/1031_1/outer_bound_optimized.csv
```

**결과:**
- 198개 → 14개 포인트
- 7.8배 속도 향상
- 10cm 정확도

### 예시 2: Inner boundary 단순화

```bash
python3 boundary_simplifier_gui.py \
    /home/ojg/RACE/bound/1031_1/inner_bound.csv \
    -t 0.10 \
    -o /home/ojg/RACE/bound/1031_1/inner_bound_optimized.csv
```

**결과:**
- 131개 → 28개 포인트
- 4.7배 속도 향상

### 예시 3: 둘 다 처리 (스크립트)

```bash
#!/bin/bash
BOUND_DIR=/home/ojg/RACE/bound/1031_1

# Outer
python3 boundary_simplifier_gui.py \
    ${BOUND_DIR}/outer_bound.csv \
    -t 0.10 \
    -o ${BOUND_DIR}/outer_bound_opt.csv

# Inner
python3 boundary_simplifier_gui.py \
    ${BOUND_DIR}/inner_bound.csv \
    -t 0.10 \
    -o ${BOUND_DIR}/inner_bound_opt.csv

echo "✅ Done! Check ${BOUND_DIR}/"
```

---

## 💡 Tolerance 선택 가이드

| Tolerance | 점 감소율 | 속도 향상 | 정확도 | 권장 용도 |
|-----------|-----------|-----------|--------|-----------|
| **0.05m** | ~80% | 5x | ⭐⭐⭐⭐⭐ | 고정밀 필요 시 |
| **0.10m** | ~87% | 7.8x | ⭐⭐⭐⭐ | **일반 주행 (권장)** ✅ |
| **0.15m** | ~90% | 9.7x | ⭐⭐⭐ | 고속 주행 |
| **0.20m** | ~93% | 11x | ⭐⭐ | 매우 빠른 처리 필요 시 |

### 판단 기준:
- 차량 폭: ~50cm
- LiDAR 정확도: ±3cm
- **권장: 0.10m** (차량 크기의 1/5, 충분히 안전)

---

## 🔧 ring_viz_node에 적용

단순화된 boundary를 ring_viz_node에서 사용:

### 방법 1: CSV 파일 교체

```bash
# 백업
cp /home/ojg/RACE/bound/1031_1/outer_bound.csv \
   /home/ojg/RACE/bound/1031_1/outer_bound_original.csv

# 최적화된 버전으로 교체
cp /home/ojg/RACE/bound/1031_1/outer_bound_simplified.csv \
   /home/ojg/RACE/bound/1031_1/outer_bound.csv
```

### 방법 2: 파라미터 파일 수정

```yaml
# ring_viz_params.yaml
simple_scan_viz:
  ros__parameters:
    outer_bound_csv: "/home/ircv7/RACE/bound/1031_1/outer_bound_simplified.csv"
    inner_bound_csv: "/home/ircv7/RACE/bound/1031_1/inner_bound_simplified.csv"
```

---

## 📈 예상 성능 향상

### Before:
```
Outer: 198 points
Inner: 131 points
Total: 329 edges
Processing time: 2.63ms (8 clusters)
```

### After (0.10m tolerance):
```
Outer: 14 points
Inner: 28 points
Total: 42 edges
Processing time: 0.34ms (8 clusters)
Speedup: 7.8x ⚡⚡⚡
```

### 10Hz LiDAR 기준:
```
초당 절약: (2.63 - 0.34) × 10 = 22.9ms
→ 다른 알고리즘에 사용 가능한 CPU 시간!
```

---

## ⚠️ 주의사항

1. **최소 3개 포인트 필요**
   - 폴리곤은 최소 3개 꼭짓점 필요
   - 너무 많이 제거하면 경고

2. **급커브 조심**
   - Tolerance 너무 크면 코너 손실
   - 코너 많은 트랙은 0.05~0.10m 권장

3. **원본 백업**
   - 항상 원본 파일 백업
   - 여러 tolerance 값 실험 권장

4. **실제 주행 테스트**
   - 시뮬레이션에서 먼저 테스트
   - 실제 차량에서 안전 확인

---

## 🐛 트러블슈팅

### Q: GUI가 안 뜨는 경우?

```bash
# matplotlib 백엔드 확인
python3 -c "import matplotlib; print(matplotlib.get_backend())"

# 백엔드 설정
export MPLBACKEND=TkAgg
```

### Q: "No module named matplotlib"?

```bash
pip3 install matplotlib numpy
```

### Q: 클릭이 안 먹히는 경우?

- 포인트 근처(0.5m 이내)를 클릭해야 함
- 줌 인/아웃 후 다시 시도

### Q: 저장 안 되는 경우?

- 권한 확인: `ls -la /home/ojg/RACE/bound/1031_1/`
- 출력 디렉토리 존재 확인

---

## �� 도움말

더 자세한 사용법:
```bash
python3 boundary_editor_gui.py --help
python3 boundary_simplifier_gui.py --help
```

Happy editing! 🎨✨
