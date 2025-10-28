#!/usr/bin/env python3
import argparse, os, csv, sys
import cv2
import numpy as np
import yaml

# --------------------
# I/O & utils
# --------------------
def load_map(pgm_path, yaml_path=None):
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"PGM 읽기 실패: {pgm_path}")
    # 16bit PGM -> 8bit
    if img.dtype != np.uint8:
        img = cv2.convertScaleAbs(img, alpha=(255.0 / max(1, img.max())))
    meta = {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]}
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, "r") as f:
            y = yaml.safe_load(f)
            meta["resolution"] = float(y["resolution"])
            meta["origin"] = list(y["origin"])  # [x, y, yaw]
    return img, meta

def binarize_walls(img, thresh=100, invert=False, close_iter=2):
    """
    기본 가정: 벽=검정(0), 자유=흰(255).
    thresh 기준 아래(검정쪽)를 '벽=255'로 만들기 위해 THRESH_BINARY_INV 사용.
    """
    if thresh < 0:  # Otsu
        _, bw = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, bw = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY_INV)
    if invert:
        bw = 255 - bw
    if close_iter > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return bw  # 벽=255, 자유=0

def _find_contours(img, mode, method):
    out = cv2.findContours(img, mode, method)
    if len(out) == 3:
        _, contours, hierarchy = out
    else:
        contours, hierarchy = out
    if hierarchy is not None and len(hierarchy.shape) == 3:
        hierarchy = hierarchy[0]
    return list(contours), hierarchy

def free_space_outer_and_inners(free_mask, min_perimeter=200):
    """
    free_mask: 흰색=주행 가능한 영역(= 255 - walls)
    반환:
      outer: np.ndarray (Mx1x2) 자유공간의 바깥 경계 (하나)
      inners: List[np.ndarray] 자유공간 안의 '구멍' 경계들(여러 개, 0개 가능)
    """
    contours, hierarchy = _find_contours(free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        raise RuntimeError("자유공간 컨투어가 없음 (이진화/threshold 조정 필요)")

    # parent == -1 : 최상위 자유공간. 그중 면적 최대를 outer로.
    top = [(i, c) for i, c in enumerate(contours)
           if hierarchy[i][3] == -1 and cv2.arcLength(c, True) >= min_perimeter]
    if not top:
        top = [(i, c) for i, c in enumerate(contours) if hierarchy[i][3] == -1]
    if not top:
        raise RuntimeError("자유공간 최상위 컨투어가 없음")

    top.sort(key=lambda x: cv2.contourArea(x[1]), reverse=True)
    outer_idx, outer = top[0]

    # outer의 child 들은 구멍(내부 섬) → inner 경계들
    def collect_children(idx):
        res = []
        child = hierarchy[idx][2]
        while child != -1:
            res.append(child)
            child = hierarchy[child][0]  # next sibling
        return res

    inner_ids = collect_children(outer_idx)
    inners = []
    for j in inner_ids:
        cj = contours[j]
        if cv2.arcLength(cj, True) >= min_perimeter:
            inners.append(cj)

    return outer, inners

def make_inner_by_offset(walls_mask, resolution, inner_offset_m=0.3):
    """
    거리변환 기반 inner 생성(백업 플랜):
    - free = NOT walls
    - distanceTransform(free)로 벽에서의 거리 맵 생성
    - dt > offset_px 영역의 외곽선을 inner로 사용
    """
    walls = (walls_mask > 0).astype(np.uint8) * 255  # 벽=255
    free  = (255 - walls)                           # 자유=255

    dt = cv2.distanceTransform(free, distanceType=cv2.DIST_L2, maskSize=5)  # 픽셀 단위 거리
    offset_px = max(1, int(inner_offset_m / max(1e-9, resolution)))

    _, inner_mask = cv2.threshold(dt, float(offset_px), 255, cv2.THRESH_BINARY)
    inner_mask = inner_mask.astype(np.uint8)

    contours, _ = _find_contours(inner_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        return None
    contours.sort(key=lambda c: cv2.contourArea(c), reverse=True)
    return contours[0]  # 하나만 리턴

def resample_closed_contour(contour, step_px=5):
    pts = contour[:, 0, :].astype(np.float32)
    if not np.array_equal(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[0]])
    seg = np.diff(pts, axis=0)
    seglen = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seglen)])
    total = float(cum[-1])
    if total < 1e-6:
        return pts.astype(int)
    samples = np.arange(0, total, float(step_px))
    res = []
    for s in samples:
        i = max(0, min(int(np.searchsorted(cum, s) - 1), len(seg) - 1))
        t = (s - cum[i]) / max(seglen[i], 1e-9)
        p = pts[i] + t * (pts[i + 1] - pts[i])
        res.append(p)
    return np.asarray(res, dtype=int)

def pixels_to_world(pix_xy, img_h, resolution, origin):
    ox, oy, _ = origin
    wpts = []
    for (px, py) in pix_xy:
        wx = ox + float(px) * resolution
        wy = oy + float(img_h - py) * resolution
        wpts.append((wx, wy))
    return wpts

def save_csv_pixels(path, pts_xy_int):
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        for p in pts_xy_int:
            wr.writerow([int(p[0]), int(p[1])])

def save_csv_world(path, wpts):
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        for (x, y) in wpts:
            wr.writerow([f"{x:.6f}", f"{y:.6f}"])

def choose_output_paths(default_outer, default_inner, enable_gui=True):
    """
    GUI를 통해 출력 파일 경로를 선택. GUI 사용이 불가능하거나 비활성화되면 기본 경로를 그대로 반환.
    """
    if not enable_gui:
        return default_outer, default_inner
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception as exc:
        print(f"[경고] GUI 사용 불가: {exc}. 기본 파일명으로 저장합니다.", file=sys.stderr)
        return default_outer, default_inner

    root = tk.Tk()
    root.withdraw()
    try:
        outer_selected = filedialog.asksaveasfilename(
            title="outer 경계 CSV 저장 위치 선택",
            initialdir=os.path.dirname(default_outer),
            initialfile=os.path.basename(default_outer),
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if outer_selected:
            default_outer = outer_selected

        inner_selected = filedialog.asksaveasfilename(
            title="inner 경계 CSV 저장 위치 선택",
            initialdir=os.path.dirname(default_inner),
            initialfile=os.path.basename(default_inner),
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if inner_selected:
            default_inner = inner_selected
    finally:
        root.destroy()
    return default_outer, default_inner

def clearance_contours_from_free(free_mask, clearance_m, resolution, min_perimeter=200):
    """
    free_mask: 자유=255, 벽=0
    clearance_m 만큼 자유공간을 침식 → 그 결과의 경계를 리턴
    returns: outer_contour, [inner_contours...]
    """
    if clearance_m <= 0:
        eroded = free_mask.copy()
    else:
        px = max(1, int(round(clearance_m / max(1e-9, resolution))))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*px+1, 2*px+1))
        eroded = cv2.erode(free_mask, kernel, iterations=1)

    # 가장 큰 자유공간 컴포넌트만 사용 (노이즈 제거)
    contours, hierarchy = cv2.findContours(eroded, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        raise RuntimeError("침식 후 자유공간이 없음(너무 큰 clearance?).")

    hierarchy = hierarchy[0]  # (N,4)
    # 최상위(parent==-1) 중 면적 최대를 고름
    tops = [(i, c) for i, c in enumerate(contours) if hierarchy[i][3] == -1]
    if not tops:
        raise RuntimeError("최상위 자유공간이 없음.")
    tops.sort(key=lambda x: cv2.contourArea(x[1]), reverse=True)
    root_idx, outer = tops[0]

    # 둘레 조건 적용
    if cv2.arcLength(outer, True) < min_perimeter:
        raise RuntimeError("outer 둘레가 너무 작음(min_perimeter 늘리기).")

    # root의 child들이 '구멍' → inner들
    inners = []
    child = hierarchy[root_idx][2]
    while child != -1:
        c = contours[child]
        if cv2.arcLength(c, True) >= min_perimeter:
            inners.append(c)
        child = hierarchy[child][0]  # next sibling

    return outer, inners

# --------------------
# main
# --------------------
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_outdir = os.path.normpath(os.path.join(script_dir, "..", "bounds"))

    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=False, help="map .pgm")
    ap.add_argument("--yaml", required=False, help="map .yaml (origin/resolution)")
    ap.add_argument("--step_px", type=int, default=5, help="샘플링 간격(픽셀)")
    ap.add_argument("--min_perimeter", type=int, default=300, help="최소 컨투어 둘레")
    ap.add_argument("--outdir", default=default_outdir, help="CSV/디버그 출력 폴더")
    ap.add_argument("--no_gui", action="store_true", help="파일 저장 GUI 비활성화")
    ap.add_argument("--invert", action="store_true", help="벽/자유색 반전")
    ap.add_argument("--thresh", type=int, default=100, help="이진화 임계값(음수면 Otsu)")
    ap.add_argument("--close_iter", type=int, default=2, help="morph close 반복")
    ap.add_argument("--inner_offset_m", type=float, default=0.6, help="백업 inner 오프셋(m)")
    ap.add_argument("--save_world", action="store_true", default=True, help="월드좌표 CSV도 저장 (기본값: 활성화)")
    ap.add_argument("--clearance_m", type=float, default=0.25,
                help="벽에서 떨어질 여유 (m). 이만큼 벽을 팽창시켜 자유공간을 줄입니다.")
    args = ap.parse_args()

    if not args.pgm:
        # Allow choosing a PGM interactively when not provided via CLI.
        cwd = os.getcwd()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        search_dirs = []
        for d in (cwd, script_dir):
            if d not in search_dirs:
                search_dirs.append(d)

        candidates = []
        for base in search_dirs:
            for entry in sorted(os.listdir(base)):
                if entry.lower().endswith(".pgm"):
                    candidates.append(os.path.join(base, entry))

        if not candidates:
            raise RuntimeError("PGM 파일을 찾지 못했습니다. --pgm 옵션으로 직접 지정하세요.")

        print("선택 가능한 PGM 파일 목록:")
        for idx, path in enumerate(candidates, 1):
            rel = os.path.relpath(path, cwd)
            print(f"  [{idx}] {rel}")

        while True:
            choice = input("사용할 PGM 번호를 입력하세요 (취소:q): ").strip()
            if choice.lower() in {"q", "quit"}:
                raise SystemExit("사용자가 선택을 취소했습니다.")
            if choice.isdigit():
                sel = int(choice)
                if 1 <= sel <= len(candidates):
                    args.pgm = candidates[sel - 1]
                    break
            print("잘못된 선택입니다. 다시 입력하세요.")

        if not args.yaml:
            base = os.path.splitext(args.pgm)[0]
            for ext in (".yaml", ".yml"):
                cand = base + ext
                if os.path.exists(cand):
                    args.yaml = cand
                    print(f"연관된 YAML 자동 선택: {os.path.relpath(cand, cwd)}")
                    break

    args.outdir = os.path.abspath(args.outdir)
    os.makedirs(args.outdir, exist_ok=True)
    default_outer_csv = os.path.join(args.outdir, "outer_bound_pixels.csv")
    default_inner_csv = os.path.join(args.outdir, "inner_bound_pixels.csv")
    outer_csv, inner_csv = choose_output_paths(
        default_outer_csv,
        default_inner_csv,
        enable_gui=not getattr(args, "no_gui", False) if hasattr(args, "no_gui") else True
    )
    outer_csv = os.path.abspath(outer_csv)
    inner_csv = os.path.abspath(inner_csv)
    for path in (outer_csv, inner_csv):
        out_dir = os.path.dirname(path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

    img, meta = load_map(args.pgm, args.yaml)
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    H, W = img.shape[:2]

    # 벽 이진화 (벽=255, 자유=0)
    # 1) 벽 이진화 (벽=255, 자유=0)
    walls = binarize_walls(img, thresh=args.thresh, invert=args.invert, close_iter=args.close_iter)

    # 2) 자유공간 생성(자유=255)
    free = 255 - walls

    # 3) 원하는 여유거리만큼 정확히 물러난 경계 얻기
    outer, inner_list = clearance_contours_from_free(
        free_mask=free,
        clearance_m=args.clearance_m,   # <- 네가 조절
        resolution=resolution,
        min_perimeter=args.min_perimeter
    )

    # 4) 리샘플링 & 저장/시각화
    outer_s  = resample_closed_contour(outer, step_px=args.step_px)
    inners_s = [resample_closed_contour(c, step_px=args.step_px) for c in inner_list]
    all_inner = (np.vstack(inners_s) if inners_s else np.zeros((0,2), dtype=int))

    # 디버그 이미지(확인용)
    dbg = cv2.cvtColor(free, cv2.COLOR_GRAY2BGR)  # eroded가 아니라 원 free 위에 그려도 됨
    cv2.drawContours(dbg, [outer], -1, (0,0,255), 1)   # 빨강=outer
    for c in inner_list:
        cv2.drawContours(dbg, [c], -1, (0,255,0), 1)   # 초록=inner들
    debug_dir = os.path.dirname(outer_csv) or args.outdir
    os.makedirs(debug_dir, exist_ok=True)
    cv2.imwrite(os.path.join(debug_dir, "bounds_debug.png"), dbg)

    # CSV 저장 (픽셀/월드)
    save_csv_pixels(outer_csv, outer_s)
    save_csv_pixels(inner_csv, all_inner)
    if args.save_world:
        outer_base = os.path.splitext(outer_csv)[0]
        inner_base = os.path.splitext(inner_csv)[0]
        outer_w = pixels_to_world(outer_s, H, resolution, origin)
        inner_w = pixels_to_world(all_inner, H, resolution, origin)
        save_csv_world(f"{outer_base}_world.csv", outer_w)
        save_csv_world(f"{inner_base}_world.csv", inner_w)


    print(f"[OK] 저장 완료 -> {outer_csv}, {inner_csv}")
    if args.save_world:
        print(f"[OK] 월드좌표도 저장 -> {outer_base}_world.csv, {inner_base}_world.csv")
    print(f"이미지 크기: {W}x{H}, resolution={resolution}, origin={origin}")

if __name__ == "__main__":
    main()
