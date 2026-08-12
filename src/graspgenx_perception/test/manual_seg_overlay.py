#!/usr/bin/env python3
"""저장된 씬의 seg.png 를 rgb.png 위에 얹어 눈으로 확인한다.

    python3 src/graspgenx_perception/test/manual_seg_overlay.py output/00

`/yolo_seg/overlay` 는 **라이브 컬러 스트림**이라 캡처 순간과 다를 수 있고, 무엇보다
작업공간 박스·반경 크롭·min_pixels 로 걸러진 **뒤**의 라벨을 보여주지 않는다.
GraspGenX 가 실제로 먹은 것은 seg.png 다 — 그걸 그대로 그린다.

ROS 를 import 하지 않는다. 로봇/카메라가 꺼져 있어도 돈다.
"""

import argparse
import json
import os

import cv2
import numpy as np


def overlay(scene_dir, alpha=0.5):
    rgb = cv2.imread(os.path.join(scene_dir, 'rgb.png'))
    seg = cv2.imread(os.path.join(scene_dir, 'seg.png'), cv2.IMREAD_UNCHANGED)
    if rgb is None or seg is None:
        raise FileNotFoundError(f'{scene_dir} 에 rgb.png / seg.png 가 없다')
    with open(os.path.join(scene_dir, 'meta_data.json')) as f:
        label_map = json.load(f)['label_map']

    out = rgb.copy()
    rng = np.random.default_rng(3)          # 씬마다 같은 색이 나오게 고정
    lines = []
    for name, v in sorted(label_map.items(), key=lambda kv: kv[1]):
        if v == 0:                          # ground 는 배경이라 안 칠한다
            continue
        m = seg == v
        px = int(m.sum())
        lines.append(f'  {name:10s} = {v:3d} : {px:6d} px')
        if px == 0:
            continue
        col = rng.integers(60, 255, 3)
        out[m] = (alpha * out[m] + (1 - alpha) * col).astype(np.uint8)
        ys, xs = np.nonzero(m)
        # 중앙값 — 흩어진 마스크에서 평균은 아무 데도 없는 곳을 가리킨다
        pt = (int(np.median(xs)) - 40, int(np.median(ys)))
        for color, thick in (((0, 0, 0), 3), ((255, 255, 255), 1)):
            cv2.putText(out, f'{name} {px}px', pt,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thick)
    return out, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scene_dir', help='예: output/00')
    ap.add_argument('-o', '--out', default=None, help='기본: <scene_dir>/seg_overlay.png')
    args = ap.parse_args()

    img, lines = overlay(args.scene_dir)
    out = args.out or os.path.join(args.scene_dir, 'seg_overlay.png')
    if not cv2.imwrite(out, img):
        raise IOError(f'{out} 쓰기 실패')
    print(f'{args.scene_dir} 라벨:')
    print('\n'.join(lines))
    print(f'저장: {out}')


if __name__ == '__main__':
    raise SystemExit(main())
