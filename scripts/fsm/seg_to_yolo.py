#!/usr/bin/env python3
"""capture_graspgenx_scene.py 가 뱉은 seg.png(라벨맵) -> YOLO-seg 학습 라벨.

마스크는 만들지 않는다. capture 스크립트가 depth(작업공간 박스 + 테이블면 위 obj_min_h)로
이미 만들어 뒀다 — SAM/YOLOE/사람 클릭 0회. 여기서는 폴리곤으로 바꾸기만 한다.

    # 물체 하나만 테이블에 놓고 위치·자세 바꿔가며 scene 을 여러 개 캡처한 뒤:
    python3 scripts/seg_to_yolo.py data/graspgenx_scene dataset --cls 0 --name bolt

geometric seg 는 "덩어리"만 알지 클래스는 모른다. 그래서 한 번에 한 종류만 놓고 캡처하고
종류마다 --cls 를 바꿔 돌린다. 사람이 하는 일은 물체 종류당 --cls 지정 1회가 전부다.

ponytail: 덩어리당 외곽선 1개만 쓴다(구멍·떨어진 조각 무시). 도넛 모양 물체를 학습시킬 때 다시 본다.
"""

import argparse
import json
import pathlib
import shutil

import cv2
import numpy as np

LABEL_OBJ_BASE = 100  # capture_graspgenx_scene.py:100 과 같은 규약 (obj_1 -> 101)


def polygons(seg, min_pixels=300, eps_ratio=0.002):
    """uint8 라벨맵 -> [정규화된 폴리곤 (x1,y1,x2,y2,...)]. 물체 라벨(>100)만 본다."""
    h, w = seg.shape
    out = []
    for v in np.unique(seg):
        if v <= LABEL_OBJ_BASE:
            continue  # 0=ground, 2=table
        m = (seg == v).astype(np.uint8)
        if int(m.sum()) < min_pixels:
            continue
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        c = cv2.approxPolyDP(c, eps_ratio * cv2.arcLength(c, True), True).reshape(-1, 2)
        if len(c) < 3:
            continue
        out.append((c / (w, h)).clip(0.0, 1.0).reshape(-1))
    return out


def convert(src, dst, cls, min_pixels):
    """<src>/<scene>/{rgb.png,seg.png} -> <dst>/{images,labels}/train/<scene>.*"""
    img_dir = dst / 'images' / 'train'
    lbl_dir = dst / 'labels' / 'train'
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    n_scene = n_obj = 0
    for seg_path in sorted(src.glob('*/seg.png')):
        rgb_path = seg_path.with_name('rgb.png')
        if not rgb_path.exists():
            print(f'  skip {seg_path.parent.name}: rgb.png 없음')
            continue
        seg = cv2.imread(str(seg_path), cv2.IMREAD_UNCHANGED)
        polys = polygons(seg, min_pixels)
        if not polys:
            print(f'  skip {seg_path.parent.name}: 물체 라벨 0개 (obj_min_h/bounds 확인)')
            continue
        # scene 디렉토리명이 '00' 처럼 짧아 batch 를 여러 번 돌리면 덮어쓴다 -> src 이름을 접두사로
        stem = f'{src.name}_{seg_path.parent.name}'
        shutil.copyfile(rgb_path, img_dir / f'{stem}.png')
        (lbl_dir / f'{stem}.txt').write_text(
            '\n'.join(f'{cls} ' + ' '.join(f'{x:.6f}' for x in p) for p in polys) + '\n')
        n_scene += 1
        n_obj += len(polys)
    return n_scene, n_obj


def self_check():
    seg = np.zeros((100, 200), np.uint8)
    seg[10:90, 20:60] = 2                 # table -> 무시돼야 한다
    seg[20:60, 100:140] = LABEL_OBJ_BASE + 1
    seg[0:2, 0:2] = LABEL_OBJ_BASE + 2    # 4px, min_pixels 미만 -> 버려져야 한다
    polys = polygons(seg, min_pixels=300)
    assert len(polys) == 1, polys
    xs, ys = polys[0][0::2], polys[0][1::2]
    # 외곽선은 채워진 마지막 픽셀을 가리킨다: 열 100..139 -> 0.500/0.695, 행 20..59 -> 0.20/0.59
    assert abs(xs.min() - 0.500) < 1e-6 and abs(xs.max() - 0.695) < 1e-6, xs
    assert abs(ys.min() - 0.200) < 1e-6 and abs(ys.max() - 0.590) < 1e-6, ys
    assert polygons(np.zeros((10, 10), np.uint8)) == []
    print('self-check PASS')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src', nargs='?', type=pathlib.Path, help='캡처 루트 (<src>/<scene>/seg.png)')
    ap.add_argument('dst', nargs='?', type=pathlib.Path, help='출력 데이터셋 루트')
    ap.add_argument('--cls', type=int, default=0, help='이 배치 전체에 붙일 클래스 id')
    ap.add_argument('--name', default='', help='data.yaml 에 쓸 클래스 이름 (--cls 순서대로 누적)')
    ap.add_argument('--min-pixels', type=int, default=300)
    ap.add_argument('--self-check', action='store_true')
    a = ap.parse_args()

    if a.self_check:
        return self_check()
    if not a.src or not a.dst:
        ap.error('src 와 dst 가 필요하다 (또는 --self-check)')

    n_scene, n_obj = convert(a.src, a.dst, a.cls, a.min_pixels)
    print(f'{n_scene} scene / {n_obj} object -> {a.dst}')

    # data.yaml 은 --cls 를 바꿔 여러 번 돌리므로 이름을 누적해서 갱신한다
    names_path = a.dst / 'names.json'
    names = json.loads(names_path.read_text()) if names_path.exists() else {}
    names[str(a.cls)] = a.name or names.get(str(a.cls), f'class{a.cls}')
    names_path.write_text(json.dumps(names, ensure_ascii=False))
    lines = [f'path: {a.dst.resolve()}', 'train: images/train', 'val: images/train', 'names:']
    lines += [f'  {k}: {names[k]}' for k in sorted(names, key=int)]
    (a.dst / 'data.yaml').write_text('\n'.join(lines) + '\n')
    print(f'  data.yaml names={ {k: names[k] for k in sorted(names, key=int)} }')
    print('  ⚠️ val 이 train 과 같다 — mAP 를 믿지 말 것. 검증은 실기 장면으로 한다')


if __name__ == '__main__':
    main()
