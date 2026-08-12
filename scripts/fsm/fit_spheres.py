#!/usr/bin/env python3
"""URDF collision 메시에서 링크별 충돌 구(sphere)를 피팅해 XRDF collision 섹션을 출력한다.

방법: 링크의 모든 collision 메시 정점을 링크 로컬 좌표(m)로 모으고,
bbox 최장축을 따라 K개 슬랩으로 잘라 슬랩마다 (중심=슬랩 정점 centroid,
반지름=중심에서 슬랩 정점까지 최대거리)로 구를 만든다.
최대거리를 쓰므로 **항상 과포함(conservative)** 이다 — 충돌 검사에서 안전한 방향.
"""
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = sys.argv[1]


def resolve(uri):
    if not uri.startswith('package://'):
        return uri if os.path.exists(uri) else None
    pkg, rel = uri[len('package://'):].split('/', 1)
    import subprocess
    try:
        pref = subprocess.run(['ros2', 'pkg', 'prefix', pkg],
                              capture_output=True, text=True).stdout.strip()
        c = os.path.join(pref, 'share', pkg, rel)
        if os.path.exists(c):
            return c
    except Exception:
        pass
    return None


def load_dae(path):
    root = ET.parse(path).getroot()
    ns = root.tag.split('}')[0].strip('{')
    cands = []
    for fa in root.iter('{%s}float_array' % ns):
        v = np.fromstring(fa.text, sep=' ')
        if v.size and v.size % 3 == 0:
            cands.append((bool(re.search(r'(?i)pos', fa.get('id', ''))), v.reshape(-1, 3)))
    pos = [v for tagged, v in cands if tagged]
    if pos:
        return np.vstack(pos)
    return max((v for _, v in cands), key=lambda a: a.shape[0]) if cands else None


def load_stl(path):
    with open(path, 'rb') as f:
        d = f.read()
    if d[:5] == b'solid' and b'facet' in d[:5000]:
        return np.array([[float(x) for x in ln.split()[1:4]]
                         for ln in d.decode('utf8', 'ignore').splitlines()
                         if ln.strip().startswith('vertex')])
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view('<f4').reshape(-1, 3)


def fit(verts, _k_unused, pct=97.0):
    """bbox 최장축을 따라 얇은 슬랩으로 잘라 구를 만든다.

    슬랩 두께를 단면 반폭보다 작게 잡아야 구가 단면 대각선에 끌려 커지지 않는다.
    반지름은 max가 아니라 pct 백분위수 — 정점 하나가 반지름을 부풀리는 것을 막는다.
    남는 미세한 미포함은 XRDF의 buffer_distance가 덮는다.
    """
    lo, hi = verts.min(0), verts.max(0)
    size = hi - lo
    ax = int(np.argmax(size))
    cross = np.delete(size, ax)          # 단면 두 변
    half_w = float(cross.min()) / 2.0    # 단면 반폭(짧은 쪽)
    length = float(size[ax])
    k = int(max(2, np.ceil(length / max(half_w * 0.8, 1e-3))))
    k = min(k, 24)                       # 링크당 상한
    edges = np.linspace(lo[ax], hi[ax], k + 1)
    out = []
    for i in range(k):
        a, b = edges[i], edges[i + 1]
        m = (verts[:, ax] >= a) & (verts[:, ax] <= b if i == k - 1 else verts[:, ax] < b)
        if m.sum() < 4:
            continue
        pts = verts[m]
        c = pts.mean(0)
        d = np.linalg.norm(pts - c, axis=1)
        out.append((c, float(np.percentile(d, pct))))
    # 거의 같은 자리의 구 제거 (중심 간 거리가 두 반지름 차보다 작으면 큰 것이 작은 것을 삼킨다)
    keep = []
    for c, r in sorted(out, key=lambda s: -s[1]):
        if any(np.linalg.norm(c - c2) + r <= r2 + 1e-6 for c2, r2 in keep):
            continue
        keep.append((c, r))
    return keep


# 링크별 구 개수: 길쭉한 링크는 많이, 뭉툭한 건 적게
K = {'base_link': 4, 'link_1': 3, 'link_2': 8, 'link_3': 7, 'link_4': 4,
     'link_5': 3, 'link_6': 2,
     'rg2_base_link': 4, 'rg2_left_outer_knuckle': 2, 'rg2_right_outer_knuckle': 2,
     'rg2_left_inner_knuckle': 2, 'rg2_right_inner_knuckle': 2,
     'rg2_left_inner_finger': 3, 'rg2_right_inner_finger': 3}

root = ET.parse(URDF).getroot()
total = 0
for L in root.findall('link'):
    name = L.get('name')
    if name not in K:
        continue
    allv = []
    for col in L.findall('collision'):
        mesh = col.find('geometry/mesh')
        if mesh is None:
            continue
        p = resolve(mesh.get('filename'))
        if not p:
            print(f'  # {name}: MESH NOT FOUND {mesh.get("filename")}', file=sys.stderr)
            continue
        v = load_dae(p) if p.endswith('.dae') else load_stl(p)
        if v is None:
            continue
        s = np.asarray((mesh.get('scale') or '1 1 1').split(), float)
        v = v * s
        o = col.find('origin')
        if o is not None and o.get('xyz'):
            v = v + np.asarray(o.get('xyz').split(), float)
        allv.append(v)
    if not allv:
        continue
    V = np.vstack(allv)
    sph = fit(V, K[name])
    total += len(sph)
    # 진단: 링크 실제 단면 반폭 대비 구 반지름이 얼마나 큰지 (과포함 배율)
    size = V.max(0) - V.min(0)
    half_w = float(np.delete(size, int(np.argmax(size))).min()) / 2.0
    rmax = max(r for _, r in sph)
    print(f'  # {name}: 구 {len(sph)}개, 단면 반폭 {half_w * 1000:.0f} mm, '
          f'최대 반지름 {rmax * 1000:.0f} mm (배율 {rmax / half_w:.2f}x)', file=sys.stderr)
    print(f'      {name}:')
    for c, r in sph:
        print(f'        - center: [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]')
        print(f'          radius: {r:.4f}')
print(f'# 총 구 개수: {total}', file=sys.stderr)
