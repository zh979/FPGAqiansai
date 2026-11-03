"""
ds_check.py -- 数据集体检

训练前必须先确认数据本身没问题。小数据集上, 几十个标注错误就足以
让模型学歪, 而且在 mAP 上表现为"模型不行", 很容易误判成算法问题。

检查项:
  1. 图/标注配对 -- 漏标的图会被 voc_annotation.py 静默跳过
  2. 类别名拼写 -- "weed"/"Weed"/"wed" 会被当成三个类
  3. 类别顺序 -- weed 必须是 0, 否则变成"打作物保护杂草"
  4. 框尺寸分布 -- 太小的框网络学不到
  5. 越界/退化框 -- 坐标超图或宽高为 0
  6. 每图目标数 -- 看场景覆盖度
"""
import os
import sys
import glob
import struct
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

NET = 416


def bmp_size(path):
    with open(path, 'rb') as f:
        head = f.read(26)
    if head[:2] != b'BM':
        return None
    w = struct.unpack_from('<i', head, 18)[0]
    h = struct.unpack_from('<i', head, 22)[0]
    return w, h


def main(d):
    bmps = sorted(glob.glob(os.path.join(d, '*.bmp')))
    xmls = sorted(glob.glob(os.path.join(d, '*.xml')))
    print('=' * 62)
    print('dataset: %s' % d)
    print('  images      %d' % len(bmps))
    print('  annotations %d' % len(xmls))

    bset = {os.path.splitext(os.path.basename(p))[0] for p in bmps}
    xset = {os.path.splitext(os.path.basename(p))[0] for p in xmls}

    no_ann = sorted(bset - xset)
    no_img = sorted(xset - bset)
    print()
    print('--- 1. pairing ---')
    if no_ann:
        print('  images WITHOUT annotation: %d' % len(no_ann))
        for n in no_ann[:20]:
            print('      %s' % n)
        if len(no_ann) > 20:
            print('      ... +%d more' % (len(no_ann) - 20))
        print('    -> these are silently skipped during training')
    else:
        print('  all images annotated')
    if no_img:
        print('  annotations WITHOUT image: %d  %s' % (len(no_img), no_img[:5]))

    # classes.txt
    print()
    print('--- 2. classes.txt ---')
    cf = os.path.join(d, 'classes.txt')
    declared = []
    if os.path.exists(cf):
        with open(cf, 'r', encoding='utf-8', errors='replace') as f:
            declared = [l.strip() for l in f if l.strip()]
        print('  %s' % declared)
        if declared and declared[0] != 'weed':
            print('    !! first line must be "weed" (code hardcodes YP_CLS_WEED=0)')
        if len(declared) != 2:
            print('    !! expected exactly 2 classes, got %d' % len(declared))
    else:
        print('  classes.txt missing')

    # parse annotations
    cls_cnt = Counter()
    per_img = []
    sizes = []          # (w_px, h_px) of boxes in image pixels
    bad_box = []
    img_dims = Counter()
    raw_names = Counter()

    for xp in xmls:
        stem = os.path.splitext(os.path.basename(xp))[0]
        try:
            root = ET.parse(xp).getroot()
        except Exception as e:
            bad_box.append((stem, 'XML parse error: %s' % e))
            continue
        sz = root.find('size')
        iw = int(float(sz.find('width').text)) if sz is not None else 0
        ih = int(float(sz.find('height').text)) if sz is not None else 0
        img_dims[(iw, ih)] += 1

        n = 0
        for obj in root.findall('object'):
            name = (obj.find('name').text or '').strip()
            raw_names[name] += 1
            cls_cnt[name] += 1
            bb = obj.find('bndbox')
            x1 = float(bb.find('xmin').text); y1 = float(bb.find('ymin').text)
            x2 = float(bb.find('xmax').text); y2 = float(bb.find('ymax').text)
            bw, bh = x2 - x1, y2 - y1
            if bw <= 1 or bh <= 1:
                bad_box.append((stem, 'degenerate %gx%g' % (bw, bh)))
            elif x1 < 0 or y1 < 0 or (iw and x2 > iw) or (ih and y2 > ih):
                bad_box.append((stem, 'out of bounds (%g,%g)-(%g,%g) img %dx%d'
                                % (x1, y1, x2, y2, iw, ih)))
            else:
                sizes.append((bw, bh))
            n += 1
        per_img.append((stem, n))

    print()
    print('--- 3. class names as written in XML ---')
    for k, v in raw_names.most_common():
        flag = '' if k in declared else '   <-- NOT in classes.txt'
        print('  %-20s %5d%s' % (repr(k), v, flag))
    tot = sum(raw_names.values())
    if tot:
        print('  total boxes %d' % tot)
        if len(raw_names) > len(declared):
            print('  !! more distinct names than declared classes -> typos?')

    print()
    print('--- 4. image dimensions ---')
    for k, v in img_dims.most_common():
        print('  %sx%s : %d' % (k[0], k[1], v))
    if len(img_dims) > 1:
        print('  !! mixed resolutions')

    print()
    print('--- 5. box size distribution (in image px) ---')
    if sizes:
        sizes.sort(key=lambda t: t[0] * t[1])
        iw = img_dims.most_common(1)[0][0][0] or 800
        scale = NET / float(iw)
        print('  image width %d -> net %d, scale %.3f' % (iw, NET, scale))
        print('  %-6s %-14s %-14s %s' % ('pct', 'box w x h', 'in net', 'note'))
        for p in (0, 5, 25, 50, 75, 95, 100):
            i = min(int(p / 100.0 * (len(sizes) - 1)), len(sizes) - 1)
            bw, bh = sizes[i]
            nw, nh = bw * scale, bh * scale
            note = ''
            if min(nw, nh) < 16:
                note = 'TOO SMALL (<1 P4 cell)'
            elif min(nw, nh) < 32:
                note = 'small, may be missed'
            print('  P%-5d %6.0f x %-6.0f %6.1f x %-6.1f %s'
                  % (p, bw, bh, nw, nh, note))
        tiny = sum(1 for bw, bh in sizes if min(bw, bh) * scale < 16)
        small = sum(1 for bw, bh in sizes if min(bw, bh) * scale < 32)
        print('  boxes <16px in net: %d (%.1f%%)  <32px: %d (%.1f%%)'
              % (tiny, 100.0 * tiny / len(sizes), small, 100.0 * small / len(sizes)))

    print()
    print('--- 6. objects per image ---')
    hist = Counter(n for _, n in per_img)
    for k in sorted(hist):
        print('  %d objects: %d images' % (k, hist[k]))
    empty = [s for s, n in per_img if n == 0]
    if empty:
        print('  !! %d annotations with ZERO objects: %s' % (len(empty), empty[:10]))

    print()
    print('--- 7. scene group coverage (from filename _sN) ---')
    grp = Counter()
    for p in bmps:
        b = os.path.basename(p)
        if '_s' in b:
            grp['s' + b.split('_s')[1][0]] += 1
        else:
            grp['(none)'] += 1
    for k in sorted(grp):
        print('  %-8s %d' % (k, grp[k]))

    print()
    print('--- 8. problems ---')
    if bad_box:
        print('  %d bad boxes:' % len(bad_box))
        for s, m in bad_box[:15]:
            print('      %s: %s' % (s, m))
        if len(bad_box) > 15:
            print('      ... +%d more' % (len(bad_box) - 15))
    else:
        print('  no degenerate / out-of-bounds boxes')

    print()
    print('=' * 62)
    print('VERDICT')
    ok = True
    if no_ann:
        print('  - %d unannotated images: annotate or move them out' % len(no_ann))
        ok = False
    if len(raw_names) > 2:
        print('  - class name typos present, must unify')
        ok = False
    if declared and declared[0] != 'weed':
        print('  - classes.txt order wrong')
        ok = False
    if bad_box:
        print('  - %d invalid boxes need fixing' % len(bad_box))
        ok = False
    if sizes:
        r = cls_cnt.get('weed', 0) / float(max(cls_cnt.get('crop', 1), 1))
        print('  - class balance weed:crop = %.2f  %s'
              % (r, '(ok)' if 0.4 < r < 2.5 else '(IMBALANCED)'))
    if ok:
        print('  dataset structurally clean, ready to convert')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'E:/dataset')
