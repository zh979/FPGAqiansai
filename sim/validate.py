"""
validate.py -- 验证 yolo_post.cpp 的算法正确性

做两件事:
  A. "reference": 严格按 utils_bbox.py (PyTorch) 的语义用 numpy 朴素实现,
     全量 sigmoid, 不做任何优化。这是黄金参考。
  B. "ported":    逐行复现 yolo_post.cpp 的代码路径 -- logit 空间预筛、
     选择排序、逐类贪心 NMS。

两者跑同一份张量, 逐框对比。完全一致 => C++ 移植的数值行为正确。
"""
import numpy as np
import sys

NUM_CLASSES = 80
BBOX_ATTRS = 5 + NUM_CLASSES
OUT_CH = BBOX_ATTRS * 3
INPUT_SIZE = 416

ANCHORS = np.array([[10, 14], [23, 27], [37, 58],
                    [81, 82], [135, 169], [344, 319]], dtype=np.float32)
MASK = [[3, 4, 5], [1, 2, 3]]


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


# ---------------------------------------------------------------- A: reference
def ref_decode(raw, grid, scale_idx):
    """严格照抄 decode_box: view(1,3,85,H,W).permute(0,1,3,4,2)"""
    p = raw.reshape(3, BBOX_ATTRS, grid, grid)

    stride = INPUT_SIZE / grid
    scaled = ANCHORS[MASK[scale_idx]] / stride          # 网格单位

    gx = np.arange(grid, dtype=np.float32)[None, None, :]   # col
    gy = np.arange(grid, dtype=np.float32)[None, :, None]   # row

    x = sigmoid(p[:, 0]) + gx
    y = sigmoid(p[:, 1]) + gy
    w = np.exp(p[:, 2]) * scaled[:, 0][:, None, None]
    h = np.exp(p[:, 3]) * scaled[:, 1][:, None, None]
    conf = sigmoid(p[:, 4])
    cls = sigmoid(p[:, 5:])                              # (3,80,H,W)

    # 归一化: 除以网格数
    out = np.stack([x / grid, y / grid, w / grid, h / grid, conf], axis=1)
    out = out.reshape(3, 5, grid * grid).transpose(0, 2, 1).reshape(-1, 5)
    cls = cls.reshape(3, NUM_CLASSES, grid * grid).transpose(0, 2, 1).reshape(-1, NUM_CLASSES)
    return np.concatenate([out, cls], axis=1)            # (N, 85)


def ref_nms_greedy(boxes, scores, thr):
    """torchvision.ops.nms 的语义: 按分数降序贪心抑制"""
    idx = np.argsort(-scores, kind='stable')
    keep = []
    while idx.size:
        i = idx[0]
        keep.append(i)
        if idx.size == 1:
            break
        rest = idx[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        iw = np.maximum(0.0, xx2 - xx1)
        ih = np.maximum(0.0, yy2 - yy1)
        inter = iw * ih
        a1 = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        a2 = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / (a1 + a2 - inter)
        idx = rest[iou <= thr]
    return np.array(keep, dtype=np.int64)


def reference(out0, out1, img_w, img_h, conf_thres, nms_thres):
    pred = np.concatenate([ref_decode(out0, 13, 0),
                           ref_decode(out1, 26, 1)], axis=0)

    # 中心宽高 -> 角点
    corner = pred.copy()
    corner[:, 0] = pred[:, 0] - pred[:, 2] / 2
    corner[:, 1] = pred[:, 1] - pred[:, 3] / 2
    corner[:, 2] = pred[:, 0] + pred[:, 2] / 2
    corner[:, 3] = pred[:, 1] + pred[:, 3] / 2

    class_conf = corner[:, 5:5 + NUM_CLASSES].max(axis=1)
    class_pred = corner[:, 5:5 + NUM_CLASSES].argmax(axis=1)
    mask = (corner[:, 4] * class_conf) >= conf_thres
    if not mask.any():
        return []

    d_box = corner[mask, :4]
    d_obj = corner[mask, 4]
    d_cls = class_conf[mask]
    d_id = class_pred[mask]
    score = d_obj * d_cls

    res = []
    for c in np.unique(d_id):
        sel = d_id == c
        keep = ref_nms_greedy(d_box[sel], score[sel], nms_thres)
        for k in keep:
            b = d_box[sel][k]
            res.append((float(score[sel][k]), int(c),
                        float(b[1] * img_h), float(b[0] * img_w),
                        float(b[3] * img_h), float(b[2] * img_w)))
    res.sort(key=lambda t: -t[0])
    return res


# ------------------------------------------------------------------- B: ported
def ported_decode_scale(raw, grid, scale_idx, t_gate, conf_thres, cand):
    """逐行复现 yp_decode_scale"""
    HW = grid * grid
    for a in range(3):
        base = a * BBOX_ATTRS * HW
        aw, ah = ANCHORS[MASK[scale_idx][a]]
        for r in range(grid):
            for c in range(grid):
                p = r * grid + c
                t_obj = raw[base + 4 * HW + p]
                if t_obj < t_gate:                       # 预筛 1: 不算 sigmoid
                    continue
                best_k, best_t = 0, raw[base + 5 * HW + p]
                for k in range(1, NUM_CLASSES):
                    t = raw[base + (5 + k) * HW + p]
                    if t > best_t:
                        best_t, best_k = t, k
                if best_t < t_gate:                      # 预筛 2
                    continue
                obj = 1.0 / (1.0 + np.exp(-t_obj))
                cls = 1.0 / (1.0 + np.exp(-best_t))
                sc = obj * cls
                if sc < conf_thres:
                    continue
                t_x = raw[base + 0 * HW + p]
                t_y = raw[base + 1 * HW + p]
                t_w = raw[base + 2 * HW + p]
                t_h = raw[base + 3 * HW + p]
                cx = (1.0 / (1.0 + np.exp(-t_x)) + c) / grid
                cy = (1.0 / (1.0 + np.exp(-t_y)) + r) / grid
                bw = np.exp(t_w) * aw / INPUT_SIZE
                bh = np.exp(t_h) * ah / INPUT_SIZE
                cand.append(dict(box=[cx - bw / 2, cy - bh / 2,
                                      cx + bw / 2, cy + bh / 2],
                                 obj=obj, cls=cls, score=sc, cid=best_k))
    return cand


def ported_iou(a, b):
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    iw = ix2 - ix1; ih = iy2 - iy1
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    aa = (a[2] - a[0]) * (a[3] - a[1])
    bb = (b[2] - b[0]) * (b[3] - b[1])
    uni = aa + bb - inter
    return inter / uni if uni > 0 else 0.0


def ported(out0, out1, img_w, img_h, conf_thres, nms_thres):
    if conf_thres <= 0:
        t_gate = -1e30
    elif conf_thres >= 1:
        t_gate = 1e30
    else:
        t_gate = np.log(conf_thres / (1.0 - conf_thres))

    cand = []
    ported_decode_scale(out0, 13, 0, t_gate, conf_thres, cand)
    ported_decode_scale(out1, 26, 1, t_gate, conf_thres, cand)
    n = len(cand)
    if n == 0:
        return []

    order = list(range(n))
    for i in range(n - 1):                               # 选择排序, 同 C++
        best = i
        for j in range(i + 1, n):
            if cand[order[j]]['score'] > cand[order[best]]['score']:
                best = j
        if best != i:
            order[i], order[best] = order[best], order[i]

    removed = [False] * n
    res = []
    for i in range(n):
        ii = order[i]
        if removed[ii]:
            continue
        k = cand[ii]
        res.append((k['score'], k['cid'],
                    k['box'][1] * img_h, k['box'][0] * img_w,
                    k['box'][3] * img_h, k['box'][2] * img_w))
        for j in range(i + 1, n):
            jj = order[j]
            if removed[jj] or cand[jj]['cid'] != k['cid']:
                continue
            if ported_iou(k['box'], cand[jj]['box']) > nms_thres:
                removed[jj] = True
    return res


# ----------------------------------------------------------------------- main
def png_size(path):
    """从 PNG 文件头直接读宽高, 不依赖 PIL"""
    import struct
    with open(path, 'rb') as f:
        d = f.read(33)
    if d[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('不是 PNG 文件: ' + path)
    w, h = struct.unpack('>II', d[16:24])
    return int(w), int(h)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('用法:')
        print('  python validate.py <out0.bin> <out1.bin> <img.png> [conf] [nms]')
        print('  python validate.py <out0.bin> <out1.bin> <W> <H>    [conf] [nms]')
        sys.exit(1)

    p0, p1 = sys.argv[1], sys.argv[2]
    a3 = sys.argv[3] if len(sys.argv) > 3 else '640'
    if a3.lower().endswith('.png'):
        img_w, img_h = png_size(a3)
        rest = sys.argv[4:]
        print('图像尺寸取自 %s: %dx%d' % (a3, img_w, img_h))
    else:
        img_w = int(a3)
        img_h = int(sys.argv[4]) if len(sys.argv) > 4 else 480
        rest = sys.argv[5:]
    CONF = float(rest[0]) if len(rest) > 0 else 0.5
    NMS = float(rest[1]) if len(rest) > 1 else 0.3

    out0 = np.fromfile(p0, dtype=np.float32)
    out1 = np.fromfile(p1, dtype=np.float32)
    assert out0.size == OUT_CH * 13 * 13, '%s: %d floats, 预期 %d' % (p0, out0.size, OUT_CH * 13 * 13)
    assert out1.size == OUT_CH * 26 * 26, '%s: %d floats, 预期 %d' % (p1, out1.size, OUT_CH * 26 * 26)

    A = reference(out0, out1, img_w, img_h, CONF, NMS)
    B = ported(out0, out1, img_w, img_h, CONF, NMS)

    print('reference (numpy 朴素全量):  %d 个框' % len(A))
    print('ported    (C++ 代码路径):    %d 个框' % len(B))
    print()
    print('%-22s | %-22s' % ('reference', 'ported'))
    print('-' * 48)
    ok = len(A) == len(B)
    for i in range(max(len(A), len(B))):
        sa = ('%.4f c%-2d %.1f,%.1f,%.1f,%.1f' % A[i][:2] + '') if False else ''
        la = ('%.3f c%-2d %4.0f %4.0f %4.0f %4.0f' % (A[i][0], A[i][1], A[i][2], A[i][3], A[i][4], A[i][5])) if i < len(A) else '-'
        lb = ('%.3f c%-2d %4.0f %4.0f %4.0f %4.0f' % (B[i][0], B[i][1], B[i][2], B[i][3], B[i][4], B[i][5])) if i < len(B) else '-'
        flag = ''
        if i < len(A) and i < len(B):
            d = max(abs(A[i][k] - B[i][k]) for k in range(6))
            if d > 1e-3 or A[i][1] != B[i][1]:
                flag = '  <== 不一致 (max diff %.2e)' % d
                ok = False
        else:
            ok = False
            flag = '  <== 数量不匹配'
        print('%-22s | %-22s%s' % (la, lb, flag))
    print()
    print('结论: ' + ('两种实现完全一致，移植正确' if ok else '存在差异，需排查'))

    # 预筛效果统计
    total = 3 * 13 * 13 + 3 * 26 * 26
    t_gate = np.log(CONF / (1 - CONF))
    surv = 0
    for raw, grid, si in ((out0, 13, 0), (out1, 26, 1)):
        HW = grid * grid
        for a in range(3):
            base = a * BBOX_ATTRS * HW
            surv += int((raw[base + 4 * HW: base + 5 * HW] >= t_gate).sum())
    print()
    print('预筛效果: 全部锚框 %d -> objectness 过关 %d (%.2f%%)' %
          (total, surv, 100.0 * surv / total))
    print('  朴素实现超越函数调用: %d 次' % (total * BBOX_ATTRS))
    print('  预筛后 (估):          %d 次  -> 降低约 %.0f 倍' %
          (surv * BBOX_ATTRS, (total * BBOX_ATTRS) / max(1, surv * BBOX_ATTRS)))
