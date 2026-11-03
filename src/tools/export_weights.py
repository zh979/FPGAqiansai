"""
export_weights.py -- 训练产出 -> 板上可用的 folded_weights

做三件事:
  1. BN 折叠。把 conv+BN 合成一个等效的 conv (推理时数学上完全等价),
     这样板上就不需要实现 BN 层 -- 加速器只有卷积。
     折叠公式见 bn_merge.py, 已在原作者的 80 类模型上验证过。
  2. 校验形状。检测头的通道数必须是 (类别数+5)*3, 和板上
     basic_op.h 的 OUT_CH 对应。不符就直接报错, 别等上板才发现。
  3. 列出文件清单和总大小, 便于拷 SD 卡时核对。

用法:
    python export_weights.py                      # 用 logs/best_epoch_weights.pth
    python export_weights.py logs/last_epoch_weights.pth out_dir
"""
import os
import sys
import shutil
import numpy as np
import torch

# 本项目: 2 类 -> 每个 anchor 7 个属性 -> 检测头 21 通道
NUM_CLASSES = 2
EXPECT_OUT_CH = (NUM_CLASSES + 5) * 3

EPS = 1e-5


def fuse(state, pre):
    """conv + BN -> 等效 conv。返回 (weight, bias)"""
    w = state[pre + 'conv.weight'].cpu().numpy()
    g = state[pre + 'bn.weight'].cpu().numpy()
    b = state[pre + 'bn.bias'].cpu().numpy()
    m = state[pre + 'bn.running_mean'].cpu().numpy()
    v = state[pre + 'bn.running_var'].cpu().numpy()
    std = np.sqrt(v + EPS)
    fw = w * (g / std).reshape(-1, 1, 1, 1)
    fb = b - m * g / std
    return fw.astype(np.float32), fb.astype(np.float32)


def save(arr, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    arr.astype(np.float32).tofile(path)
    return arr.size * 4


def main(ckpt, out):
    if not os.path.exists(ckpt):
        print('not found: %s' % ckpt)
        print('train first, or pass the checkpoint path as argv[1]')
        return 1

    state = torch.load(ckpt, map_location='cpu')
    if 'model' in state and isinstance(state['model'], dict):
        state = state['model']
    print('checkpoint %s  (%d tensors)' % (ckpt, len(state)))

    # ---- 形状校验 ----
    bad = []
    for k in ('yolo_headP5.1.weight', 'yolo_headP4.1.weight'):
        if k not in state:
            bad.append('%s missing' % k)
            continue
        ch = state[k].shape[0]
        tag = 'OK' if ch == EXPECT_OUT_CH else 'MISMATCH'
        print('  %-26s out_ch=%-4d expect %-4d %s' % (k, ch, EXPECT_OUT_CH, tag))
        if ch != EXPECT_OUT_CH:
            bad.append('%s has %d channels, expect %d' % (k, ch, EXPECT_OUT_CH))
    if bad:
        print()
        print('ABORT:')
        for b in bad:
            print('  %s' % b)
        print('  -> checkpoint does not match NUM_CLASSES=%d' % NUM_CLASSES)
        return 2

    if os.path.exists(out):
        shutil.rmtree(out)

    total = 0

    def put(name, w, b):
        nonlocal total
        total += save(w, os.path.join(out, name, 'w.bin'))
        total += save(b, os.path.join(out, name, 'b.bin'))

    # ---- backbone ----
    for i in (1, 2, 3):
        w, b = fuse(state, 'backbone.conv%d.' % i)
        put('BasicConv%d' % i, w, b)

    # ---- resblocks: 每个有 4 个 conv+BN ----
    for i in (1, 2, 3):
        pre = 'backbone.resblock_body%d.' % i
        d = os.path.join(out, 'ResBlock%d' % i)
        os.makedirs(d, exist_ok=True)
        for j in (1, 2, 3, 4):
            w, b = fuse(state, pre + 'conv%d.' % j)
            total += save(w, os.path.join(d, 'w%d.bin' % j))
            total += save(b, os.path.join(d, 'b%d.bin' % j))

    # ---- conv_for_P5 ----
    w, b = fuse(state, 'conv_for_P5.')
    put('conv_forP5', w, b)

    # ---- upsample ----
    w, b = fuse(state, 'upsample.upsample.0.')
    put('upsample', w, b)

    # ---- 检测头: .0 是 conv+BN, .1 是纯 conv(无 BN, 不折叠) ----
    for head in ('yolo_headP4', 'yolo_headP5'):
        d = os.path.join(out, head)
        os.makedirs(d, exist_ok=True)
        w1, b1 = fuse(state, head + '.0.')
        total += save(w1, os.path.join(d, 'w1.bin'))
        total += save(b1, os.path.join(d, 'b1.bin'))
        w2 = state[head + '.1.weight'].cpu().numpy()
        b2 = state[head + '.1.bias'].cpu().numpy()
        total += save(w2, os.path.join(d, 'w2.bin'))
        total += save(b2, os.path.join(d, 'b2.bin'))

    print()
    print('=' * 58)
    print('wrote %s   total %.2f MB (float32)' % (out, total / 1048576.0))
    print()
    print('files:')
    for root, dirs, files in sorted(os.walk(out)):
        if not files:
            continue
        rel = os.path.relpath(root, out)
        sz = sum(os.path.getsize(os.path.join(root, f)) for f in files)
        print('  %-16s %d files  %8.1f KB' % (rel, len(files), sz / 1024.0))

    print()
    print('next: copy this folder to the SD card root as "folded_weights"')
    print('      board reads them as float32 and converts to Q7.9 (x512)')
    print()
    print('      also set in the Vitis project:')
    print('        basic_op.h   OUT_CH = %d' % EXPECT_OUT_CH)
    print('        yolo_post.h  YP_MODEL_WEED = 1')
    return 0


if __name__ == '__main__':
    ckpt = sys.argv[1] if len(sys.argv) > 1 else 'logs/best_epoch_weights.pth'
    out = sys.argv[2] if len(sys.argv) > 2 else 'folded_weights'
    sys.exit(main(ckpt, out))
