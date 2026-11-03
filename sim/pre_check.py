"""
pre_check.py -- 预处理验证工具, 两种用法

用法 1: 对比我的缩放算法 vs Python 的 BICUBIC (纯 PC, 不用板子)
    python pre_check.py compare <img.png> <参考img.bin>
  复现 yolo_pre.cpp 的算法, 和 Python 参考的网络输入逐元素比较,
  量化出缩放方式差异带来的影响。

用法 2: 把板子写回的预处理结果还原成 PNG (验证字节序和几何)
    python pre_check.py view <板子的pre.bin> <输出.png> [q|f]
  q = Q7.9 的 short (板上内存直接 dump), f = float32 (经 write_data 转换)
  拍纯红物体, 若还原图是红的则 YPRE_SRC_BGR=0 正确, 是蓝的则要改成 1。
"""
import sys
import numpy as np

NS = 416
NPX = NS * NS


def build_lut():
    """与 yolo_pre.cpp 的 g_q 完全一致"""
    return np.array([(v * 512 + 127) // 255 for v in range(256)], dtype=np.int16)


def build_tabs(roi_x, roi_y, roi_w, roi_h):
    """与 yolo_pre.cpp 的 yp_build_tabs 完全一致 (取格子中心)"""
    i = np.arange(NS, dtype=np.int64)
    xt = roi_x + ((i * 2 + 1) * roi_w) // (2 * NS)
    yt = roi_y + ((i * 2 + 1) * roi_h) // (2 * NS)
    return xt.astype(np.int64), yt.astype(np.int64)


FBITS = 4
FONE = 1 << FBITS


def build_bilin_tabs(roi_x, roi_y, roi_w, roi_h):
    """与 yolo_pre.cpp 的双线性定点表完全一致"""
    i = np.arange(NS, dtype=np.int64)
    sx = ((2 * i + 1) * roi_w * FONE) // (2 * NS) - (FONE // 2)
    sy = ((2 * i + 1) * roi_h * FONE) // (2 * NS) - (FONE // 2)
    sx = np.maximum(sx, 0)
    sy = np.maximum(sy, 0)
    return (roi_x + (sx >> FBITS), sx & (FONE - 1),
            roi_y + (sy >> FBITS), sy & (FONE - 1))


def my_preprocess(rgb, mode='box2x2', roi=None):
    """复现 yolo_pre.cpp。rgb: (H,W,3) uint8。返回 (3,416,416) int16 Q7.9"""
    H, W, _ = rgb.shape
    if roi is None:
        roi = (0, 0, W, H)
    rx, ry, rw, rh = roi

    # 复现 yolo_preprocess 入口处的 ROI 夹取, 否则贴边 ROI 会索引越界
    if rw <= 0:
        rw = W
    if rh <= 0:
        rh = H
    rx = min(max(rx, 0), W - 1)
    ry = min(max(ry, 0), H - 1)
    if rx + rw > W:
        rw = W - rx
    if ry + rh > H:
        rh = H - ry

    lut = build_lut()
    xt, yt = build_tabs(rx, ry, rw, rh)

    if mode == 'bilinear':
        xi, xf, yi, yf = build_bilin_tabs(rx, ry, rw, rh)
        x1 = np.minimum(xi + 1, W - 1)
        y1 = np.minimum(yi + 1, H - 1)
        wx = xf[None, :, None]
        iwx = FONE - wx
        wy = yf[:, None, None]
        iwy = FONE - wy
        a = rgb[np.ix_(yi, xi)].astype(np.int32)
        b = rgb[np.ix_(yi, x1)].astype(np.int32)
        e = rgb[np.ix_(y1, xi)].astype(np.int32)
        f = rgb[np.ix_(y1, x1)].astype(np.int32)
        t = (a * iwx + b * wx) * iwy + (e * iwx + f * wx) * wy
        samp = ((t + 128) >> 8).astype(np.uint8)
        q = lut[samp]
        return np.transpose(q, (2, 0, 1)).astype(np.int16)

    if mode == 'nearest':
        samp = rgb[np.ix_(yt, xt)]                     # (416,416,3)
    else:
        sx = max(rw // NS, 1)
        sy = max(rh // NS, 1)
        yt2 = np.minimum(yt + sy, H - 1)
        xt2 = np.minimum(xt + sx, W - 1)
        a = rgb[np.ix_(yt, xt)].astype(np.int32)
        b = rgb[np.ix_(yt, xt2)].astype(np.int32)
        e = rgb[np.ix_(yt2, xt)].astype(np.int32)
        f = rgb[np.ix_(yt2, xt2)].astype(np.int32)
        samp = ((a + b + e + f + 2) >> 2).astype(np.uint8)

    q = lut[samp]                                      # (416,416,3)
    return np.transpose(q, (2, 0, 1)).astype(np.int16)  # CHW


def cmd_compare(png_path, ref_bin):
    from PIL import Image
    img = Image.open(png_path).convert('RGB')
    rgb = np.array(img, dtype=np.uint8)
    print('输入图 %s  %dx%d' % (png_path, img.size[0], img.size[1]))

    ref = np.fromfile(ref_bin, dtype=np.float32)
    if ref.size != 3 * NPX:
        print('参考 img.bin 元素数 %d, 预期 %d -- 文件不对?' % (ref.size, 3 * NPX))
        return
    # 板端 read_params 的行为: round(float * 512)
    ref_q = np.round(ref * 512.0).astype(np.int32).reshape(3, NS, NS)
    print('参考网络输入 (Python BICUBIC + round*512): 值域 [%d, %d]'
          % (ref_q.min(), ref_q.max()))
    print()

    for mode in ('nearest', 'box2x2', 'bilinear'):
        mine = my_preprocess(rgb, mode).astype(np.int32)
        d = np.abs(mine - ref_q)
        # 换算回 0..255 灰度尺度便于理解: Q7.9 的 512 对应 255
        scale = 255.0 / 512.0
        print('%-8s  平均绝对误差 %6.2f (%.2f/255)   最大 %4d (%.0f/255)   '
              '误差>10%%的像素占比 %.2f%%'
              % (mode, d.mean(), d.mean() * scale, d.max(), d.max() * scale,
                 100.0 * (d > 51).mean()))

    # 字节序反了会是什么样 -- 给个参照
    mine = my_preprocess(rgb, 'bilinear').astype(np.int32)
    swapped = mine[::-1]          # R/B 互换
    d_ok = np.abs(mine - ref_q).mean()
    d_sw = np.abs(swapped - ref_q).mean()
    print()
    print('字节序参照: 正确顺序平均误差 %.2f, R/B 互换后 %.2f (相差 %.1f 倍)'
          % (d_ok, d_sw, d_sw / max(d_ok, 1e-6)))


def cmd_view(bin_path, out_png, dtype='q'):
    from PIL import Image
    if dtype == 'f':
        a = np.fromfile(bin_path, dtype=np.float32)
        if a.size != 3 * NPX:
            print('元素数 %d, 预期 %d' % (a.size, 3 * NPX)); return
        v = a.reshape(3, NS, NS) * 512.0
    else:
        a = np.fromfile(bin_path, dtype=np.int16)
        if a.size != 3 * NPX:
            print('元素数 %d, 预期 %d (若是 float 请传 f)' % (a.size, 3 * NPX)); return
        v = a.reshape(3, NS, NS).astype(np.float32)

    # Q7.9: 512 对应 1.0, 再映射到 0..255
    rgb = np.clip(v * (255.0 / 512.0), 0, 255).astype(np.uint8)
    rgb = np.transpose(rgb, (1, 2, 0))
    Image.fromarray(rgb, 'RGB').save(out_png)
    print('已保存 %s' % out_png)
    print('各通道均值:  R=%.1f  G=%.1f  B=%.1f' %
          (rgb[:, :, 0].mean(), rgb[:, :, 1].mean(), rgb[:, :, 2].mean()))
    print('打开图片检查: 颜色对不对、有没有左右/上下颠倒、有没有斜向错切')
    print('  斜向错切通常意味着 src_stride 传错了')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    if sys.argv[1] == 'compare' and len(sys.argv) >= 4:
        cmd_compare(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == 'view' and len(sys.argv) >= 4:
        cmd_view(sys.argv[2], sys.argv[3],
                 sys.argv[4] if len(sys.argv) > 4 else 'q')
    else:
        print(__doc__)
