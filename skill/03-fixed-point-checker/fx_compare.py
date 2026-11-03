#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fx_compare.py — 定点实现输出 vs 浮点参考输出的误差统计

用途：把一个定点加速器（或任何定点的 DSP 链）的输出，与浮点参考逐元素比对，
回答三个问题：
    1. 差多少（最大 / RMS / 分位数，按真实物理量而非原始码值）
    2. 差在哪（最差的若干个位置，配合 shape 能定位到通道与行列）
    3. 有没有饱和（码值贴住 dtype 上下限的样本数）

用法：
    python fx_compare.py ref.bin dut.bin --dut-dtype int16 --scale 512
    python fx_compare.py ref.bin dut.bin --dut-dtype int16 --scale 512 \
           --shape 32,13,13 --per-channel --tol 0.05
    python fx_compare.py --selftest

参数：
    ref / dut        两个裸二进制文件，元素顺序、元素个数必须一致
    --ref-dtype      参考文件类型，默认 float32
    --dut-dtype      被测文件类型，默认 int16
    --scale          定点缩放因子：真实值 = 码值 / scale；默认 1（表示文件里已是浮点）
    --shape          形如 32,13,13，用于计算最差位置的下标；不给则按一维处理
    --per-channel    shape 为 (C, ...) 时，额外给出每个通道的误差
    --tol            最大绝对误差阈值（真实单位）；超过则退出码为 1
    --top            打印最差的前 N 个位置，默认 5

退出码：0 通过；1 超过 --tol；2 输入错误
"""

import argparse
import sys

import numpy as np

DTYPES = {
    "float32": np.float32, "float64": np.float64,
    "int8": np.int8, "uint8": np.uint8,
    "int16": np.int16, "uint16": np.uint16,
    "int32": np.int32, "uint32": np.uint32,
}


def load(path, dtype, scale):
    raw = np.fromfile(path, dtype=DTYPES[dtype]).astype(np.float64)
    if raw.size == 0:
        raise ValueError("文件为空：%s" % path)
    return raw / float(scale)


def saturate_count(path, dtype):
    """统计贴住 dtype 上下限的样本数（定点化的饱和标志）。"""
    arr = np.fromfile(path, dtype=DTYPES[dtype])
    if not np.issubdtype(arr.dtype, np.integer):
        return 0, 0
    info = np.iinfo(arr.dtype)
    low = int(np.count_nonzero(arr == info.min))
    high = int(np.count_nonzero(arr == info.max))
    return low, high


def describe(tag, err, ref):
    ref_rms = float(np.sqrt(np.mean(ref ** 2))) or 1e-12
    return {
        "tag": tag,
        "n": int(err.size),
        "max": float(np.max(np.abs(err))),
        "rms": float(np.sqrt(np.mean(err ** 2))),
        "rel_rms": float(np.sqrt(np.mean(err ** 2)) / ref_rms),
        "p99": float(np.percentile(np.abs(err), 99)),
        "mean": float(np.mean(np.abs(err))),
    }


def print_row(s):
    print("  %-14s n=%-8d max=%-10.5f rms=%-10.5f rel_rms=%-8.3f%% p99=%-10.5f"
          % (s["tag"], s["n"], s["max"], s["rms"], s["rel_rms"] * 100, s["p99"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description="定点输出与浮点参考的误差比对")
    ap.add_argument("ref", nargs="?")
    ap.add_argument("dut", nargs="?")
    ap.add_argument("--ref-dtype", default="float32", choices=DTYPES)
    ap.add_argument("--dut-dtype", default="int16", choices=DTYPES)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--shape", default="")
    ap.add_argument("--per-channel", action="store_true")
    ap.add_argument("--tol", type=float, default=None)
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)

    if a.selftest:
        return selftest()

    if not a.ref or not a.dut:
        ap.error("需要 ref 与 dut 两个文件（或用 --selftest）")

    try:
        ref = load(a.ref, a.ref_dtype, 1.0)
        dut = load(a.dut, a.dut_dtype, a.scale)
    except (ValueError, OSError) as e:
        print("输入错误：%s" % e, file=sys.stderr)
        return 2

    if ref.size != dut.size:
        print("元素个数不一致：ref=%d dut=%d" % (ref.size, dut.size), file=sys.stderr)
        return 2

    shape = None
    if a.shape:
        try:
            shape = tuple(int(x) for x in a.shape.split(","))
        except ValueError:
            print("--shape 格式应为 32,13,13", file=sys.stderr)
            return 2
        if int(np.prod(shape)) != ref.size:
            print("shape 乘积 %d 与文件元素数 %d 不一致" % (int(np.prod(shape)), ref.size),
                  file=sys.stderr)
            return 2
        ref = ref.reshape(shape)
        dut = dut.reshape(shape)

    err = dut - ref
    print("=" * 68)
    print("文件：ref=%s  dut=%s" % (a.ref, a.dut))
    print("约定：真实值 = 码值 / %g" % a.scale)
    print("=" * 68)
    print_row(describe("总体", err.ravel(), ref.ravel()))

    low, high = saturate_count(a.dut, a.dut_dtype)
    if low or high:
        print("  饱和：贴下限 %d 个，贴上限 %d 个（共 %d）" % (low, high, low + high))
    else:
        print("  饱和：无")

    if shape and len(shape) >= 2 and a.per_channel:
        print("-" * 68)
        print("按通道：")
        for c in range(shape[0]):
            print_row(describe("ch%02d" % c, err[c].ravel(), ref[c].ravel()))

    flat = np.abs(err).ravel()
    k = min(a.top, flat.size)
    idx = np.argsort(flat)[-k:][::-1]
    print("-" * 68)
    print("最差 %d 个位置：" % k)
    for i in idx:
        pos = np.unravel_index(int(i), shape) if shape else (int(i),)
        print("  %-22s ref=%9.5f  dut=%9.5f  err=%9.5f"
              % (str(pos), ref.ravel()[i], dut.ravel()[i], flat[i]))

    ok = True
    if a.tol is not None:
        worst = float(np.max(flat))
        ok = worst <= a.tol
        print("-" * 68)
        print("判定：最大误差 %.5f %s 阈值 %.5f → %s"
              % (worst, "<=" if ok else ">", a.tol, "通过" if ok else "不通过"))
    return 0 if ok else 1


def selftest():
    """用已知量化误差自测：int16/512 的理论最大误差应为半个 LSB。"""
    rng = np.random.default_rng(0)
    ref = (rng.standard_normal((8, 5, 5)) * 3.0).astype(np.float32)
    scale = 512.0
    q = np.clip(np.round(ref.astype(np.float64) * scale), -32768, 32767).astype(np.int16)

    ref.tofile("_st_ref.bin")
    q.tofile("_st_dut.bin")

    rc = main(["_st_ref.bin", "_st_dut.bin", "--scale", str(scale),
               "--shape", "8,5,5", "--tol", "0.002", "--top", "3"])
    import os
    os.remove("_st_ref.bin")
    os.remove("_st_dut.bin")

    half_lsb = 0.5 / scale
    print("\n[selftest] 半个 LSB = %.6f，若上面的 max 不超过它，脚本正确。" % half_lsb)
    return rc


if __name__ == "__main__":
    sys.exit(main())
