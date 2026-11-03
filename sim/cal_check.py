"""
cal_check.py -- 验证 calib.cpp 的最小二乘拟合

复现 calib.cpp 的算法(法方程 + 高斯消元), 与 numpy 的 lstsq 对比。
同时测退化情况(共线点)是否能被正确检出。
"""
import numpy as np


def port_fit(pts):
    """复现 calib.cpp 的 cal_fit。pts: [(px,py,gx,gy), ...]
    返回 (a, b, rms_px) 或 None 表示奇异"""
    n = len(pts)
    if n < 3:
        return None

    Spx = Spy = Spx2 = Spy2 = Spxpy = 0.0
    Sgx = Sgy = Spxgx = Spygx = Spxgy = Spygy = 0.0
    for px, py, gx, gy in pts:
        Spx += px; Spy += py
        Spx2 += px * px; Spy2 += py * py
        Spxpy += px * py
        Sgx += gx; Sgy += gy
        Spxgx += px * gx; Spygx += py * gx
        Spxgy += px * gy; Spygy += py * gy

    A = [[Spx2, Spxpy, Spx],
         [Spxpy, Spy2, Spy],
         [Spx, Spy, float(n)]]
    r1 = [Spxgx, Spygx, Sgx]
    r2 = [Spxgy, Spygy, Sgy]

    # 高斯消元, 带部分主元 (复现 solve3x3_2rhs)
    A = [row[:] for row in A]
    r1 = r1[:]
    r2 = r2[:]
    for col in range(3):
        piv, best = col, abs(A[col][col])
        for r in range(col + 1, 3):
            if abs(A[r][col]) > best:
                best, piv = abs(A[r][col]), r
        if best < 1e-9:
            return None
        if piv != col:
            A[col], A[piv] = A[piv], A[col]
            r1[col], r1[piv] = r1[piv], r1[col]
            r2[col], r2[piv] = r2[piv], r2[col]
        for r in range(col + 1, 3):
            f = A[r][col] / A[col][col]
            if f == 0.0:
                continue
            for k in range(col, 3):
                A[r][k] -= f * A[col][k]
            r1[r] -= f * r1[col]
            r2[r] -= f * r2[col]

    x1 = [0.0] * 3
    x2 = [0.0] * 3
    for i in (2, 1, 0):
        s1, s2 = r1[i], r2[i]
        for k in range(i + 1, 3):
            s1 -= A[i][k] * x1[k]
            s2 -= A[i][k] * x2[k]
        x1[i] = s1 / A[i][i]
        x2[i] = s2 / A[i][i]

    sx = (x1[0] ** 2 + x1[1] ** 2) ** 0.5
    sy = (x2[0] ** 2 + x2[1] ** 2) ** 0.5
    scale = (sx + sy) * 0.5 or 1.0
    sse = 0.0
    for px, py, gx, gy in pts:
        ex = (x1[0] * px + x1[1] * py + x1[2]) - gx
        ey = (x2[0] * px + x2[1] * py + x2[2]) - gy
        sse += ex * ex + ey * ey
    rms = (sse / len(pts)) ** 0.5 / scale
    return x1, x2, rms


def ref_fit(pts):
    """numpy 参考解"""
    M = np.array([[p[0], p[1], 1.0] for p in pts])
    gx = np.array([p[2] for p in pts])
    gy = np.array([p[3] for p in pts])
    a, *_ = np.linalg.lstsq(M, gx, rcond=None)
    b, *_ = np.linalg.lstsq(M, gy, rcond=None)
    return a, b


def main():
    # 真实映射: 云台脉冲 = 像素 * 增益 + 偏置, 带轻微交叉项(安装不正)
    TA = [3.2, 0.15, -520.0]
    TB = [-0.08, 2.9, 310.0]

    print("=== 1. 理想标定点(无噪声), 6 点 ===")
    pts = []
    for px, py in [(100, 80), (700, 90), (400, 240), (120, 400), (680, 410), (400, 100)]:
        gx = TA[0] * px + TA[1] * py + TA[2]
        gy = TB[0] * px + TB[1] * py + TB[2]
        pts.append((px, py, gx, gy))

    r = port_fit(pts)
    a_ref, b_ref = ref_fit(pts)
    print("  真实 a = %s" % [round(v, 6) for v in TA])
    print("  拟合 a = %s" % [round(v, 6) for v in r[0]])
    print("  numpy a= %s" % [round(v, 6) for v in a_ref])
    da = max(abs(r[0][i] - a_ref[i]) for i in range(3))
    db = max(abs(r[1][i] - b_ref[i]) for i in range(3))
    print("  与 numpy 最大偏差: a %.2e  b %.2e" % (da, db))
    print("  rms = %.4f px (应接近 0)" % r[2])

    print()
    print("=== 2. 带测量噪声 (±3 脉冲), 6 点 ===")
    np.random.seed(11)
    pts2 = [(p[0], p[1],
             p[2] + np.random.randn() * 3,
             p[3] + np.random.randn() * 3) for p in pts]
    r2 = port_fit(pts2)
    print("  拟合 a = %s" % [round(v, 4) for v in r2[0]])
    print("  rms = %.3f px" % r2[2])
    # 预测精度: 在一个未参与标定的点上检验
    tx, ty = 550.0, 300.0
    true_gx = TA[0] * tx + TA[1] * ty + TA[2]
    pred_gx = r2[0][0] * tx + r2[0][1] * ty + r2[0][2]
    print("  未见点 (%.0f,%.0f): 真实 gx=%.1f 预测 %.1f 误差 %.2f 脉冲"
          % (tx, ty, true_gx, pred_gx, abs(pred_gx - true_gx)))
    print("  换算: %.2f 脉冲 / %.2f px/脉冲 = %.3f px"
          % (abs(pred_gx - true_gx), TA[0], abs(pred_gx - true_gx) / TA[0]))

    print()
    print("=== 3. 退化: 3 点共线 (应返回奇异) ===")
    col = [(100, 100, 0, 0), (200, 200, 320, 290), (300, 300, 640, 580)]
    print("  结果: %s" % ("检出奇异 (正确)" if port_fit(col) is None else "未检出 (错误!)"))

    print()
    print("=== 4. 退化: 点数不足 (2 点) ===")
    print("  结果: %s" % ("拒绝 (正确)" if port_fit(pts[:2]) is None else "未拒绝 (错误!)"))

    print()
    print("=== 5. 最少点数 (3 点非共线) ===")
    r5 = port_fit([pts[0], pts[1], pts[3]])
    print("  结果: %s, rms=%.4f" % ("成功" if r5 else "失败", r5[2] if r5 else -1))

    print()
    ok = r and da < 1e-6 and db < 1e-6 and port_fit(col) is None
    print("结论: " + ("拟合算法正确, 退化检测有效" if ok else "存在问题, 需排查"))


if __name__ == "__main__":
    main()
