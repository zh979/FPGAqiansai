"""
kf_check.py -- 验证 track.cpp 里手工展开的 Kalman 与标准矩阵实现一致

track.cpp 把 4x4 矩阵运算按 x/y 两个独立子系统手工展开了(为了速度)。
展开容易写错, 所以这里用 numpy 做标准矩阵版参考, 逐步对比状态和协方差。
"""
import numpy as np

Q_POS, Q_VEL, R_MEAS = 1.0, 4.0, 25.0


# ---------------------------------------------------------- 参考实现
class RefKF:
    """标准矩阵形式, 对应 case2/tracking/kalman_filter.py 的模型"""

    def __init__(self):
        self.x = np.zeros((4, 1))
        self.P = np.zeros((4, 4))
        self.H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    def start(self, px, py):
        self.x = np.array([[px], [py], [0.0], [0.0]])
        self.P = np.diag([16.0, 16.0, 400.0, 400.0])

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        # track.cpp 里 Q 是 diag(q_pos*dt, q_pos*dt, q_vel*dt, q_vel*dt)
        Q = np.diag([Q_POS * dt, Q_POS * dt, Q_VEL * dt, Q_VEL * dt])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, px, py):
        z = np.array([[px], [py]])
        R = np.eye(2) * R_MEAS
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P


# ------------------------------------------------ 复现 track.cpp 的展开
class PortKF:
    def __init__(self):
        self.x = [0.0] * 4
        self.P = [[0.0] * 4 for _ in range(4)]

    def start(self, px, py):
        self.x = [px, py, 0.0, 0.0]
        self.P = [[0.0] * 4 for _ in range(4)]
        self.P[0][0] = 16.0
        self.P[1][1] = 16.0
        self.P[2][2] = 400.0
        self.P[3][3] = 400.0

    def predict(self, dt):
        self.x[0] += self.x[2] * dt
        self.x[1] += self.x[3] * dt
        q_p = Q_POS * dt
        q_v = Q_VEL * dt
        for k in range(2):
            ip, iv = k, k + 2
            ppp = self.P[ip][ip]
            ppv = self.P[ip][iv]
            pvp = self.P[iv][ip]
            pvv = self.P[iv][iv]
            self.P[ip][ip] = ppp + dt * (ppv + pvp) + dt * dt * pvv + q_p
            self.P[ip][iv] = ppv + dt * pvv
            self.P[iv][ip] = pvp + dt * pvv
            self.P[iv][iv] = pvv + q_v

    def update(self, px, py):
        z = [px, py]
        for k in range(2):
            ip, iv = k, k + 2
            S = self.P[ip][ip] + R_MEAS
            if S <= 0:
                continue
            inv = 1.0 / S
            Kp = self.P[ip][ip] * inv
            Kv = self.P[iv][ip] * inv
            innov = z[k] - self.x[ip]
            self.x[ip] += Kp * innov
            self.x[iv] += Kv * innov
            ppp = self.P[ip][ip]
            ppv = self.P[ip][iv]
            pvp = self.P[iv][ip]
            pvv = self.P[iv][iv]
            self.P[ip][ip] = ppp - Kp * ppp
            self.P[ip][iv] = ppv - Kp * ppv
            self.P[iv][ip] = pvp - Kv * ppp
            self.P[iv][iv] = pvv - Kv * ppv


# ---------------------------------------------------------------- 对比
def main():
    np.random.seed(3)
    a, b = RefKF(), PortKF()
    a.start(400.0, 240.0)
    b.start(400.0, 240.0)

    dt = 0.465          # 实测检测周期
    # 目标以 120 px/s 向右、40 px/s 向下匀速走, 观测带噪声
    tx, ty, vx, vy = 400.0, 240.0, 120.0, 40.0
    max_dx = 0.0
    max_dP = 0.0

    print("step   ref_x     port_x    ref_vx   port_vx   maxdiff")
    for i in range(25):
        tx += vx * dt
        ty += vy * dt
        zx = tx + np.random.randn() * 5
        zy = ty + np.random.randn() * 5

        a.predict(dt)
        b.predict(dt)
        a.update(zx, zy)
        b.update(zx, zy)

        dx = max(abs(a.x[j, 0] - b.x[j]) for j in range(4))
        dP = max(abs(a.P[r, c] - b.P[r][c]) for r in range(4) for c in range(4))
        max_dx = max(max_dx, dx)
        max_dP = max(max_dP, dP)
        if i % 5 == 0 or i == 24:
            print("%4d  %8.2f  %8.2f  %7.2f  %7.2f  %.2e"
                  % (i, a.x[0, 0], b.x[0], a.x[2, 0], b.x[2], dx))

    print()
    print("状态最大偏差   %.3e" % max_dx)
    print("协方差最大偏差 %.3e" % max_dP)
    ok = max_dx < 1e-9 and max_dP < 1e-9
    print("结论: " + ("手工展开与标准矩阵实现一致" if ok else "存在差异, 需排查"))

    print()
    print("--- 收敛性: 真实速度 vx=120, vy=40 ---")
    print("估计 vx=%.1f  vy=%.1f" % (b.x[2], b.x[3]))
    print("位置误差 %.1f px" % abs(b.x[0] - tx))

    # 前馈补偿效果
    print()
    print("--- 延迟前馈 (lead=0.465s) ---")
    lead = 0.465
    nolead = abs(b.x[0] - (tx + vx * lead))
    withlead = abs((b.x[0] + b.x[2] * lead) - (tx + vx * lead))
    print("目标 0.465s 后实际位置 %.1f" % (tx + vx * lead))
    print("不补偿输出 %.1f, 误差 %.1f px" % (b.x[0], nolead))
    print("补偿后输出 %.1f, 误差 %.1f px" % (b.x[0] + b.x[2] * lead, withlead))
    print("误差降低 %.0f%%" % (100 * (1 - withlead / nolead)) if nolead > 0 else "")


if __name__ == "__main__":
    main()
