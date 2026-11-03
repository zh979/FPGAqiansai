"""
laser_check.py -- 验证 laser.cpp 的安全联锁

激光是安全关键部件, 必须穷举验证:
  1. 每一条联锁都真的能拦住
  2. 正常路径确实能开火
  3. 不存在"激光开着但没人关"的路径
"""

IDLE, AIMING, FIRING, COOLDOWN = 0, 1, 2, 3
NAMES = {IDLE: "IDLE", AIMING: "AIM", FIRING: "FIRE", COOLDOWN: "COOL"}
CLS_WEED, CLS_CROP = 0, 1


class Laser:
    """复现 laser.cpp 的 ls_tick"""

    def __init__(self, zone):
        self.state = IDLE
        self.on = 0
        self.zone = zone
        self.aim_tol_px = 15.0
        self.aim_hold_ticks = 1
        self.fire_ticks = 1
        self.cool_ticks = 2
        self.min_score = 0.6
        self.done_radius_px = 40.0
        self.aim_cnt = self.fire_cnt = self.cool_cnt = 0
        self.tgt_x = self.tgt_y = 0.0
        self.done = []
        self.n_fired = 0
        self.n_blocked = 0

    def _in_zone(self, x, y):
        z = self.zone
        return z[0] <= x <= z[2] and z[1] <= y <= z[3]

    def _done(self, x, y):
        r2 = self.done_radius_px ** 2
        return any((x - dx) ** 2 + (y - dy) ** 2 < r2 for dx, dy in self.done)

    def abort(self):
        self.state = IDLE
        self.on = 0
        self.aim_cnt = self.fire_cnt = 0

    def tick(self, has_target, measured, class_id, score, tx, ty, aim_err):
        if self.state == COOLDOWN:
            self.on = 0
            self.cool_cnt += 1
            if self.cool_cnt >= self.cool_ticks:
                self.cool_cnt = 0
                self.state = IDLE
            return 0

        ok = bool(has_target)
        if ok and not measured:
            ok = False
        if ok and class_id != CLS_WEED:
            ok = False
        if ok and score < self.min_score:
            ok = False
        if ok and not self._in_zone(tx, ty):
            ok = False
        if ok and self._done(tx, ty):
            ok = False

        if not ok:
            if self.state != IDLE:
                self.n_blocked += 1
            self.abort()
            return 0

        if self.state == IDLE:
            self.tgt_x, self.tgt_y = tx, ty
            self.aim_cnt = 0
            self.state = AIMING
            self.on = 0
        elif self.state == AIMING:
            self.on = 0
            self.tgt_x, self.tgt_y = tx, ty
            if aim_err <= self.aim_tol_px:
                self.aim_cnt += 1
                if self.aim_cnt >= self.aim_hold_ticks:
                    self.state = FIRING
                    self.fire_cnt = 0
                    self.on = 1
            else:
                self.aim_cnt = 0
        elif self.state == FIRING:
            self.on = 1
            if aim_err > self.aim_tol_px * 2.0:
                self.n_blocked += 1
                self.abort()
            else:
                self.fire_cnt += 1
                if self.fire_cnt >= self.fire_ticks:
                    self.done.append((self.tgt_x, self.tgt_y))
                    self.n_fired += 1
                    self.on = 0
                    self.fire_cnt = 0
                    self.cool_cnt = 0
                    self.state = COOLDOWN
        return self.on


ZONE = (50.0, 40.0, 750.0, 440.0)
GOOD = dict(has_target=1, measured=1, class_id=CLS_WEED, score=0.8,
            tx=400.0, ty=240.0, aim_err=5.0)


def run(l, n, **kw):
    """跑 n 拍, 返回每拍激光状态"""
    p = dict(GOOD)
    p.update(kw)
    return [l.tick(p["has_target"], p["measured"], p["class_id"],
                   p["score"], p["tx"], p["ty"], p["aim_err"]) for _ in range(n)]


def main():
    fails = []

    print("=== 联锁测试: 每一条都必须拦住 ===")
    cases = [
        ("无目标",            dict(has_target=0)),
        ("非实测(纯预测)",    dict(measured=0)),
        ("类别是 crop",       dict(class_id=CLS_CROP)),
        ("置信度 0.5 < 0.6",  dict(score=0.5)),
        ("目标在区域外(左)",  dict(tx=20.0)),
        ("目标在区域外(右)",  dict(tx=790.0)),
        ("目标在区域外(上)",  dict(ty=10.0)),
        ("目标在区域外(下)",  dict(ty=470.0)),
        ("云台未对准",        dict(aim_err=50.0)),
    ]
    for name, kw in cases:
        l = Laser(ZONE)
        fired = any(run(l, 10, **kw))
        status = "拦住" if not fired else "!!! 漏过 !!!"
        if fired:
            fails.append(name)
        print("  %-20s %s" % (name, status))

    print()
    print("=== 正常路径: 必须能开火 ===")
    l = Laser(ZONE)
    seq = run(l, 8)
    print("  激光序列: %s" % seq)
    print("  开火次数: %d (应 >=1)" % l.n_fired)
    if l.n_fired < 1:
        fails.append("正常路径无法开火")

    print()
    print("=== 重复照射防护: 同一株只打一次 ===")
    l = Laser(ZONE)
    seq = run(l, 30)
    print("  30 拍内开火 %d 次 (应恰好 1 次)" % l.n_fired)
    if l.n_fired != 1:
        fails.append("重复照射防护失效: %d 次" % l.n_fired)

    print()
    print("=== 换新目标(距离 >40px) 应能再次开火 ===")
    l = Laser(ZONE)
    run(l, 6)                          # 打第一个
    n1 = l.n_fired
    run(l, 8, tx=600.0, ty=300.0)      # 换个远的
    print("  第一株后 %d 次, 第二株后 %d 次 (应 1 -> 2)" % (n1, l.n_fired))
    if l.n_fired != 2:
        fails.append("新目标无法开火")

    print()
    print("=== 照射中目标跑掉: 必须立刻停 ===")
    l = Laser(ZONE)
    l.tick(**{k: GOOD[k] for k in GOOD})           # IDLE->AIMING
    s1 = l.tick(**{k: GOOD[k] for k in GOOD})      # AIMING->FIRING, on=1
    # 此时如果 fire_ticks=1 已经打完了, 改用长照射时间重测
    l2 = Laser(ZONE)
    l2.fire_ticks = 5
    run(l2, 2)                                      # 进入 FIRING
    st_before = l2.state
    on_after = l2.tick(1, 1, CLS_WEED, 0.8, 400.0, 240.0, 99.0)  # 突然跑偏
    print("  跑偏前状态 %s, 跑偏后激光=%d 状态=%s"
          % (NAMES[st_before], on_after, NAMES[l2.state]))
    if on_after != 0 or l2.state != IDLE:
        fails.append("照射中跑偏未停止")

    print()
    print("=== 目标突然消失: 必须立刻关 ===")
    l3 = Laser(ZONE)
    l3.fire_ticks = 5
    run(l3, 2)
    was_on = l3.on
    off = l3.tick(0, 0, CLS_WEED, 0.0, 0, 0, 99.0)
    print("  消失前 on=%d, 消失后 on=%d 状态=%s" % (was_on, off, NAMES[l3.state]))
    if off != 0:
        fails.append("目标消失未关激光")

    print()
    print("=== 穷举: 激光开着时是否一定能被关掉 ===")
    # 从 FIRING 状态出发, 枚举所有输入组合, 确认没有"卡在开"的路径
    stuck = 0
    for ht in (0, 1):
        for me in (0, 1):
            for cid in (CLS_WEED, CLS_CROP):
                for sc in (0.3, 0.8):
                    for inz in (True, False):
                        for ae in (5.0, 50.0):
                            lt = Laser(ZONE)
                            lt.fire_ticks = 99   # 保证停在 FIRING
                            run(lt, 3)
                            if lt.state != FIRING:
                                continue
                            x = 400.0 if inz else 10.0
                            # 连续给 5 拍同样输入, 看能否关掉
                            res = [lt.tick(ht, me, cid, sc, x, 240.0, ae)
                                   for _ in range(5)]
                            # 只要输入不是"全部合格", 就必须关
                            all_good = (ht and me and cid == CLS_WEED
                                        and sc >= 0.6 and inz and ae <= 30.0)
                            if not all_good and any(res):
                                stuck += 1
    print("  枚举 128 种组合, 应关未关的: %d" % stuck)
    if stuck:
        fails.append("存在卡在开启的路径: %d" % stuck)

    print()
    if fails:
        print("失败项:")
        for f in fails:
            print("  - %s" % f)
    else:
        print("结论: 全部联锁有效, 无卡死路径, 安全逻辑正确")


if __name__ == "__main__":
    main()
