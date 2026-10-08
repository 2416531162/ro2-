"""作业速度默认值及分段上限；弯道、倒车、定位接管仍按各自更低的限速。"""
DEFAULT_PAINT_SPEED = 0.8
DEFAULT_TRANSIT_SPEED = 1.5
MAX_SECTION_SPEED = 1.5
TURN_SPEED_MPS = .35  # 正常前进转场/掉头的弯道速度；弯前平滑降速。
PAINT_ARC_SPEED_MPS = .5  # 画圆和画弧线独立定速，不归入普通转场过弯。
# 差速车轮子最低速度（10-08 用户「这个轮子最低不要低于0.3最好是」）：R 1 m 转场弧 0.35 m/s 时内侧轮要 0.20 m/s，
# 实际跑到 0.28（驱动器低速读数不准），车只转出 79 %、偏 12–17 cm。转场弧按内侧轮不低于它提速（不超过横向加速度）
WHEEL_MIN_MPS = .3


def transit_bend_speed(kappa, track_m, lat_accel_mps2, base=TURN_SPEED_MPS):
    """差速车前进转场弯（不喷）的目标后轴速度：base，弯急时提到内侧轮 v·(1 − |κ|·轨距/2) ≥ WHEEL_MIN_MPS，
    不超过横向加速度 √(a/|κ|) 和 MAX_SECTION_SPEED（R 1 m ≈ 0.45、R 1.5 m ≈ 0.39、R 2 m ≈ 0.36 m/s）。"""
    half = track_m / 2.
    k = abs(kappa)
    if not 0. < k * half < 1. or lat_accel_mps2 <= 0.:
        return base
    return max(base, min(WHEEL_MIN_MPS / (1. - k * half), (lat_accel_mps2 / k) ** .5, MAX_SECTION_SPEED))
