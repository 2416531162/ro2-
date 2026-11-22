"""作业速度默认值及分段上限；弯道、倒车、定位接管仍按各自更低的限速。"""
DEFAULT_PAINT_SPEED = 0.8
DEFAULT_TRANSIT_SPEED = 1.5
MAX_SECTION_SPEED = 1.5
# 正常前进转场/掉头的弯道速度（转场弯、圆弧掉头、圆弧进线，不喷）；弯前平滑降速。2026-11-22 用户「圆弧转弯现在太慢了……
# 车速提到0.45」：0.35 → 0.45（原来 R 1.5 m 实车只走 0.39，圆弧进线 9 m 弧要 23 s）
TURN_SPEED_MPS = .45
SHORT_RUNUP_MPS = .35  # 短直线助跑限速（_short_runup_here），原来借用 TURN_SPEED_MPS 0.35，弯道提速时不跟着变
PAINT_ARC_SPEED_MPS = .5  # 画圆和画弧线独立定速，不归入普通转场过弯。
# 差速车轮子最低速度（10-08 用户「这个轮子最低不要低于0.3最好是」）：R 1 m 转场弧 0.35 m/s 时内侧轮要 0.20 m/s，
# 实际跑到 0.28（驱动器低速读数不准），车只转出 79 %、偏 12–17 cm。转场弧按内侧轮不低于它提速（不超过横向加速度）
WHEEL_MIN_MPS = .3


def transit_bend_speed(kappa, track_m, lat_accel_mps2, base=TURN_SPEED_MPS):
    """差速车前进转场弯（不喷）的目标后轴速度：base，弯急时提到内侧轮 v·(1 − |κ|·轨距/2) ≥ WHEEL_MIN_MPS，
    往上提的部分不超过横向加速度 √(a/|κ|) 和 MAX_SECTION_SPEED（base 0.45：R 1 m 以上都是 0.45；R ≤ 1 m 的弯
    base 本身就超过横向加速度 0.20（R 1 m 0.20、R 0.5 m 0.41 m/s²），照 base 走，和原来 0.35 时 R < 0.6 m 一样）。"""
    half = track_m / 2.
    k = abs(kappa)
    if not 0. < k * half < 1. or lat_accel_mps2 <= 0.:
        return base
    return max(base, min(WHEEL_MIN_MPS / (1. - k * half), (lat_accel_mps2 / k) ** .5, MAX_SECTION_SPEED))
