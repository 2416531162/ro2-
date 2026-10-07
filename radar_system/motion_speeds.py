"""作业速度默认值及分段上限；弯道、倒车、定位接管仍按各自更低的限速。"""
DEFAULT_PAINT_SPEED = 0.8
DEFAULT_TRANSIT_SPEED = 1.5
MAX_SECTION_SPEED = 1.5
TURN_SPEED_MPS = .35  # 正常前进转场/掉头的弯道速度；弯前平滑降速。
PAINT_ARC_SPEED_MPS = .5  # 画圆和画弧线独立定速，不归入普通转场过弯。
