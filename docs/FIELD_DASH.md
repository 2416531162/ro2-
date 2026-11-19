# 场地作业虚线

App「划线线型」给图形加的 `dash`（`elements[].dash`，`{mode:'length',dash,gap}` 或 `{mode:'count',count}`）车端照着开关水泵；以前车端忽略它、整条喷实线。

## 区间怎么算

- `field_planner.dash_ranges`：与 App `FieldDash.ranges` 同一算法。线长模式从起点按「线 + 空」排，最后一截被线尾截短，线尾落在空隙里就不喷；段数模式线、空等长，两头都有线。线、空都不短于 5 cm，最多 500 截，超出报「图形名 + 原因」。
- 从每条原图形（折线的每条边、圆弧、整圆）的起点量；倒着画、整圆从别的角度起画都按原图形上的位置换算（`travel_spans`），画出来的各截和 App 地图上的虚线对齐。整圆首尾两截在原起点相接（段数模式）时合成一截，不在原起点停喷又开喷。
- 圆弧按喷头半径量，再换算成后轴路程。

## 计划和执行

- `motions[].paint_spans`：刷线段内各截的 [起, 止]（后轴沿段米数，升序），`paint_from` / `paint_to` 是第一截起点、最后一截终点；实线和转场段为 null。预览 `preview[].paint_spans_m` 同值。`paint_length_m` 按实际要喷的几截算。
- 车端开始作业时校验（`route_operations.field_paint_spans`）：1–2000 截、升序不重叠、首尾与 `paint_from` / `paint_to` 一致，否则拒绝开始。
- 水泵按 `Section.paint_spans` 每截开关（`route_operations.paint_status`），每截的停喷照直线越线 / 圆弧提前 5 cm 的规则。
- 车怎么开不受虚线影响：`Section.paint_dashed` 为真时跟踪器按 `paint_from..paint_to` 整条线走——车速、线头稳行、纠偏都和同一条实线一样，空隙里不提速、下一截不当新线头。没有这一条时空隙按转场处理，Gen5 仿真里每截 0.5 m/s ↔ 0.66 m/s 来回变。
- 断点继续、接入合段、任务存档恢复、断点前助跑都保留各截和 `paint_dashed`；补画按原图形位置截取，补画范围全落在空隙里会报「没有要喷的」。

## 验证

- `tests/test_field_dash.py`：区间与 App 一致、各种走法各截落在图上的线段上、计划 JSON、校验、续画、补画；软件底盘按车上喷线定速开一遍，生产喷涂判据只在各截内开泵、每截都喷到、车速与同图实线一致。
- Gen5 喷涂仿真（边线 1.0 / 0.6、整圆 6 段、圆弧 0.8 / 0.8、一条实线）：车端水泵只在各截内开、每截都喷到，虚线全程 0.5 m/s。
