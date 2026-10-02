"""整场图纸快照与局部补画：区间单位始终是喷头沿目标线的米数。"""
import copy
import math

import route_plan as rp
from field_planner import (Settings, parse_strokes, polygon, stroke_options,
                           target_points, transit, speed_for, score_key, add_scores)


def encode_source(job, fusion):
    """保存原图纸的经纬度，不从重新优化的行驶路径反推图纸。"""
    source = copy.deepcopy(job)
    source['boundary'] = [fusion.to_geodetic(*q) for q in job['boundary']]
    for item in source['elements']:
        item['points'] = [fusion.to_geodetic(*q) for q in item.get('points', [])]
        if item.get('center') is not None:
            x, y = item['center']
            angle = math.radians(item.get('start_deg', 0))
            item['heading_point'] = fusion.to_geodetic(x + math.cos(angle), y + math.sin(angle))
            item['center'] = fusion.to_geodetic(x, y)
    return source


def decode_source(source, fusion):
    job = copy.deepcopy(source)
    job['boundary'] = [list(fusion.to_local(*q)) for q in source['boundary']]
    for item in job['elements']:
        item['points'] = [list(fusion.to_local(*q)) for q in item.get('points', [])]
        if item.get('center') is not None:
            x, y = fusion.to_local(*item['center'])
            hx, hy = fusion.to_local(*item.pop('heading_point'))
            item['center'] = [x, y]
            item['start_deg'] = math.degrees(math.atan2(hy - y, hx - x))
    return job


def strokes(job):
    # 补画可显式选择原任务范围以外的源图形；原任务选边和断点不因此改变。
    return parse_strokes(job['elements'], max_strokes=20000)[0]


def length(stroke):
    return math.dist(stroke['a'], stroke['b']) if stroke['kind'] == 'line' else stroke['r'] * abs(stroke['sweep'])


def crop(stroke, start, end):
    total = length(stroke)
    for value in (start, end):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('补画范围必须为有限数值')
    if start < 0 or end > total + 1e-4 or end - start < rp.MIN_LEG_M:
        raise ValueError('补画范围须在线段内且至少 %.1f m' % rp.MIN_LEG_M)
    end = min(end, total)
    out = copy.deepcopy(stroke)
    if stroke['kind'] == 'line':
        a, b = stroke['a'], stroke['b']
        out['a'] = tuple(a[i] + (b[i] - a[i]) * start / total for i in (0, 1))
        out['b'] = tuple(a[i] + (b[i] - a[i]) * end / total for i in (0, 1))
    else:
        sign = 1 if stroke['sweep'] > 0 else -1
        # 全圆也固定在原图纸起角量区间；不能让全圆优化器重新挑起角。
        out.update(kind='arc', start=stroke['start'] + sign * start / stroke['r'],
                   sweep=sign * (end - start) / stroke['r'])
    return out


def drawing_points(stroke, max_points=128):
    if stroke['kind'] == 'line':
        return [list(stroke['a']), list(stroke['b'])]
    n = max(2, min(max_points - 1, math.ceil(length(stroke) / .2)))
    cx, cy = stroke['c']
    return [[cx + stroke['r'] * math.cos(stroke['start'] + stroke['sweep'] * i / n),
             cy + stroke['r'] * math.sin(stroke['start'] + stroke['sweep'] * i / n)] for i in range(n + 1)]


def catalog(job):
    source_strokes = strokes(job)
    # 遥测预览全图约 1000 点；实际执行仍使用精确直线和圆弧。
    budget = min(128, max(3, 1000 // len(source_strokes)))
    return [dict(id=i, name=s['name'], kind='polyline' if s['kind']=='line' else s['kind'], length_m=length(s),
                 source_element_id=s['key'][0], source_edge_index=s['key'][1], points=drawing_points(s,budget))
            for i, s in enumerate(source_strokes)]


def configuration(record, mission):
    saved = record['field_context']
    return Settings(saved['forward'], saved['right'], mission.body, mission.min_route_radius + .001,
                    record['cruise'], saved['transit_speed'], yaw_rate=mission.geo.pivot_rate_rad_s,
                    yaw_accel=mission.geo.pivot_accel_rad_s2, accel=mission.path_cfg.accel_mps2,
                    lateral_accel=mission.path_cfg.lat_accel_mps2)


def prepare(record, mission, command, car):
    job = decode_source(record['field_source'], mission.fusion)
    ident, index = command.get('element_id'), command.get('edge_index')
    if not isinstance(ident, str) or not 1 <= len(ident) <= 100:
        raise ValueError('补画需要原场地图形编号')
    if index is not None and (type(index) is not int or not 0 <= index < 400):
        raise ValueError('补画边编号无效')
    key = (command.get('element_id'), command.get('edge_index'))
    # 执行只解析显式选择的这一条，不能让整图目录数量扩大本次作业范围。
    candidates = parse_strokes(job['elements'], {key})[0]
    if len(candidates) != 1:
        raise ValueError('补画图形或边编号不存在，请重新选择')
    selected = crop(candidates[0], command.get('from_m'), command.get('to_m'))
    cfg = configuration(record, mission)
    _, planes = polygon(job['boundary'])
    choices = []
    for option in stroke_options(selected, cfg, planes):
        entry, score, _ = transit(car, option['start'], planes, cfg)
        choices.append((score_key(add_scores(score, option['score'])), option, entry,
                        max(score[1], option['score'][1])))
    if not choices:
        raise ValueError('喷头偏移或最小转弯半径不支持这段补画')
    _, option, entry, outside = min(choices, key=lambda v: v[0])
    entry_m = sum(s.path.length for s in entry)
    if entry_m > rp.MAX_ENTRY_M:
        raise ValueError('请先手动开到补画线附近，再预览接入')
    for sec in entry:
        sec.cruise_mps = speed_for(sec, cfg)
        sec.field_role = 'transit'
    sec = option['section']
    sec.cruise_mps = option['speed']
    sec.field_role = 'paint'
    target = [dict(direction=1, points=[list(p) for p in target_points(option)])]
    return [sec], entry, entry_m, outside, target, selected['name']
