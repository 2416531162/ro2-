from pathlib import Path
D=Path(__file__).parent;s=(D/'BASELINE.py').read_text()
s=s.replace('深度热力图','深度距离图').replace('🌐 深度距离图','深度距离图 · Z / m')
s=s.replace('DARK = NO RETURN','DARK = INVALID / OUT OF RANGE')
s=s.replace('深度距离图 · 非温度图 · 暗灰表示无回波','深度距离图 · 光轴深度 Z / m · 暗灰表示无效或超范围')
helper='''def depth_quality(depth, far_mm):
    finite=np.isfinite(depth)
    missing=(~finite)|(depth<=0)|(depth==65535)
    valid=finite&(depth>=DEPTH_VALID_MIN_MM)&(depth<=DEPTH_VALID_MAX_MM)
    outside=(~missing)&(~valid)
    return dict(valid=float(np.mean(valid)),missing=float(np.mean(missing)),
                outside=float(np.mean(outside)),clipped=float(np.mean(valid&(depth>far_mm))))


'''
s=s.replace('def depth_center_mm(depth):',helper+'def depth_center_mm(depth):')
s=s.replace("        self.depth_hint.setText(f'纯深度 Z · 有效 {ratio:.0%} / 无回波 {1-ratio:.0%} · 红近蓝远 · 超色标截断')", "        quality=depth_quality(depth,far)\n        self.depth_hint.setText(f\"深度 Z/m · 有效 {quality['valid']:.0%} · 无效 {quality['missing']:.0%} · 超范围 {quality['outside']:.0%} · 色标饱和 {quality['clipped']:.0%}\")")
s=s.replace("        valid=np.isfinite(depth)&(depth>=DEPTH_VALID_MIN_MM)&(depth<=DEPTH_VALID_MAX_MM)\n        ratio=float(np.mean(valid))\n",'')
(D/'MODIFIED_FILE.py').write_text(s)
