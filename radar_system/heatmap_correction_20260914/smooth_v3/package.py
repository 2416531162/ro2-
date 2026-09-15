from pathlib import Path
import ast,hashlib,difflib
D=Path(__file__).resolve().parent;b=(D/'BASELINE.py').read_text();m=(D/'MODIFIED_FILE.py').read_text()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['decode_depth_mm','depth_center_mm','depth_quality','render_depth_heatmap','RadarCanvas','CorsConfigDialog']:
    find=lambda s:next(n for n in ast.parse(s).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name)
    assert ast.dump(find(b))==ast.dump(find(m))
assert sha(D/'MODIFIED_FILE.py')==sha(D.parents[1]/'board_radar_gui.py')
assert sha(D/'rollback_test.py')==sha(D/'BASELINE.py')
(D/'DIFF_FILE.patch').write_text(''.join(difflib.unified_diff(b.splitlines(True),m.splitlines(True),fromfile='a/board_radar_gui.py',tofile='b/board_radar_gui.py')))
r=f'''平滑展示 / 原始测距分离 — 2026-09-14
CHANGED: prepare_display_depth / render_depth_display / DepthDisplayWorker / refresh_depth / depth_style
本次实际增加处理算法与显示模式，不是改名。
显示原理：原始深度->独立float32显示副本->3x3 bilateral(range sigma35mm, spatial sigma1)->有条件小洞估算->固定色标->Qt平滑缩放。
原始模式：直接复用原render_depth_heatmap，逐像素输出一致，无估算。
补洞限制：封闭连通域面积<=9像素，不接触图像边缘；至少5个原始有效3x3邻居；邻域最大/最小差<=100mm。每次仅基于原始邻居计算，估算不迭代扩散。
估算标记：单独estimated布尔mask，图上灰白斜纹标记，面板明确显示“展示估算 N px（灰纹）”；估算不计入原始有效率。
大空洞保持缺失；深度断面不被当作平面进行大范围平均；不使用RGB底图，不按直方图改色标，不做跨帧累积。
测量：depth_center_mm、depth_quality、decode_depth_mm、render_depth_heatmap AST均未改变；距离仍从result.record.array原始深度取得，同一帧切换平滑/原始不改变测距。
后台：DepthDisplayWorker最新请求覆盖旧请求，避免FIFO积压；Qt只接收完成结果，输出与量程/模式匹配后显示；停止时join工作线程避免OpenCV后台析构异常。
性能：移除不必要的全图2倍中间缓冲，Qt按实际尺寸缩放；OpenCV线程数设2，关闭OpenCL；计时只为本设备样本结果。
保留：RadarCanvas与CorsConfigDialog AST一致；雷达、相机、AI模型/推理程序未修改或重启。

MODIFIED_FILE: {D/'MODIFIED_FILE.py'}
DIFF_FILE: {D/'DIFF_FILE.patch'}
VERIFICATION: {D/'VERIFICATION.txt'}
ROLLBACK: {D/'ROLLBACK.sh'}
BASELINE_SHA256: {sha(D/'BASELINE.py')}
MODIFIED_SHA256: {sha(D/'MODIFIED_FILE.py')}
LOCAL_ACTIVE: {D.parents[1]/'board_radar_gui.py'}
BOARD_ACTIVE: /root/radar_system/board_radar_gui.py
BOARD_BACKUP: /root/radar_system/board_radar_gui.py.before-smooth-v3

BASELINE_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/depth-smooth/test.py /tmp/depth-smooth/BASELINE.py'"
BASELINE_INPUT: 种子42的带15mm噪声平面、小单像素空洞、12x15大空洞、1m/3m深度断面、相同真实深度样本。
BASELINE_LITERAL_OUTPUT:
{(D/'baseline.out').read_text()}BASELINE_EXIT: {(D/'baseline.exit').read_text().strip()}
说明：基线尚无新增展示层接口，8项新功能契约失败，不表示基线原始测距均有问题。

MODIFIED_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/depth-smooth/test.py /tmp/depth-smooth/MODIFIED_FILE.py'"
MODIFIED_INPUT: 与BASELINE相同；检查源数据不变、平面噪声下降、估算mask、缺失保留、断面、斜纹、原始模式一致、测量不变。
MODIFIED_LITERAL_OUTPUT:
{(D/'modified.out').read_text()}MODIFIED_EXIT: {(D/'modified.exit').read_text().strip()}

COMMAND_PREFIX: D="{D}"
ROLLBACK_PREPARE: cp "$D/MODIFIED_FILE.py" "$D/rollback_test.py"
ROLLBACK_COMMAND: "$D/ROLLBACK.sh" "$D/rollback_test.py"
ROLLBACK_INPUT: 修改版独立副本，不触碰活动程序或交付文件。
ROLLBACK_LITERAL_OUTPUT: {(D/'rollback.out').read_text().strip()}
ROLLBACK_EXIT: {(D/'rollback.exit').read_text().strip()}
RESTORED_BEHAVIOR_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/depth-smooth/test.py /tmp/depth-smooth/rollback_test.py'"
RESTORED_BEHAVIOR_LITERAL_OUTPUT:
{(D/'rollback-behavior.out').read_text()}RESTORED_BEHAVIOR_EXIT: {(D/'rollback-behavior.exit').read_text().strip()}
RESTORED_STATUS: 原始哈希恢复、8项新接口契约重新失败，与BASELINE相同；运行耗时是浮动指标无需逐字匹配。修改版与活动程序保持更改。
ROLLBACK_USAGE: ROLLBACK.sh接收本地修改副本路径，哈希保护后恢复；板端备份保留，未执行活动板端回滚。

GUI_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; QT_QPA_PLATFORM=offscreen python3 /tmp/depth-smooth/test_gui.py /tmp/depth-smooth/MODIFIED_FILE.py'"
GUI_INPUT: 真实深度样本；平滑/原始切换、原始测距不变、量程切换、停流/恢复、后台工作线程关闭、RGB模式切换。
GUI_LITERAL_OUTPUT: {(D/'gui.out').read_text().strip()}
GUI_EXIT: {(D/'gui.exit').read_text().strip()}
Qt另有XDG_RUNTIME_DIR/propagateSizeHints提示；最终退出码0，未出现异常栈或析构中止。

REAL_REPLAY: 原始深度307200像素；样本中仅196像素为显示估算(0.064%)；不表示以后每帧都固定196。
FINAL_GUI_PID: 43323
FINAL_BOARD_SHA256: a258d2253316494a429e6c0198243a9200562f482ba330b4562f886bc944c02b
保留雷达PID22771、AI PID22772。
截图:
{D/'depth-smooth-preview.png'} （实际Qt离线真实数据回放）
{D/'smooth-live.png'} （板端当前画面，抓图时已选AI模式；平滑切换控件和此前深度显示估算计数存在）
现场截图的29FPS属于AI/RGB显示，不用作平滑深度帧率；本次平滑渲染性能以真实样本median_ms记录为准。
边界：这是距离数据的平滑可视化，不产生温度数据；相同距离的平面仍会呈相近颜色，大面积未测到区域仍为空，不声称已经恢复完整场景。
'''
(D/'VERIFICATION.txt').write_text(r)
for name in ['MODIFIED_FILE.py','DIFF_FILE.patch','VERIFICATION.txt','ROLLBACK.sh']:
    p=D/name;assert p.read_bytes();print('reopened',str(p),'bytes='+str(p.stat().st_size))
assert (D/'ROLLBACK.sh').stat().st_mode&0o111
print('measurement_ast=unchanged raw_rollback_hash=matched local_release=matched')
