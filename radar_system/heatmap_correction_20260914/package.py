from pathlib import Path
import hashlib,difflib,ast
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=(D/'BASELINE.py').read_text();after=(D/'MODIFIED_FILE.py').read_text()
(D/'DIFF_FILE.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='a/board_radar_gui.py',tofile='b/board_radar_gui.py')))
a=ast.parse(before);b=ast.parse(after)
for name in ['RadarCanvas','CorsConfigDialog']:
    pick=lambda t:next(n for n in t.body if isinstance(n,ast.ClassDef) and n.name==name)
    assert ast.dump(pick(a))==ast.dump(pick(b))
assert sha(D/'MODIFIED_FILE.py')==sha(D.parent/'board_radar_gui.py')
report=f'''深度热力图显示修正 — 2026-09-14
当前：纯深度版已部署，板端保持热力图页面。

CHANGED: render_depth_heatmap / _draw_heatmap_legend / decode_depth_mm / depth_center_mm / refresh_depth / depth_range
原因：原版热图混合72%热色+28%RGB，并用暗RGB填补大空洞，形态闭运算/膨胀补小洞；高斯alpha造成无效边界带色；动态百分位色标让同一米数随场景变色。
修改：完全移除RGB混合、补洞及alpha混合。有效像素由原始深度直接映射256色连续LUT；无效像素统一暗灰，近红远蓝。
量程：默认0.2–2.0m，可选0.2–4.5m或0.2–5.5m；固定量程不随画面改变，超色标显示端点色，超过5500mm按无效处理。
色标：5个米制刻度，和图像共用同一LUT；注明深度Z、红近蓝远、暗色无回波；等比例显示，留出内边距防止底部文字被裁切。
读取：16UC1毫米/32FC1米、大端/小端、step行填充；最新深度帧邮箱、25Hz刷新上限；超过0.6秒过期清空旧颜色/距离。
测距：中心11x11区域有效率>=35%、至少12点才取中位数；跨深度面则返回未知。仅更改热图测点，不替换AI检测算法。
保留：RadarCanvas、CorsConfigDialog AST完全相同；未修改或重启雷达、AI推理与相机驱动。

MODIFIED_FILE: {D/'MODIFIED_FILE.py'}
DIFF_FILE: {D/'DIFF_FILE.patch'}
VERIFICATION: {D/'VERIFICATION.txt'}
ROLLBACK: {D/'ROLLBACK.sh'}
BASELINE_SHA256: {sha(D/'BASELINE.py')}
MODIFIED_SHA256: {sha(D/'MODIFIED_FILE.py')}
LOCAL_ACTIVE: {D.parent/'board_radar_gui.py'}
BOARD_ACTIVE: /root/radar_system/board_radar_gui.py
BOARD_BACKUP: /root/radar_system/board_radar_gui.py.before-pure-depth-20260914

BASELINE_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/heatmap-correction/test_heatmap.py /tmp/heatmap-correction/BASELINE.py'"
BASELINE_INPUT: 同一合成梯度/不同RGB/深度空洞/端点/行填充/字节序/中心异常用例及真实camera-baseline/depth.bin、rgb.bin。
BASELINE_LITERAL_OUTPUT:
{(D/'baseline.out').read_text()}BASELINE_EXIT: {(D/'baseline.exit').read_text().strip()}

MODIFIED_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/heatmap-correction/test_heatmap.py /tmp/heatmap-correction/MODIFIED_FILE.py'"
MODIFIED_INPUT: 与BASELINE相同。
MODIFIED_LITERAL_OUTPUT:
{(D/'modified.out').read_text()}MODIFIED_EXIT: {(D/'modified.exit').read_text().strip()}

COMMAND_PREFIX: D="{D}"
ROLLBACK_PREPARE: cp "$D/MODIFIED_FILE.py" "$D/rollback_test.py"
ROLLBACK_COMMAND: "$D/ROLLBACK.sh" "$D/rollback_test.py"
ROLLBACK_INPUT: 修改版独立副本，活动源码及交付修改版不参与恢复。
ROLLBACK_LITERAL_OUTPUT: {(D/'rollback.out').read_text().strip()}
ROLLBACK_EXIT: {(D/'rollback.exit').read_text().strip()}
RESTORED_BEHAVIOR_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/heatmap-correction/test_heatmap.py /tmp/heatmap-correction/rollback_test.py'"
RESTORED_BEHAVIOR_LITERAL_OUTPUT:
{(D/'rollback-behavior.out').read_text()}RESTORED_BEHAVIOR_EXIT: {(D/'rollback-behavior.exit').read_text().strip()}
RESTORED_STATUS: 独立副本恢复原始哈希及原有1通过8失败行为；与BASELINE输出逐字节相同；MODIFIED_FILE和活动源码保留改进版。
ROLLBACK_USAGE: "$D/ROLLBACK.sh" 恢复本地活动文件；"$D/ROLLBACK.sh" --board 恢复板端并重启GUI，需要同目录restart_gui.py。
板端恢复分支已准备但未在活动程序执行；避免重新引入混合显示。

GUI_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; QT_QPA_PLATFORM=offscreen python3 /tmp/heatmap-correction/test_gui.py /tmp/heatmap-correction/MODIFIED_FILE.py'"
GUI_INPUT: 真实深度样本、量程切换、停流/恢复、AI/RGB切换，1920x1080。
GUI_LITERAL_OUTPUT: {(D/'gui.out').read_text().strip()}
GUI_EXIT: {(D/'gui.exit').read_text().strip()}
Qt另有XDG_RUNTIME_DIR/propagateSizeHints环境提示；无异常栈。

LIVE_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/probe_camera.py /tmp/heatmap-live-data'"
LIVE_INPUT: 实际RGB、depth_raw及相机内参，约9秒采集。
LIVE_LITERAL_OUTPUT:
{(D/'live.out').read_text()}LIVE_EXIT: {(D/'live.exit').read_text().strip()}
RAW_LITERAL_OUTPUT: {(D/'raw-stats.out').read_text().strip()}
原始307200像素中195864是零值(63.76%)，6500超出配置有效范围(2.12%)，104836有效(34.13%)。
因此约66%的暗区来自采集无回波/范围筛选，不是再做渐变就能恢复的数据；平面同距呈相近颜色属距离图的正常含义。
当前探针深度23.96Hz，现场截图显示23FPS；仅为这段测量，不保证所有场景相同。
图像不是温度测量/红外热成像；色标单位为米。

现场截图（已重新打开目视确认数值色标完整、无RGB底图、相机与雷达仍运行）:
{D/'heatmap-live.png'}
现场样本目录: {D/'heatmap-live-data'}
GUI PID: 26077（本次部署启动）；雷达与AI PID 22771/22772保持。
'''
(D/'VERIFICATION.txt').write_text(report)
for name in ['MODIFIED_FILE.py','DIFF_FILE.patch','VERIFICATION.txt','ROLLBACK.sh']:
    p=D/name;assert p.read_bytes();print('reopened',p,'bytes='+str(p.stat().st_size))
assert (D/'ROLLBACK.sh').stat().st_mode & 0o111
print('local_active_matches=1 radar_cors_ast=unchanged rollback_executable=1')
