from pathlib import Path
import hashlib,zipfile
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=f'''N10P 雷达优化验证记录 — 2026-09-14
CHANGED: N10PDecoder.feed / SweepAssembler.add / RealLidarNode.publish_scan / scan_payload / project_point / RadarCanvas.paintEvent / refresh_lidar
读取: 108字节帧、460800波特率、累加校验、掉字节逐字节重同步、异常角度拒收、有效第二回波兜底、整圈发布、断流清空组帧、不再重复发布旧帧。
显示: ROS +90度统一为左，遍历全部半度点做四向测距；点径8px改3.2px、静态网格缓存、20Hz最新帧邮箱；RTK卫星不再混入雷达测距平面。
状态: 有效点数/圈频/帧龄/暂停点云/量程选中；停流和空回波标记未知，不报告安全。
时间: scan_time为主机接收的一圈周期；stamp估算圈开始，非硬件时钟；角度桶重排后time_increment=0，不编造逐点时间。

MODIFIED_FILE: {D/'MODIFIED_FILE.zip'}
DIFF_FILE: {D/'DIFF_FILE.patch'}
VERIFICATION: {D/'VERIFICATION.txt'}
ROLLBACK: {D/'ROLLBACK.sh'}
冻结测试源码: {D/'FROZEN_RELEASE'}
原始源码: {D/'BASELINE'}
原始串口: {D/'n10p-raw.bin'}
RAW_SHA256: {sha(D/'n10p-raw.bin')}
'''
for p in sorted((D/'BASELINE').glob('*.py')):r+=f'BASELINE_SHA256 {p.name}: {sha(p)}\n'
for p in sorted((D/'FROZEN_RELEASE').glob('*.py')):r+=f'MODIFIED_SHA256 {p.name}: {sha(p)}\n'
r+=f'''\nCOMMAND_PREFIX: D="{D}"
BASELINE_COMMAND: python3 "$D/test_pipeline.py" "$D/BASELINE"
BASELINE_INPUT: 合成协议/掉字节/异常角度/双回波/空圈/停流用例 + 同一107713字节真实串口记录。
BASELINE_LITERAL_OUTPUT:
{(D/'baseline.out').read_text()}BASELINE_EXIT: {(D/'baseline.exit').read_text().strip()}
MODIFIED_COMMAND: python3 "$D/test_pipeline.py" "$D/FROZEN_RELEASE"
MODIFIED_INPUT: 与BASELINE相同。
MODIFIED_LITERAL_OUTPUT:
{(D/'modified.out').read_text()}MODIFIED_EXIT: {(D/'modified.exit').read_text().strip()}
ROLLBACK_PREPARE: cp -R "$D/FROZEN_RELEASE" "$D/rollback_frozen"
ROLLBACK_COMMAND: "$D/ROLLBACK.sh" "$D/rollback_frozen"
ROLLBACK_INPUT: 冻结修改版的独立副本。
ROLLBACK_LITERAL_OUTPUT: {(D/'rollback.out').read_text().strip()}
ROLLBACK_EXIT: {(D/'rollback.exit').read_text().strip()}
RESTORED_BEHAVIOR_COMMAND: python3 "$D/test_pipeline.py" "$D/rollback_frozen"
RESTORED_BEHAVIOR_LITERAL_OUTPUT:
{(D/'rollback-behavior.out').read_text()}RESTORED_BEHAVIOR_EXIT: {(D/'rollback-behavior.exit').read_text().strip()}
RESTORED_STATUS: 原2文件哈希及测试输出恢复为BASELINE，新增模块删除。原有8项失败重新出现，退出1为预期。冻结修改版保持改进内容。
ROLLBACK.sh按哈希保护后续改动；--board依赖同目录rollback_board.sh；板端回滚分支未实际执行。

GUI_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; QT_QPA_PLATFORM=offscreen python3 /tmp/n10p_test_gui.py /tmp/n10p-opt'"
GUI_INPUT: 合成方向/暂停/失联/恢复/空圈、真实scan样本，1920x1080及1600x900。
GUI_LITERAL_OUTPUT: {(D/'gui.out').read_text().strip()}
GUI_EXIT: 0
Qt offscreen另有XDG_RUNTIME_DIR和propagateSizeHints提示，无测试异常。

LIVE_COMMAND: adb shell "bash -c 'source /opt/ros/jazzy/setup.bash; python3 /tmp/n10p_live_probe.py /tmp/n10p-scan-after.json'"
LIVE_INPUT: 实际/scan和/lidar/status，5秒探针包含ROS发现时间。
LIVE_BASELINE_LITERAL_OUTPUT: {(D/'live-baseline.out').read_text().strip()}
LIVE_MODIFIED_LITERAL_OUTPUT: {(D/'live-final.out').read_text().strip()}
LIVE_EXIT: {(D/'live-final.exit').read_text().strip()}
原20Hz含旧帧重复发布；新版约10Hz是整圈更新，不是雷达转速降低。
旧帧重盖时间戳的2.4ms与新版含采集周期的101.5ms不可当作真实延迟前后比较。
累计3次校验/头错误已丢弃并重同步；记录末尾angle_errors=0、overruns=0。

真实断流试验: kill -STOP 21523；1.2秒后截图；EXIT trap执行kill -CONT 21523。
断流: 点云清空、四向--、环境状态未知，相机持续刷新。
恢复: stale=false，点云/距离恢复；恢复截图10.0Hz、406/720点。
目视复核截图:
{D/'n10p-before.png'}
{D/'n10p-stale.png'}
{D/'n10p-final.png'}

部署及并行编辑记录:
14:23已部署GUI和雷达节点；14:25调整串口轮询为10ms；其后三文件板端哈希均通过。
14:27起检测到工作区及板端GUI又被其他过程修改CORS/相机代码，且修改影响了原MODIFIED_FILE目录。
为保留新增代码，未用旧副本覆盖活动文件。交付压缩包使用FROZEN_RELEASE锁定此前已测试版本。
14:28板端新增代码先后出现CorsConfigDialog/CORS_CONFIG_PATH未定义，GUI退出；该并行变更非本次雷达修改。
因此早先运行截图代表本次已验证部署，不代表后续并行编辑后的程序。最终现场状态见本记录末尾追加。
板端备份目录: /root/radar_system/.n10p-before-20260914
活动路径: /root/radar_system/board_radar_gui.py、/root/radar_system/real_lidar_node.py、/root/radar_system/n10p_pipeline.py

协议参考: https://github.com/Lslidar/Lslidar_ROS2_driver/blob/8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa/lslidar_driver/src/lslidar_driver.cc
已核对厂商N10_P的108字节、16组双回波、7/5/105偏移、460800波特率与累加校验。
'''
(D/'VERIFICATION.txt').write_text(r)
with zipfile.ZipFile(D/'MODIFIED_FILE.zip') as z:
    assert z.testzip() is None
    for p in (D/'FROZEN_RELEASE').glob('*.py'):assert z.read(p.name)==p.read_bytes()
for name in ['MODIFIED_FILE.zip','DIFF_FILE.patch','VERIFICATION.txt','ROLLBACK.sh']:
    p=D/name; assert p.read_bytes(); print('reopened',p,'bytes='+str(p.stat().st_size))
assert (D/'ROLLBACK.sh').stat().st_mode & 0o111
print('archive_verified=3 rollback_executable=1')
