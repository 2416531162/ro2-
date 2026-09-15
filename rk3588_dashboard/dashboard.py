#!/usr/bin/env python3
"""
====================================================================
  RK3588 极客硬件监控仪表盘 (RK3588 Geek Dashboard)
  - 硬件平台: 正点原子 ATK-DLRK3588
  - 系统支持: Ubuntu 24.04 / Debian / Linux 6.x
  - 功能特性: 
      1. 8核 CPU 实时频率与利用率 (4x A76 + 4x A55)
      2. 芯片各温区温度监控 (SoC / CPU / GPU / NPU)
      3. 内存与系统负载实时图表
      4. 板载 'work' LED 在线交互控制 (常亮 / 熄灭 / 心跳模式)
  - 依赖: 纯 Python 3 原生标准库，无须 pip 安装任何第三方包，离线即跑！
====================================================================
"""

import http.server
import socketserver
import json
import os
import re
import time
import threading
from urllib.parse import parse_qs, urlparse

PORT = 8080
LED_PATH = "/sys/class/leds/work"

# 全局保存上一次读取的 CPU 时间切片，用于精确计算利用率
cpu_last_stats = {}
cpu_usage_cache = [0.0] * 8
total_usage_cache = 0.0

def read_file(path, default=""):
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return default

def write_file(path, val):
    try:
        with open(path, "w") as f:
            f.write(str(val))
        return True
    except Exception as e:
        print(f"[Error writing {path}]: {e}")
        return False

def update_cpu_usage():
    """后台计算 8 核 CPU 利用率"""
    global cpu_last_stats, cpu_usage_cache, total_usage_cache
    stat_content = read_file("/proc/stat")
    if not stat_content:
        return

    lines = stat_content.splitlines()
    new_stats = {}
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        if name == "cpu" or (name.startswith("cpu") and name[3:].isdigit()):
            values = [int(x) for x in parts[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
            new_stats[name] = (idle, total)

    if cpu_last_stats:
        for i in range(8):
            key = f"cpu{i}"
            if key in new_stats and key in cpu_last_stats:
                prev_idle, prev_total = cpu_last_stats[key]
                cur_idle, cur_total = new_stats[key]
                diff_total = cur_total - prev_total
                diff_idle = cur_idle - prev_idle
                if diff_total > 0:
                    usage = (diff_total - diff_idle) / diff_total * 100.0
                    cpu_usage_cache[i] = round(max(0.0, min(100.0, usage)), 1)
        
        if "cpu" in new_stats and "cpu" in cpu_last_stats:
            prev_idle, prev_total = cpu_last_stats["cpu"]
            cur_idle, cur_total = new_stats["cpu"]
            diff_total = cur_total - prev_total
            diff_idle = cur_idle - prev_idle
            if diff_total > 0:
                total_usage_cache = round(max(0.0, min(100.0, (diff_total - diff_idle) / diff_total * 100.0)), 1)

    cpu_last_stats = new_stats

def cpu_monitor_loop():
    while True:
        try:
            update_cpu_usage()
        except Exception:
            pass
        time.sleep(1.0)

def get_cpu_info():
    """读取 8 个核心频率"""
    cores = []
    for i in range(8):
        freq_path = f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq"
        freq_khz_str = read_file(freq_path, "0")
        try:
            freq_mhz = round(int(freq_khz_str) / 1000.0, 1)
        except ValueError:
            freq_mhz = 0
        
        core_type = "A55 (小核)" if i < 4 else "A76 (大核)"
        cores.append({
            "id": i,
            "type": core_type,
            "freq_mhz": freq_mhz,
            "usage": cpu_usage_cache[i] if i < len(cpu_usage_cache) else 0.0
        })
    return cores

def get_thermal_info():
    """读取所有温区传感器数据"""
    thermals = []
    base_dir = "/sys/class/thermal"
    if os.path.exists(base_dir):
        for entry in sorted(os.listdir(base_dir)):
            if entry.startswith("thermal_zone"):
                t_type = read_file(os.path.join(base_dir, entry, "type"), entry)
                t_temp = read_file(os.path.join(base_dir, entry, "temp"), "0")
                try:
                    celsius = round(int(t_temp) / 1000.0, 1)
                except ValueError:
                    celsius = 0.0
                thermals.append({
                    "zone": entry,
                    "name": t_type,
                    "temp": celsius
                })
    return thermals

def get_mem_info():
    """读取系统内存"""
    mem = {"total_mb": 0, "used_mb": 0, "free_mb": 0, "percent": 0.0}
    content = read_file("/proc/meminfo")
    total, avail = 0, 0
    for line in content.splitlines():
        if line.startswith("MemTotal:"):
            total = int(re.search(r'\d+', line).group())
        elif line.startswith("MemAvailable:"):
            avail = int(re.search(r'\d+', line).group())
    if total > 0:
        used = total - avail
        mem["total_mb"] = round(total / 1024, 1)
        mem["used_mb"] = round(used / 1024, 1)
        mem["free_mb"] = round(avail / 1024, 1)
        mem["percent"] = round((used / total) * 100.0, 1)
    return mem

def get_led_state():
    """获取 LED 状态"""
    trigger = read_file(f"{LED_PATH}/trigger", "")
    brightness = read_file(f"{LED_PATH}/brightness", "0")
    cur_trigger = "unknown"
    match = re.search(r'\[(.*?)\]', trigger)
    if match:
        cur_trigger = match.group(1)
    return {
        "trigger": cur_trigger,
        "brightness": brightness,
        "is_on": brightness != "0"
    }

def set_led_mode(mode):
    """设置 LED 模式"""
    if mode == "on":
        write_file(f"{LED_PATH}/trigger", "none")
        write_file(f"{LED_PATH}/brightness", "1")
    elif mode == "off":
        write_file(f"{LED_PATH}/trigger", "none")
        write_file(f"{LED_PATH}/brightness", "0")
    elif mode == "heartbeat":
        write_file(f"{LED_PATH}/trigger", "heartbeat")
    return get_led_state()

def get_system_uptime():
    """获取运行时间与负载"""
    uptime_raw = read_file("/proc/uptime", "0 0").split()[0]
    try:
        secs = int(float(uptime_raw))
        hours, rem = divmod(secs, 3600)
        minutes, seconds = divmod(rem, 60)
        formatted_uptime = f"{hours}小时 {minutes}分 {seconds}秒"
    except Exception:
        formatted_uptime = "未知"
    
    load_avg = [round(x, 2) for x in os.getloadavg()] if hasattr(os, "getloadavg") else [0, 0, 0]
    return {
        "uptime": formatted_uptime,
        "loadavg": load_avg,
        "total_cpu_usage": total_usage_cache
    }

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RK3588 极客硬件监控仪表盘</title>
    <style>
        :root {
            --bg-color: #0d1117;
            --card-bg: rgba(22, 27, 34, 0.85);
            --border-color: rgba(56, 139, 253, 0.2);
            --accent-cyan: #58a6ff;
            --accent-green: #3fb950;
            --accent-purple: #bc8cff;
            --accent-orange: #d29922;
            --accent-red: #f85149;
            --text-main: #f0f6fc;
            --text-sub: #8b949e;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body {
            background-color: var(--bg-color);
            color: var(--text-main);
            min-height: 100vh;
            padding: 24px 16px;
            background-image: radial-gradient(circle at 10% 20%, rgba(56, 139, 253, 0.08) 0%, transparent 40%),
                              radial-gradient(circle at 90% 80%, rgba(188, 140, 255, 0.08) 0%, transparent 40%);
        }
        .container { max-width: 1100px; margin: 0 auto; }
        
        /* 顶部 Header */
        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 12px;
            padding-bottom: 20px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
            margin-bottom: 24px;
        }
        .header-title h1 {
            font-size: 24px;
            font-weight: 700;
            letter-spacing: 0.5px;
            background: linear-gradient(90deg, #58a6ff, #bc8cff);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .header-title p { font-size: 13px; color: var(--text-sub); margin-top: 4px; }
        .badge {
            background: rgba(88, 166, 255, 0.15);
            border: 1px solid var(--accent-cyan);
            color: var(--accent-cyan);
            padding: 6px 12px;
            border-radius: 20px;
            font-size: 13px;
            font-weight: 600;
        }

        /* 网格布局 */
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }

        /* 卡片基础样式 */
        .card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 12px;
            padding: 20px;
            backdrop-filter: blur(10px);
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
            transition: border-color 0.3s;
        }
        .card:hover { border-color: rgba(88, 166, 255, 0.4); }
        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 16px;
        }
        .card-title { font-size: 16px; font-weight: 600; color: var(--text-main); }
        .card-value { font-size: 20px; font-weight: 700; color: var(--accent-cyan); }

        /* 核心利用率条 */
        .core-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }
        .core-item {
            background: rgba(255,255,255,0.03);
            border-radius: 8px;
            padding: 10px;
            border: 1px solid rgba(255,255,255,0.05);
        }
        .core-header {
            display: flex;
            justify-content: space-between;
            font-size: 12px;
            margin-bottom: 6px;
        }
        .progress-bar-bg {
            background: rgba(255, 255, 255, 0.1);
            height: 6px;
            border-radius: 3px;
            overflow: hidden;
        }
        .progress-bar {
            height: 100%;
            background: linear-gradient(90deg, #58a6ff, #3fb950);
            width: 0%;
            transition: width 0.4s ease;
        }

        /* 温度标签 */
        .thermal-list {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
            gap: 10px;
        }
        .thermal-box {
            background: rgba(255,255,255,0.03);
            border: 1px solid rgba(255,255,255,0.06);
            border-radius: 8px;
            padding: 12px;
            text-align: center;
        }
        .thermal-name { font-size: 11px; color: var(--text-sub); margin-bottom: 4px; }
        .thermal-temp { font-size: 18px; font-weight: bold; }
        .temp-cool { color: var(--accent-green); }
        .temp-warm { color: var(--accent-orange); }
        .temp-hot { color: var(--accent-red); }

        /* LED 交互控制器 */
        .led-panel {
            display: flex;
            align-items: center;
            justify-content: space-between;
            flex-wrap: wrap;
            gap: 16px;
        }
        .led-indicator-wrapper {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .led-light {
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background-color: #333;
            box-shadow: 0 0 0 rgba(0,0,0,0);
            transition: all 0.3s;
        }
        .led-light.on {
            background-color: #3fb950;
            box-shadow: 0 0 16px #3fb950;
        }
        .led-light.heartbeat {
            animation: pulse 1.2s infinite ease-in-out;
            background-color: #bc8cff;
            box-shadow: 0 0 16px #bc8cff;
        }
        @keyframes pulse {
            0% { transform: scale(0.95); opacity: 0.3; }
            50% { transform: scale(1.15); opacity: 1; filter: drop-shadow(0 0 10px #bc8cff); }
            100% { transform: scale(0.95); opacity: 0.3; }
        }
        .btn-group {
            display: flex;
            gap: 8px;
        }
        .btn {
            background: rgba(255, 255, 255, 0.08);
            border: 1px solid rgba(255, 255, 255, 0.2);
            color: var(--text-main);
            padding: 8px 14px;
            border-radius: 6px;
            font-size: 13px;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn:hover { background: rgba(255, 255, 255, 0.18); border-color: var(--accent-cyan); }

        /* 内存条 */
        .mem-visual {
            margin-top: 10px;
        }
        .mem-progress {
            background: rgba(255,255,255,0.1);
            height: 12px;
            border-radius: 6px;
            overflow: hidden;
            margin-bottom: 8px;
        }
        .mem-fill {
            height: 100%;
            background: linear-gradient(90deg, #58a6ff, #bc8cff);
            width: 0%;
            transition: width 0.5s ease;
        }
        .mem-detail {
            display: flex;
            justify-content: space-between;
            font-size: 12px;
            color: var(--text-sub);
        }

        footer {
            text-align: center;
            font-size: 12px;
            color: var(--text-sub);
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid rgba(255, 255, 255, 0.05);
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="header-title">
                <h1>正点原子 ATK-DLRK3588 极客面板</h1>
                <p>Rockchip RK3588 (8-Core CPU + 6 TOPS NPU) 边缘监控中心</p>
            </div>
            <div class="badge" id="uptime-badge">运行时间: 读取中...</div>
        </header>

        <!-- 硬件控制区: 板载 LED 交互 -->
        <div class="card" style="margin-bottom: 20px; border-color: rgba(188, 140, 255, 0.3);">
            <div class="card-header">
                <span class="card-title">💡 板载物理硬件控制 (Sysfs 'work' LED)</span>
                <span id="led-status-text" style="font-size: 13px; color: var(--text-sub);">模式: 读取中...</span>
            </div>
            <div class="led-panel">
                <div class="led-indicator-wrapper">
                    <div id="led-light" class="led-light"></div>
                    <div>
                        <strong style="font-size: 14px;">板载 User LED (work)</strong>
                        <div style="font-size: 12px; color: var(--text-sub);">硬件引脚映射: /sys/class/leds/work</div>
                    </div>
                </div>
                <div class="btn-group">
                    <button class="btn" onclick="controlLed('on')">🟢 常亮 (ON)</button>
                    <button class="btn" onclick="controlLed('off')">⚫ 熄灭 (OFF)</button>
                    <button class="btn" onclick="controlLed('heartbeat')">💓 心跳脉冲 (Heartbeat)</button>
                </div>
            </div>
        </div>

        <div class="grid">
            <!-- CPU 状态卡片 -->
            <div class="card" style="grid-column: span 2;">
                <div class="card-header">
                    <div>
                        <span class="card-title">⚡ RK3588 处理器核心 (4x A76 大核 + 4x A55 小核)</span>
                        <div style="font-size: 12px; color: var(--text-sub); margin-top: 4px;">总体使用率: <span id="total-cpu-usage" style="color: var(--accent-cyan); font-weight: bold;">0%</span> | 负载: <span id="cpu-loadavg">0.0, 0.0, 0.0</span></div>
                    </div>
                </div>
                <div class="core-grid" id="core-grid">
                    <!-- 8 核心动态生成 -->
                </div>
            </div>

            <!-- 内存状态卡片 -->
            <div class="card">
                <div class="card-header">
                    <span class="card-title">💾 运行内存 (RAM)</span>
                    <span class="card-value" id="mem-percent">0%</span>
                </div>
                <div class="mem-visual">
                    <div class="mem-progress">
                        <div id="mem-fill" class="mem-fill"></div>
                    </div>
                    <div class="mem-detail">
                        <span id="mem-used">已用: 0 MB</span>
                        <span id="mem-total">总量: 0 MB</span>
                    </div>
                </div>
            </div>
        </div>

        <!-- 芯片各温区卡片 -->
        <div class="card">
            <div class="card-header">
                <span class="card-title">🌡️ 芯片各功能单元温度 (Thermal Zones)</span>
                <span style="font-size: 12px; color: var(--text-sub);">正常工作范围: 30°C ~ 75°C</span>
            </div>
            <div class="thermal-list" id="thermal-list">
                <!-- 动态传感器 -->
            </div>
        </div>

        <footer>
            Pair-Programming with AI Assistant &bull; Powered by Python Standard Library &bull; ATK-DLRK3588
        </footer>
    </div>

    <script>
        async function fetchStats() {
            try {
                const res = await fetch('/api/stats');
                const data = await res.json();
                renderStats(data);
            } catch (err) {
                console.error("Fetch stats failed:", err);
            }
        }

        async function controlLed(mode) {
            try {
                const res = await fetch('/api/led?mode=' + mode, { method: 'POST' });
                const result = await res.json();
                renderLed(result);
            } catch (err) {
                alert("控制 LED 失败: " + err);
            }
        }

        function renderLed(led) {
            const light = document.getElementById('led-light');
            const statusText = document.getElementById('led-status-text');
            light.className = 'led-light';
            
            if (led.trigger === 'heartbeat') {
                light.classList.add('heartbeat');
                statusText.innerText = '当前模式: 💓 动态心跳脉冲模式';
            } else if (led.is_on) {
                light.classList.add('on');
                statusText.innerText = '当前模式: 🟢 手动常亮状态';
            } else {
                statusText.innerText = '当前模式: ⚫ 手动熄灭状态';
            }
        }

        function renderStats(data) {
            // 系统信息
            document.getElementById('uptime-badge').innerText = '运行时间: ' + data.system.uptime;
            document.getElementById('total-cpu-usage').innerText = data.system.total_cpu_usage + '%';
            document.getElementById('cpu-loadavg').innerText = data.system.loadavg.join(', ');

            // 内存
            document.getElementById('mem-percent').innerText = data.memory.percent + '%';
            document.getElementById('mem-fill').style.width = data.memory.percent + '%';
            document.getElementById('mem-used').innerText = '已用: ' + data.memory.used_mb + ' MB';
            document.getElementById('mem-total').innerText = '总量: ' + data.memory.total_mb + ' MB';

            // CPU 8核
            const coreGrid = document.getElementById('core-grid');
            coreGrid.innerHTML = '';
            data.cores.forEach(c => {
                const isBig = c.id >= 4;
                const badgeColor = isBig ? '#bc8cff' : '#58a6ff';
                const item = document.createElement('div');
                item.className = 'core-item';
                item.innerHTML = `
                    <div class="core-header">
                        <span><strong style="color:${badgeColor}">Core #${c.id}</strong> (${c.type})</span>
                        <span>${c.freq_mhz} MHz | <strong>${c.usage}%</strong></span>
                    </div>
                    <div class="progress-bar-bg">
                        <div class="progress-bar" style="width: ${c.usage}%; background: ${isBig ? 'linear-gradient(90deg, #58a6ff, #bc8cff)' : 'linear-gradient(90deg, #58a6ff, #3fb950)'};"></div>
                    </div>
                `;
                coreGrid.appendChild(item);
            });

            // 温区
            const thermalList = document.getElementById('thermal-list');
            thermalList.innerHTML = '';
            data.thermals.forEach(t => {
                let colorClass = 'temp-cool';
                if (t.temp > 65) colorClass = 'temp-hot';
                else if (t.temp > 50) colorClass = 'temp-warm';

                const box = document.createElement('div');
                box.className = 'thermal-box';
                box.innerHTML = `
                    <div class="thermal-name">${t.name || t.zone}</div>
                    <div class="thermal-temp ${colorClass}">${t.temp} °C</div>
                `;
                thermalList.appendChild(box);
            });

            // LED
            renderLed(data.led);
        }

        // 定时轮询 (1.5秒刷新一次)
        fetchStats();
        setInterval(fetchStats, 1500);
    </script>
</body>
</html>
"""

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/" or parsed.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode("utf-8"))
        elif parsed.path == "/api/stats":
            self.send_response(200)
            self.send_header("Content-type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            data = {
                "system": get_system_uptime(),
                "cores": get_cpu_info(),
                "thermals": get_thermal_info(),
                "memory": get_mem_info(),
                "led": get_led_state()
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/led":
            params = parse_qs(parsed.query)
            mode = params.get("mode", [""])[0]
            new_state = set_led_mode(mode)
            self.send_response(200)
            self.send_header("Content-type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(new_state).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

def main():
    t = threading.Thread(target=cpu_monitor_loop, daemon=True)
    t.start()

    server_address = ("0.0.0.0", PORT)
    with socketserver.TCPServer(server_address, DashboardHandler) as httpd:
        print("=" * 60)
        print("  🚀 RK3588 极客硬件监控仪表盘已成功启动！")
        print(f"  📡 本地端口: {PORT}")
        print("  💡 板载 LED 控制路径: /sys/class/leds/work")
        print("  💡 在 Mac 终端运行: adb forward tcp:8080 tcp:8080")
        print("  🌐 然后在 Mac 浏览器打开: http://localhost:8080")
        print("=" * 60)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n正在关闭监控服务...")

if __name__ == "__main__":
    main()
