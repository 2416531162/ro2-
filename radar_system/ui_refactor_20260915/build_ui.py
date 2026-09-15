from pathlib import Path
import ast
import textwrap

P = Path(__file__).resolve().parent
source = (P/'BASELINE.py').read_text()
tree = ast.parse(source)
window = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BoardRadarMainWindow')
snippets = (P/'ui_methods.py').read_text()
methods = {n.name: ast.get_source_segment(snippets, n) for n in ast.parse(snippets).body if isinstance(n, ast.FunctionDef)}
lines = source.splitlines(True)
edits = []
for method in window.body:
    if isinstance(method, ast.FunctionDef) and method.name in methods:
        edits.append((method.lineno-1, method.end_lineno, textwrap.indent(methods.pop(method.name), '    ')+'\n'))
for start, end, replacement in sorted(edits, reverse=True):
    lines[start:end] = [replacement]
source = ''.join(lines)
new_methods = '\n\n'.join(textwrap.indent(s, '    ') for s in methods.values())
source = source.replace('\ndef main():', '\n'+new_methods+'\n\ndef main():')

theme = '''
LIGHT_STYLE = """
QWidget#perceptionRoot, QDialog { background:#F3F6F8; color:#21394D; }
QLabel { background:transparent; border:none; color:#21394D; }
QFrame#panel { background:#FFFFFF; border:1px solid #DCE5EC; border-radius:14px; }
QPushButton { background:#F0F4F7; color:#425B70; border:1px solid #D8E2EA;
              border-radius:8px; padding:9px 15px; min-height:24px; font-size:15px; }
QPushButton:hover { background:#E5EEF3; border-color:#A8BAC7; }
QPushButton:checked { background:#DFF2F0; color:#086E74; border:1px solid #77B5B3; font-weight:600; }
QPushButton:focus { border:2px solid #087F83; }
QPushButton:disabled { color:#8697A6; background:#F5F7F9; }
QComboBox, QLineEdit { background:#FFFFFF; color:#304C63; border:1px solid #CBD9E4;
                      border-radius:7px; padding:8px 12px; min-height:24px; font-size:14px; }
QComboBox:focus, QLineEdit:focus { border:2px solid #087F83; }
QComboBox QAbstractItemView { background:white; color:#304C63; selection-background-color:#DFF2F0; selection-color:#086E74; }
QToolTip { background:#20394B; color:white; border:none; padding:8px; }
"""

def chip_style(state):
    colors = {'good': ('#E6F4EC', '#247451'), 'warn': ('#FFF3DA', '#8B621A'),
              'muted': ('#E9EFF3', '#5D7183')}
    background, foreground = colors[state]
    return f'background:{background};color:{foreground};font-size:14px;border-radius:8px;padding:9px 12px;'

'''
source = source.replace('\nclass BoardRadarMainWindow', '\n'+theme+'class BoardRadarMainWindow')
# Recolor the polar grid only; projection, ranges and metric thresholds are untouched.
start = source.index('class RadarCanvas')
end = source.index('class CorsConfigDialog')
canvas = source[start:end]
for old, new in {'#0b1220':'#182D3D','#385167':'#7892A7','#25364a':'#355368',
 '#aebfd0':'#B8CAD8','#354b62':'#668397','#1d2c3e':'#2D495D',
 '#d0deea':'#CEE0EB','#ff747f':'#FF7A86','#ffc66a':'#F6CA78',
 '#61d8e8':'#67DFD8','#e3eef9':'#F0F6F8'}.items(): canvas=canvas.replace(old,new)
canvas=canvas.replace('QColor(color), 3.2', 'QColor(color), 3.8')
source=source[:start]+canvas+source[end:]
# Keep the existing CORS form and handlers, restyle its inputs and buttons.
start=source.index('class CorsConfigDialog')
end=source.index('LIGHT_STYLE =')
dialog=source[start:end]
for old,new in {'#181f29':'#F8FAFC','#00f2fe':'#087F83','#ecf2f8':'#243B50',
 '#111923':'#FFFFFF','#303c4b':'#CAD8E2','#94a3b8':'#60768A','#10b981':'#247451',
 '#ff8c91':'#B9404A','#f6c879':'#8B621A','#242f3d':'#EDF2F6','#3d4f63':'#CAD8E2',
 '#4a2f39':'#FFE5E8','#38bdf8':'#146EB4','#38242c':'#FFF0F1',
 '#9f525a':'#E6ABB2','#69dec4':'#087F83','#283440':'#DCE5EC',
 '#a1afc0':'#607488','#059669':'#087F83'}.items():dialog=dialog.replace(old,new)
dialog=dialog.replace('setFixedSize(560, 600)','setFixedSize(620, 660)')
source=source[:start]+dialog+source[end:]
# Neutral, lower-luminance surroundings; high-contrast sensor plane stays separate.
for old,new in {'#F3F6F8':'#CBD6DF','#FFFFFF':'#DFE7ED','#F8FAFC':'#DFE7ED',
 '#F0F4F7':'#D5E0E8','#EDF2F5':'#C7D4DE','#DCE5EC':'#B5C4D0',
 '#E2E9EF':'#B5C4D0','#E9EFF3':'#C3D1DC','#F5F7F9':'#D6DFE6'}.items():
    source=source.replace(old,new)
source=source.replace("    def on_rgb_frame(self, qimage):\n", "    def on_rgb_frame(self, qimage):\n        self._camera_received = time.monotonic()\n")
source=source.replace('🌈 彩色模式: 实时实景','彩色实景 · 实时画面')
source=source.replace('🎯 AI锁定: [{label}] {d:.2f}m (X:{x:+.2f} Z:{z:.2f}m)', '最近目标 · {label}  {d:.2f} m   |   横向 {x:+.2f} m · 深度 {z:.2f} m')
source=source.replace('AI 空间检测: 未发现目标', '当前画面未发现可识别目标')
source=source.replace('🖥️ 全屏显示', '全屏显示').replace('🖥️ 退出全屏', '窗口模式')
source=source.replace("'#ffc66a', '#3d301d'", "'#8B621A', '#FFF3DA'")
source=source.replace("'#ff8790', '#41232d'", "'#B53E4B', '#FFF0F1'")
source=source.replace("'#73dcc3', '#173a36'", "'#247451', '#EAF6EF'")
source=source.replace("padding:9px; background:#3d301d; color:#ffc66a; border-radius:8px;", "padding:8px 18px;background:#FFF3DA;color:#8B621A;border-radius:10px;font-size:16px;")
source=source.replace("padding:9px; border-radius:8px; background:{bg}; color:{color}; font-weight:bold;", "padding:8px 18px;border-radius:10px;background:{bg};color:{color};font-size:16px;font-weight:600;")
ast.parse(source)
(P/'MODIFIED_FILE.py').write_text(source)
print('BUILT', len(source), 'characters')
