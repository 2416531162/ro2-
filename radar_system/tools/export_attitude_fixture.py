"""导出坡度开关与永久记忆 IMU 清零遥测，供 App 协议回归使用。"""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import app_link
import export_app_fixtures


def sample():
    wire=export_app_fixtures.fixture_messages()['tel_route_sample.json']
    tilt=app_link._line_tilt(dict(height_m=.53,enabled=True,active=True,reason=None,
        imu_zero_supported=True,zeroed=True,zero_persistent=True,zero_save_error=None,zero_roll_deg=2.,zero_pitch_deg=-1.,
        roll_deg=0.,pitch_deg=0.,forward_m=0.,left_m=0.,blocked=None,zero_blocked=None))
    wire['line'].update(phase='IDLE',tilt=tilt,
        attitude=dict(roll_deg=0.,pitch_deg=0.,yaw_deg=12.6,source='hda536t'),
        last_result=dict(id='zero1',cmd='imu_zero',ok=True,message='横滚、俯仰已清零并保存，重启后保留'))
    return wire

if __name__=='__main__':
    Path(sys.argv[1]).write_text(json.dumps(sample(),ensure_ascii=False,indent=2)+'\n')
