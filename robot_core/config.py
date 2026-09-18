"""Single calibrated profile, validated before any node opens devices."""
import hashlib
import json
import math
import os
from pathlib import Path

DEFAULT_PATH = Path(__file__).with_name('robot.json')


def load_profile(path=None):
    path = Path(path or os.environ.get('RK3588_ROBOT_CONFIG', DEFAULT_PATH))
    profile = json.loads(path.read_text())
    default = json.loads(DEFAULT_PATH.read_text())
    if set(profile) != set(default) or profile.get('schema_version') != 1:
        raise ValueError('unsupported robot profile schema')
    for section, values in default.items():
        if not isinstance(values, dict):
            continue
        if not isinstance(profile[section], dict) or set(profile[section]) != set(values):
            raise ValueError('invalid profile section: ' + section)
        for key, expected in values.items():
            value = profile[section][key]
            if isinstance(expected, str):
                if not isinstance(value, str) or not value:
                    raise ValueError('invalid frame/topic: ' + key)
            elif type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('invalid numeric profile value: ' + key)
    g, s, c = profile['geometry'], profile['safety'], profile['sensors']
    if min(g.values()) <= 0 or not 0 < g['max_steer_rad'] < .9:
        raise ValueError('invalid robot geometry')
    if not -0.6 < c['camera_pitch_rad'] < .6 or c['camera_x_m'] > g['front_m']:
        raise ValueError('invalid camera mount')
    if min(s.values()) <= 0 or s['command_timeout_s'] > 1 or s['scan_timeout_s'] > 1:
        raise ValueError('invalid safety deadlines')
    if s['min_scan_points'] != int(s['min_scan_points']):
        raise ValueError('min_scan_points must be an integer')
    loc = profile['localization']
    if any(v <= 0 for k, v in loc.items() if not isinstance(v, str)) or loc['max_extrapolation_s'] > loc['timeout_s']:
        raise ValueError('invalid localization deadlines')
    if min(profile['driver'].values()) <= 0 or profile['driver']['max_speed_m_s'] > 2.5:
        raise ValueError('invalid driver limits')
    manual = profile['manual']
    if not 0 < manual['low_mps'] <= manual['med_mps'] <= manual['high_mps'] <= profile['driver']['max_speed_m_s']:
        raise ValueError('invalid manual speed tiers')
    if not 0 < manual['reverse_scale'] <= 1:
        raise ValueError('invalid reverse speed ratio')
    if profile['frames']['odom'] == profile['frames']['base']:
        raise ValueError('odom and base frames must differ')
    return profile


def profile_hash(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest()[:16]


PROFILE = load_profile()
