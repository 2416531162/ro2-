"""Read the deployment EnvironmentFile without evaluating shell expressions.

Only assignment lines are accepted. Values follow systemd's unquoted, single-
quoted and double-quoted EnvironmentFile rules; invalid lines fail closed.
"""
import os
from pathlib import Path
import re
import shlex
import sys

ASSIGNMENT = re.compile(r'([A-Za-z_][A-Za-z_0-9]*)=(.*)\Z')
DEFAULT_FILE = Path('/etc/rk3588/runtime.env')


def parse_file(path):
    values = {}
    lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    i = 0
    while i < len(lines):
        number = i + 1
        line = lines[i].rstrip('\r\n')
        i += 1
        if not line.strip() or line.lstrip().startswith(('#', ';')):
            continue
        match = ASSIGNMENT.fullmatch(line.lstrip())
        if not match:
            raise ValueError(f'{path}:{number}: expected NAME=value')
        key, raw = match.groups()
        raw = raw.lstrip(' \t')
        quote = raw[0] if raw.startswith(("'", '"')) else None
        pos = 1 if quote else 0
        value = ''
        while True:
            if pos == len(raw):
                if quote:
                    if i == len(lines):
                        raise ValueError(f'{path}:{number}: unterminated quoted value')
                    value += '\n'
                    raw = lines[i].rstrip('\r\n')
                    pos = 0
                    i += 1
                    continue
                break
            char = raw[pos]
            if char == quote:
                if raw[pos + 1:].strip(' \t'):
                    raise ValueError(f'{path}:{number}: text after quoted value')
                break
            if char == '\\' and quote != "'":
                if pos + 1 == len(raw):
                    if i == len(lines):
                        raise ValueError(f'{path}:{number}: dangling backslash')
                    raw = lines[i].rstrip('\r\n')
                    pos = 0
                    i += 1
                    continue
                next_char = raw[pos + 1]
                if quote != '"' or next_char in ('\\', '"', '$', '`'):
                    value += next_char
                else:
                    value += '\\' + next_char
                pos += 2
                continue
            value += char
            pos += 1
        values[key] = value if quote else value.rstrip(' \t')
    return values


def resolve(environ, project):
    explicit = 'ROBOT_RUNTIME_ENV' in environ
    path = Path(environ['ROBOT_RUNTIME_ENV']) if explicit else DEFAULT_FILE
    if explicit and not environ['ROBOT_RUNTIME_ENV']:
        raise ValueError('ROBOT_RUNTIME_ENV must name a readable file')
    path = path.absolute()
    try:
        file_values = parse_file(path)
    except FileNotFoundError:
        if explicit:
            raise ValueError(f'explicit ROBOT_RUNTIME_ENV does not exist: {path}') from None
        file_values = {}
    except (OSError, UnicodeError) as exc:
        raise ValueError(f'cannot read runtime environment {path}: {exc}') from exc

    values = {**file_values, **environ}
    source = 'explicit' if explicit else ('default' if path.is_file() else 'default (missing)')
    defaults = {
        'RK3588_PYTHON': '/usr/bin/python3',
        'RK3588_ROS_ROOT': '/opt/ros',
        'RK3588_ROBOT_CONFIG': str(project / 'robot_core/robot.json'),
        'RK3588_DEPTH_PATH_CONFIG': '/etc/rk3588/depth_path.json',
        'RK3588_POSE_MODEL': str(project / 'radar_system/models/yolo26s.pt'),
    }
    for key, default in defaults.items():
        values.setdefault(key, default)
        if not values[key]:
            raise ValueError(f'{key} must not be empty')
    if 'ROS_DISTRO' in values and values['ROS_DISTRO'] not in ('humble', 'jazzy'):
        raise ValueError(f"unsupported ROS_DISTRO={values['ROS_DISTRO']!r}; expected humble or jazzy")
    python = Path(values['RK3588_PYTHON'])
    if not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError(f'invalid RK3588_PYTHON (absolute executable required): {python}')
    for key, base in (('RK3588_ROS_ROOT', project),
                      ('RK3588_ROBOT_CONFIG', project),
                      ('RK3588_DEPTH_PATH_CONFIG', project),
                      ('RK3588_POSE_MODEL', project / 'radar_system')):
        values[key] = str((base / values[key]).absolute())
    values['ROBOT_RUNTIME_ENV'] = str(path)
    values['RK3588_RUNTIME_ENV_SOURCE'] = source
    return {key: value for key, value in values.items()
            if key not in environ or key in defaults or key in ('ROBOT_RUNTIME_ENV', 'RK3588_RUNTIME_ENV_SOURCE')}


def main():
    project = Path(__file__).resolve().parent.parent
    try:
        values = resolve(os.environ, project)
        for key, value in values.items():
            if not ASSIGNMENT.match(f'{key}='):
                raise ValueError(f'invalid environment key: {key}')
            if '\x00' in value:
                raise ValueError(f'NUL in {key}')
            print(f'export {key}={shlex.quote(value)}')
    except ValueError as exc:
        print(f'runtime environment: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
