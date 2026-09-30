"""作业自动记录：什么时候开段、收段、结束作业，点开始前后留多少，空间满了删什么。不依赖 ROS（假的 bag writer）。"""
import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'radar_system'))
import job_recorder as jr


class FakeWriter:
    instances = []

    def __init__(self, uri):
        os.makedirs(uri)
        self.uri, self.rows, self.closed = uri, [], False
        FakeWriter.instances.append(self)

    def write(self, topic, type_name, data, t_ns):
        self.rows.append((topic, data, t_ns))

    def close(self):
        self.closed = True
        Path(self.uri, 'bag.mcap').write_bytes(b'x' * len(self.rows))


class Clock:
    t = 0.0


@pytest.fixture
def env(tmp_path):
    FakeWriter.instances = []
    clk = Clock()
    extra = tmp_path / 'fusion.json'
    extra.write_text('{"a": 1}')
    logs = []

    def make():
        return jr.JobRecorder(root=str(tmp_path / 'jobs'), writer=FakeWriter, files=lambda: [str(extra), '/nonexistent/x.json'],
                              clock=lambda: clk.t, wall=lambda: 1790000000.0 + clk.t, log=logs.append)
    return make, clk, tmp_path / 'jobs'


def job(id_='aaaaaaaa-1111', state='RUNNING', label='路线作业'):
    return dict(id=id_, state=state, label=label, kind='route', plan='大', reason='')


def status(phase='IDLE', task='route', j=None, e=0.0):
    return dict(phase=phase, task=task, message=phase, tracker=dict(state='RUNNING' if phase == 'RUNNING' else 'IDLE', e=e, s=1.0),
                operations=dict(job=j), diag=dict(jumps=0, position_resets=0))


def feed(rec, clk, topic, t, decoded=None, data=b'm'):
    clk.t = t
    rec.on_message(topic, jr.TOPICS.get(topic, 'std_msgs/msg/String'), data, int(t * 1e9), decoded)


def job_json(root):
    dirs = sorted(p for p in root.iterdir() if p.is_dir())
    return [(d, json.loads((d / 'job.json').read_text())) for d in dirs]


def test_start_keeps_ten_seconds_before_and_five_after_then_done_closes_the_job(env):
    make, clk, root = env
    rec = make()
    for t in range(0, 20):
        feed(rec, clk, '/imu', t, data=b'imu%d' % t)
        feed(rec, clk, '/line/status', t + .5, status())
    assert not FakeWriter.instances                              # 空闲时不记
    feed(rec, clk, '/line/status', 20, status('RUNNING', j=job(), e=.01))
    w = FakeWriter.instances[0]
    imu = [d for topic, d, _ in w.rows if topic == '/imu']
    assert imu[0] == b'imu10' and imu[-1] == b'imu19'            # 点开始前 10 s
    for t in range(21, 31):
        feed(rec, clk, '/line/status', t, status('RUNNING', j=job(), e=.02 if t != 25 else -.05))
        feed(rec, clk, '/gnss/status', t + .1, dict(rtk='FIXED' if t < 29 else 'FLOAT', heading_ok=True, sats=30))
    feed(rec, clk, '/line/status', 31, status('IDLE', j=job(state='DONE')))
    feed(rec, clk, '/imu', 34, data=b'after')
    assert not w.closed and w.rows[-1][1] == b'after'           # 停下后还记 5 s
    feed(rec, clk, '/line/status', 36.5, status('IDLE', j=job(state='DONE')))
    assert w.closed
    [(d, info)] = job_json(root)
    assert d.name.endswith('_路线作业_aaaaaaaa') and info['id'] == 'aaaaaaaa-1111'
    assert info['result']['state'] == 'DONE' and 'plan' not in info['job']
    seg = info['segments'][0]
    assert seg['closed_by'] == 'idle' and seg['summary']['track']['e_max_m'] == .05
    assert seg['summary']['rtk']['fix_share'] == {'FIXED': .8, 'FLOAT': .2}
    assert (d / 'context' / 'seg01' / 'fusion.json').is_file() and (d / 'context' / 'seg01' / 'release.txt').is_file()
    assert rec.status()['recording'] is False and rec.dir is None


def test_pause_then_resume_after_recorder_restart_appends_to_the_same_job(env):
    make, clk, root = env
    rec = make()
    feed(rec, clk, '/line/status', 1, status('RUNNING', j=job()))
    feed(rec, clk, '/line/status', 2, status('IDLE', j=job(state='PAUSED')))
    feed(rec, clk, '/line/status', 8, status('IDLE', j=job(state='PAUSED')))
    [(d, info)] = job_json(root)
    assert len(info['segments']) == 1 and info['result'] is None    # 暂停：段结束，作业没结束
    rec.shutdown()
    rec2 = make()                                                   # 记录进程重启（部署）后继续
    feed(rec2, clk, '/line/status', 100, status('SELECTING', j=job()))
    feed(rec2, clk, '/line/status', 101, status('RUNNING', j=job()))
    feed(rec2, clk, '/line/status', 102, status('IDLE', j=job(state='DONE')))
    feed(rec2, clk, '/line/status', 108, status('IDLE', j=job(state='DONE')))
    [(d2, info)] = job_json(root)
    assert d2 == d and [s['name'] for s in info['segments']] == ['seg01', 'seg02']
    assert info['result']['state'] == 'DONE' and (d / 'seg02').is_dir()


def test_new_job_ends_a_held_one_and_a_line_task_is_its_own_job(env):
    make, clk, root = env
    rec = make()
    feed(rec, clk, '/line/status', 1, status('RUNNING', j=job('aaaaaaaa-1')))
    feed(rec, clk, '/line/status', 2, status('IDLE', j=job('aaaaaaaa-1', 'PAUSED')))
    feed(rec, clk, '/line/status', 8, status('IDLE', j=job('aaaaaaaa-1', 'PAUSED')))
    # 跑完的旧记录还留在状态里时开 A—B 直线：不能算进旧作业
    feed(rec, clk, '/line/status', 10, status('RUNNING', task='line', j=job('aaaaaaaa-1', 'PAUSED')))
    feed(rec, clk, '/line/status', 12, status('IDLE', task='line', j=job('aaaaaaaa-1', 'PAUSED')))
    feed(rec, clk, '/line/status', 18, status('IDLE', task='line', j=job('aaaaaaaa-1', 'PAUSED')))
    jobs = job_json(root)
    assert len(jobs) == 2
    held, line = jobs[0][1], jobs[1][1]
    assert held['result']['state'] == 'PAUSED'                      # 被新任务顶掉：按最后状态收尾
    assert line['id'].startswith('task-line-') and line['result'] is not None and len(line['segments']) == 1


def straight(state='idle', run=None, reason=None, err=None, max_err=None):
    return dict(imu_ok=True, state=state, run=run, reason=reason, target_deg=30.0, error_deg=err, max_error_deg=max_err,
                within_pct=95.0, elapsed_s=7.2)


def test_imu_straight_run_is_recorded_as_its_own_job(env):
    """手动页 IMU 直行：没有路线作业也要记，前 10 s、后 5 s，job.json 里有这次直行的结果（停车原因、最大偏差）。"""
    make, clk, root = env
    rec = make()
    feed(rec, clk, '/imu/data', 0.5)
    for t in range(1, 20):
        feed(rec, clk, '/line/status', t, status('IDLE', task='line'))     # 路线节点一直在发空闲
        if t < 5:
            feed(rec, clk, '/straight/status', t + .5, straight('stopped', ['app-1', 9], 'user'))   # 上一次的，不算
        elif t < 12:
            feed(rec, clk, '/straight/status', t + .5, straight('running', ['app-1', 10], err=.3, max_err=4.2))
        else:
            feed(rec, clk, '/straight/status', t + .5, straight('stopped', ['app-1', 10], 'deviation', 5.3, 5.3))
    (d, info), = job_json(root)
    assert d.name.split('_')[2] == 'IMU直行' and info['id'].startswith('task-IMU直行-')
    seg, = info['segments']
    assert seg['closed_by'] == 'idle' and info['result'] is not None
    assert FakeWriter.instances[0].rows[0][0] == '/imu/data'            # 开始前的数据也在
    runs = seg['summary']['straight']
    assert [(r['run'], r['state'], r['reason'], r['max_error_deg']) for r in runs] == [(['app-1', 10], 'stopped', 'deviation', 5.3)]


def test_status_lost_closes_the_segment_but_not_the_job(env):
    make, clk, root = env
    rec = make()
    feed(rec, clk, '/line/status', 1, status('RUNNING', j=job()))
    clk.t = 5
    rec.tick()
    assert rec.status()['recording']
    clk.t = 12
    rec.tick()
    [(d, info)] = job_json(root)
    assert info['segments'][0]['closed_by'] == 'line_status_lost' and info['result'] is None


def test_a_write_error_never_raises_and_is_reported(env):
    make, clk, root = env

    class Broken(FakeWriter):
        def write(self, *a):
            raise OSError('disk gone')

    rec = make()
    rec.writer_cls = Broken
    feed(rec, clk, '/imu', 0)
    feed(rec, clk, '/line/status', 1, status('RUNNING', j=job()))
    feed(rec, clk, '/imu', 2)
    assert not rec.status()['recording'] and 'disk gone' in rec.status()['error']
    [(d, info)] = job_json(root)
    assert info['segments'][0]['closed_by'] == 'write_error'


def test_prune_removes_oldest_jobs_but_never_the_current_one(tmp_path):
    for name in ('20260901_a', '20260902_b', '20260903_c'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'bag').write_bytes(b'x' * 600_000)
    removed = jr.prune(str(tmp_path), keep=str(tmp_path / '20260901_a'), max_total_mb=1.3, min_free_mb=0, log=lambda m: None)
    assert removed == ['20260902_b']
    assert sorted(p.name for p in tmp_path.iterdir()) == ['20260901_a', '20260903_c']
