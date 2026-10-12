"""Frozen scenario sets; not a calibrated model of the field."""
import itertools
import simulate as simulation

def benchmark():
    terrains=[dict(name='varying_lateral',slip=.003,step_slip=True),dict(name='rough',slip=.005,turn_disturbance=.012,gyro_vibration=.015),
              dict(name='rtk_wave_3m',slip=0.,noise_amp=.008,noise_period=3.),dict(name='rtk_wave_6m',slip=0.,noise_amp=.008,noise_period=6.),
              dict(name='rtk_loss',slip=.003,dropout=True),dict(name='rtk_jump',slip=.003,jump=True)]
    return [dict(terrain,delay=delay,gain=gain,imu_bias=sign*.5,initial_cm=sign*1.,seed=7 if sign>0 else 17)
            for terrain,delay,gain,sign in itertools.product(terrains,[.25,.44,.65],[.8,1.2],[-1,1])]

def holdout():
    out=[]
    for delay,gain,seed,terrain in itertools.product([.35,.55],[.9,1.1],[101,202],['rough','correlated_noise','slip_change']):
        case=dict(name=terrain,delay=delay,gain=gain,seed=seed,imu_bias=-.35 if seed==101 else .7,
                  initial_cm=-.8 if seed==101 else 1.2,length=55.)
        if terrain=='rough':case.update(slip=.004,slip_period=12,turn_disturbance=.01,gyro_vibration=.012)
        if terrain=='correlated_noise':case.update(slip=0,noise_amp=.007,noise_period=4.5 if seed==101 else 9.)
        if terrain=='slip_change':case.update(slip=.0025,slip_period=25,step_slip=True)
        out.append(case)
    return out
