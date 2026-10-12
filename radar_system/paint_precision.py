"""Accuracy budget for low-speed forward straight painting; never a ground-truth claim."""
import math

MAX_OFFSET_M = .015
START_OFFSET_M = .010
STOP_OFFSET_M = .012  # Reserve 3 mm before the requested maximum, not a sensor-accuracy guarantee.

def enabled(config, differential=True):
    return bool(differential and getattr(config,'grass_lateral_enabled',False)
                and 0 < getattr(config,'cruise_mps',math.inf) <= .5+1e-9)

def active(mission,section,tracker):
    if not enabled(getattr(tracker,'cfg',None),getattr(mission,'differential',False)):
        return False
    if getattr(section,'direction',0)<=0 or getattr(section,'field_role',None)=='transit':
        return False
    try:return bool(section.path._piece(tracker.st.s).straight)
    except (AttributeError,ValueError,TypeError):return False
