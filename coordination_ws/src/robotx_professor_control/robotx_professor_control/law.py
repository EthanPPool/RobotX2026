"""Professor's two-stern-thruster equations, transcribed without gain/sign edits.

x/q and Ft retain notebook meanings. The notebook does not establish their units
or force-axis directions. The ROS deployment contract must confirm those separately.
"""
from dataclasses import dataclass
import math

X = (3409.834796259714, 2238.596695925194, 719.8053330781249)
Q = (9.519074990274795, 29.856690797946126, 17.040946843853824)


def finite(*values):
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('NONFINITE')


@dataclass(frozen=True)
class Axis:
    value: float
    velocity: float
    acceleration: float

    def validate(self):finite(self.value,self.velocity,self.acceleration)


def rates(x, q, desired_x, desired_q):
    """Return (Ft3_dot RIGHT, Ft4_dot LEFT), using actual minus desired."""
    for axis in (x,q,desired_x,desired_q):axis.validate()
    def term(gains,a,b):
        return sum(k*(getattr(a,f)-getattr(b,f)) for k,f in zip(gains,('value','velocity','acceleration')))
    surge,yaw=term(X,x,desired_x),term(Q,q,desired_q)
    return surge+yaw,surge-yaw


def forward_axis(values,dt):
    """The notebook's exact forward differences at t, t+dt, t+2dt."""
    if len(values)!=3:raise ValueError('THREE_SAMPLES_REQUIRED')
    finite(dt,*values)
    if dt<=0:raise ValueError('INVALID_DT')
    a,b,c=values
    return Axis(a,(b-a)/dt,(a-2*b+c)/dt**2)


def forward_step(right,left,x,q,desired_x,desired_q,dt):
    """Exact discrete notebook law; future ACTUAL samples must be supplied.

    Use for offline replay or a separately validated state preview. This function
    does not fabricate future actual state from the desired reference.
    """
    finite(right,left)
    dr,dl=rates(*(forward_axis(v,dt) for v in (x,q,desired_x,desired_q)))
    return right+dt*dr,left+dt*dl


class ForceIntegrator:
    def __init__(self,limit=10.,rate_limit=20.,nominal_right=0.,nominal_left=0.):
        finite(limit,rate_limit,nominal_right,nominal_left)
        if min(limit,rate_limit)<=0 or max(abs(nominal_right),abs(nominal_left))>limit:
            raise ValueError('INVALID_FORCE_LIMITS')
        self.limit,self.rate_limit=limit,rate_limit
        self.nominal=(nominal_right,nominal_left)
        self.reset()

    def reset(self):self.forces=self.nominal

    def step(self,rate,dt):
        finite(dt,*rate)
        if not 0<dt<=.2:raise ValueError('CONTROL_TIME_GAP')
        raw=tuple(old+dt*r for old,r in zip(self.forces,rate))
        limited=tuple(max(-self.limit,min(self.limit,old+dt*max(-self.rate_limit,min(self.rate_limit,r))))
                      for old,r in zip(self.forces,rate))
        # Store the actual limited state; hidden integrator windup is not retained.
        self.forces=limited
        return limited,any(abs(a-b)>1e-9 for a,b in zip(raw,limited))
