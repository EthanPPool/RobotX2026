"""Explicit, provisional binding of map ENU references to the notebook's two axes.

The exact law stays in law.py. This guidance/binding is an engineering addition,
not a claim that the notebook contains a global x/y path-following controller.
"""
from dataclasses import dataclass,replace
import math
from .law import Axis,finite


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


@dataclass(frozen=True)
class Motion:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    ax: float=0.
    ay: float=0.
    yaw_acceleration: float=0.

    def validate(self):
        finite(*(getattr(self,k) for k in self.__dataclass_fields__))
        if self.stamp<=0:raise ValueError('INVALID_STAMP')


class MotionHistory:
    """Causal acceleration from measured velocities; no future actual state."""
    def __init__(self,tau=.25,max_gap=.2):
        finite(tau,max_gap)
        if tau<0 or max_gap<=0:raise ValueError('INVALID_HISTORY_LIMITS')
        self.tau,self.max_gap=tau,max_gap;self.reset()

    def reset(self):self.previous=None;self.ready=False

    def update(self,m):
        m.validate();old=self.previous
        if old is None:self.previous=m;return None
        dt=m.stamp-old.stamp
        if dt<=0:raise ValueError('ODOMETRY_TIME_REWIND')
        if dt>self.max_gap:self.reset();self.previous=m;raise ValueError('ODOMETRY_GAP')
        alpha=dt/(self.tau+dt)
        fields={name:getattr(old,name)+alpha*((getattr(m,v)-getattr(old,v))/dt-getattr(old,name))
                for name,v in (('ax','vx'),('ay','vy'),('yaw_acceleration','yaw_rate'))}
        self.previous=replace(m,**fields);self.ready=True
        return self.previous


@dataclass(frozen=True)
class Binding:
    lookahead: float=5.
    max_along_error: float=1.
    max_cross_error: float=1.
    max_yaw_error: float=.35

    def __post_init__(self):
        finite(self.lookahead,self.max_along_error,self.max_cross_error,self.max_yaw_error)
        if min(self.lookahead,self.max_along_error,self.max_cross_error,self.max_yaw_error)<=0:
            raise ValueError('INVALID_BINDING_LIMITS')

    def axes(self,m,r):
        """Freeze the nominal tangent for this instantaneous local linearization.

        x is translation projected onto that fixed tangent, not global east.
        Cross-track LOS correction changes desired yaw before the professor law.
        """
        m.validate();r.validate()
        c,s=math.cos(r.yaw),math.sin(r.yaw)
        ex,ey=m.x-r.x,m.y-r.y
        along,cross=c*ex+s*ey,-s*ex+c*ey
        cross_v=-s*(m.vx-r.vx)+c*(m.vy-r.vy)
        cross_a=-s*(m.ax-r.ax)+c*(m.ay-r.ay)
        length=self.lookahead;den=length*length+cross*cross
        correction=-math.atan2(cross,length)
        correction_rate=-length*cross_v/den
        correction_accel=2*length*cross*cross_v**2/den**2-length*cross_a/den
        q_error=wrap(m.yaw-r.yaw-correction)
        if abs(along)>self.max_along_error or abs(cross)>self.max_cross_error or abs(q_error)>self.max_yaw_error:
            raise ValueError('OUTSIDE_LOCAL_MODEL_DOMAIN')
        actual_x=Axis(along,c*m.vx+s*m.vy,c*m.ax+s*m.ay)
        desired_x=Axis(0.,c*r.vx+s*r.vy,c*r.ax+s*r.ay)
        actual_q=Axis(q_error,m.yaw_rate,m.yaw_acceleration)
        desired_q=Axis(0.,r.yaw_rate+correction_rate,r.yaw_acceleration+correction_accel)
        return (actual_x,actual_q,desired_x,desired_q),dict(along_error=along,cross_error=cross,
            yaw_error=q_error,heading_correction=correction)


class PoseHistory:
    """Quadratic, causal position/yaw derivatives for pose-only shared odometry.

    The shared GPS adapter leaves Odometry.twist unset. Never interpret those
    default zeros as measured surge or yaw rate. This estimator needs three
    distinct capture times and has no future-actual-state requirement.
    """
    def __init__(self,max_gap=.2,tau=.25):
        finite(max_gap,tau)
        if max_gap<=0 or tau<0:raise ValueError('INVALID_HISTORY_LIMITS')
        self.max_gap,self.tau=max_gap,tau;self.reset()

    def reset(self):self.samples=[];self.filtered=None

    def update(self,m):
        m.validate()
        if self.samples:
            old=self.samples[-1];dt=m.stamp-old.stamp
            if dt<=0:raise ValueError('ODOMETRY_TIME_REWIND')
            if dt>self.max_gap:self.reset();self.samples=[m];raise ValueError('ODOMETRY_GAP')
            m=replace(m,yaw=old.yaw+wrap(m.yaw-old.yaw))
        self.samples=(self.samples+[m])[-3:]
        if len(self.samples)<3:return None
        ts=[v.stamp-m.stamp for v in self.samples]
        def derivatives(field):
            v=a=0.
            for i in range(3):
                j,k=[z for z in range(3) if z!=i]
                den=(ts[i]-ts[j])*(ts[i]-ts[k])
                value=getattr(self.samples[i],field)
                v+=value*(-ts[j]-ts[k])/den;a+=value*2/den
            return v,a
        vx,ax=derivatives('x');vy,ay=derivatives('y');w,alpha=derivatives('yaw')
        out=replace(m,vx=vx,vy=vy,yaw_rate=w,ax=ax,ay=ay,yaw_acceleration=alpha)
        if self.filtered:
            fraction=(m.stamp-self.filtered.stamp)/(self.tau+m.stamp-self.filtered.stamp)
            out=replace(out,**{f:getattr(self.filtered,f)+fraction*(getattr(out,f)-getattr(self.filtered,f))
                              for f in ('vx','vy','yaw_rate','ax','ay','yaw_acceleration')})
        self.filtered=out
        return out
