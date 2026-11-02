"""Timestamp/frame/run validation and C2 interpolation of three ROS references."""
from dataclasses import dataclass
import math
from .law import finite
from .tracking import Motion

ACTIVE_PHASES={'SEARCH_ENTRY','OBSERVE','APPROACH_ENTRY','CIRCLE_ENTRY','TRANSIT','CIRCLE_EXIT'}


def fresh(stamp,now,limit):return math.isfinite(stamp) and stamp>0 and 0<=now-stamp<=limit


def interpolate(a,b,t):
    """Quintic Hermite segment through position, velocity and acceleration."""
    h=b.stamp-a.stamp
    if h<=0 or not a.stamp-1e-8<=t<=b.stamp+1e-8:raise ValueError('REFERENCE_TIME_OUTSIDE_WINDOW')
    u=max(0.,min(1.,(t-a.stamp)/h))
    def scalar(p0,v0,a0,p1,v1,a1):
        c0,c1,c2=p0,h*v0,.5*h*h*a0
        d0=p1-c0-c1-c2;d1=h*v1-c1-2*c2;d2=h*h*a1-2*c2
        c3,c4,c5=10*d0-4*d1+.5*d2,-15*d0+7*d1-d2,6*d0-3*d1+.5*d2
        p=c0+c1*u+c2*u*u+c3*u**3+c4*u**4+c5*u**5
        v=(c1+2*c2*u+3*c3*u*u+4*c4*u**3+5*c5*u**4)/h
        acc=(2*c2+6*c3*u+12*c4*u*u+20*c5*u**3)/(h*h)
        return p,v,acc
    x,vx,ax=scalar(a.x,a.vx,a.ax,b.x,b.vx,b.ax)
    y,vy,ay=scalar(a.y,a.vy,a.ay,b.y,b.vy,b.ay)
    q,w,alpha=scalar(a.yaw,a.yaw_rate,a.yaw_acceleration,b.yaw,b.yaw_rate,b.yaw_acceleration)
    out=Motion(t,x,y,q,vx,vy,w,ax,ay,alpha);out.validate();return out


@dataclass(frozen=True)
class Preview:
    stamp: float
    frame: str
    reference_id: str
    run_id: str
    phase: str
    valid: bool
    dt: float
    samples: tuple

    def validate(self,now,frame,reference_id,timeout=.15):
        if self.frame!=frame or self.reference_id!=reference_id:raise ValueError('REFERENCE_FRAME_OR_DATUM_MISMATCH')
        if not self.run_id or not self.valid or self.phase not in ACTIVE_PHASES:raise ValueError('REFERENCE_NOT_AUTHORIZED_FOR_MOTION')
        if not fresh(self.stamp,now,timeout):raise ValueError('REFERENCE_STALE_OR_FUTURE')
        finite(self.dt)
        if not .01<=self.dt<=.5 or len(self.samples)!=3:raise ValueError('THREE_TIMED_REFERENCES_REQUIRED')
        for i,s in enumerate(self.samples):
            s.validate()
            if abs(s.stamp-self.stamp-i*self.dt)>1e-6:raise ValueError('REFERENCE_OFFSETS_INVALID')
        if any(abs(b.yaw-a.yaw)>math.pi for a,b in zip(self.samples,self.samples[1:])):
            raise ValueError('REFERENCE_YAW_MUST_BE_UNWRAPPED')

    def at_measurement(self,stamp,tolerance=.05):
        # A just-published preview can be slightly newer than the latest capture.
        # This bounded skew is explicit; never extrapolate actual vessel state.
        if stamp<self.stamp-tolerance or stamp>self.samples[-1].stamp:
            raise ValueError('POSE_REFERENCE_TIME_MISMATCH')
        t=max(stamp,self.stamp)
        a,b=(self.samples[:2] if t<=self.samples[1].stamp else self.samples[1:])
        return interpolate(a,b,t)
