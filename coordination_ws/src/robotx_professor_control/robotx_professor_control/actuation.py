"""Measured per-thruster force curves and verified FCU manual mixer inversion.

No cubic law is assumed. Optional RPM, torque, PWM and measured motor-voltage
columns describe calibration data; only normalized motor command controls the FCU.
"""
from dataclasses import dataclass
import json
import math
from .law import finite


@dataclass(frozen=True)
class Curve:
    rows: tuple

    def __post_init__(self):
        if len(self.rows)<3:raise ValueError('CALIBRATION_NEEDS_REVERSE_NEUTRAL_FORWARD')
        keys=('force_n','command')
        for row in self.rows:
            if not all(k in row for k in keys):raise ValueError('CALIBRATION_COLUMNS_MISSING')
            finite(*(row[k] for k in row))
            if not -1<=row['command']<=1:raise ValueError('COMMAND_RANGE')
        for a,b in zip(self.rows,self.rows[1:]):
            if not a['force_n']<b['force_n'] or not a['command']<b['command']:
                raise ValueError('CALIBRATION_MUST_BE_STRICTLY_MONOTONE')
        if not any(r['force_n']==0 and r['command']==0 for r in self.rows):raise ValueError('CALIBRATION_NEUTRAL_REQUIRED')
        if self.rows[0]['force_n']>=0 or self.rows[-1]['force_n']<=0:raise ValueError('CALIBRATION_BOTH_DIRECTIONS_REQUIRED')
        optional=set(self.rows[0])-set(keys)
        if any(set(r)-set(keys)!=optional for r in self.rows):raise ValueError('INCONSISTENT_OPTIONAL_COLUMNS')

    def interpolate(self,value,key='force_n'):
        finite(value)
        if not self.rows[0][key]<=value<=self.rows[-1][key]:raise ValueError('CALIBRATION_RANGE_EXCEEDED')
        for a,b in zip(self.rows,self.rows[1:]):
            if a[key]<=value<=b[key]:
                u=(value-a[key])/(b[key]-a[key])
                return {k:a[k]+u*(b[k]-a[k]) for k in a}
        raise ValueError('CALIBRATION_RANGE_EXCEEDED')


class Calibration:
    def __init__(self,data):
        if data.get('schema_version')!=1 or data.get('calibrated') is not True:
            raise ValueError('CALIBRATION_NOT_CONFIRMED')
        if data.get('manual_mix_verified') is not True:raise ValueError('MANUAL_MIX_NOT_VERIFIED')
        self.right,self.left=Curve(tuple(data['right'])),Curve(tuple(data['left']))
        self.signs=tuple(data['model_force_to_forward_sign']) # right Ft3, left Ft4
        if len(self.signs)!=2 or any(s not in (-1,1) for s in self.signs):raise ValueError('FORCE_AXIS_SIGNS_REQUIRED')
        self.voltage_min,self.voltage_max=data['battery_voltage_range_v']
        finite(self.voltage_min,self.voltage_max)
        if not 0<self.voltage_min<self.voltage_max:raise ValueError('BATTERY_CALIBRATION_RANGE')
        self.matrix=tuple(data['manual_to_motor_matrix']) # right=a*T+b*S, left=c*T+d*S
        if len(self.matrix)!=4:raise ValueError('MIXER_MATRIX_REQUIRED')
        finite(*self.matrix)
        a,b,c,d=self.matrix;self.det=a*d-b*c
        if abs(self.det)<1e-6:raise ValueError('SINGULAR_MIXER')

    @classmethod
    def load(cls,path):
        with open(path,encoding='utf-8') as f:return cls(json.load(f))

    def map(self,right,left,voltage):
        finite(right,left,voltage)
        if not self.voltage_min<=voltage<=self.voltage_max:raise ValueError('BATTERY_OUTSIDE_CALIBRATION')
        r,l=self.right.interpolate(right*self.signs[0]),self.left.interpolate(left*self.signs[1])
        a,b,c,d=self.matrix
        throttle=(d*r['command']-b*l['command'])/self.det
        steering=(-c*r['command']+a*l['command'])/self.det
        # Do not silently clip/rescale differential mixing and change forces.
        if max(abs(throttle),abs(steering))>1+1e-9:raise ValueError('MANUAL_MIX_RANGE_EXCEEDED')
        return dict(throttle=throttle,steering=steering,right=r,left=l)
