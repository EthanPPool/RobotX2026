import math
import random
import unittest
from robotx_professor_control.law import Axis,rates,forward_step,ForceIntegrator
from robotx_professor_control.tracking import Binding,Motion,PoseHistory,MotionHistory
from robotx_professor_control.protocol import Preview,interpolate
from dataclasses import replace


def expanded_notebook(old,x,q,xd,qd,h,yaw_sign):
    # Independent transcription of Mathematica Out[1124] and Out[1127].
    ex=[a-b for a,b in zip(x,xd)];eq=[a-b for a,b in zip(q,qd)]
    surge=((719.8053330781249-2238.596695925194*h+3409.834796259714*h*h)*ex[0]
           +(-1439.6106661562499+2238.596695925194*h)*ex[1]+719.8053330781249*ex[2])
    yaw=((17.040946843853824-29.856690797946126*h+9.519074990274795*h*h)*eq[0]
         +(-34.08189368770765+29.856690797946126*h)*eq[1]+17.040946843853824*eq[2])
    return (h*old+surge+yaw_sign*yaw)/h


class LawTests(unittest.TestCase):
    def test_exact_discrete_kernel_matches_both_expanded_notebook_outputs(self):
        rng=random.Random(42)
        for h in (.02,.05,.1,.2):
            for _ in range(20):
                x,q,xd,qd=[[rng.uniform(-.2,.2) for _ in range(3)] for _ in range(4)]
                result=forward_step(1.5,-.7,x,q,xd,qd,h)
                self.assertAlmostEqual(result[0],expanded_notebook(1.5,x,q,xd,qd,h,1),places=9)
                self.assertAlmostEqual(result[1],expanded_notebook(-.7,x,q,xd,qd,h,-1),places=9)

    def test_continuous_coefficients_signs_and_axis_order(self):
        z=Axis(0,0,0)
        for field,kx,kq in zip(range(3),(3409.834796259714,2238.596695925194,719.8053330781249),(9.519074990274795,29.856690797946126,17.040946843853824)):
            terms=[0.,0.,0.];terms[field]=1.;a=Axis(*terms)
            self.assertEqual(rates(a,z,z,z),(kx,kx))
            self.assertEqual(rates(z,a,z,z),(kq,-kq))
            self.assertEqual(rates(z,z,a,z),(-kx,-kx))

    def test_zero_error_retains_nominal_force(self):
        a=Axis(.1,.2,.3)
        self.assertEqual(rates(a,a,a,a),(0.,0.))
        self.assertEqual(forward_step(2.,3.,[1,2,3],[4,5,6],[1,2,3],[4,5,6],.1),(2.,3.))

    def test_limits_have_no_hidden_windup_and_reset_to_trim(self):
        f=ForceIntegrator(1,2,.1,-.1)
        for _ in range(30):f.step((100,100),.1)
        self.assertEqual(f.forces,(1.,1.))
        self.assertAlmostEqual(f.step((-100,-100),.1)[0][0],.8)
        f.reset();self.assertEqual(f.forces,(.1,-.1))
        with self.assertRaises(ValueError):f.step((1,1),1.)

    def test_invalid_exact_preview_is_rejected(self):
        for h in (0,-1,float('nan')):
            with self.assertRaises(ValueError):forward_step(0,0,[0,0,0],[0,0,0],[0,0,0],[0,0,0],h)


class BindingTests(unittest.TestCase):
    def test_binding_rotates_with_nominal_path_not_global_east(self):
        b=Binding();r=Motion(10,0,0,0,.2,0,0)
        m=Motion(10,.1,.2,.02,.25,.01,.01,.01,.02,.03)
        a,d=b.axes(m,r)
        angle=1.2;c,s=math.cos(angle),math.sin(angle)
        def rotate(v):return replace(v,x=c*v.x-s*v.y,y=s*v.x+c*v.y,yaw=v.yaw+angle,
            vx=c*v.vx-s*v.vy,vy=s*v.vx+c*v.vy,ax=c*v.ax-s*v.ay,ay=s*v.ax+c*v.ay)
        aa,dd=b.axes(rotate(m),rotate(r))
        for u,v in zip(a,aa):
            for key in ('value','velocity','acceleration'):self.assertAlmostEqual(getattr(u,key),getattr(v,key))
        self.assertAlmostEqual(d['cross_error'],dd['cross_error'])
        self.assertLess(d['heading_correction'],0.)

    def test_yaw_wrap_and_local_domain(self):
        b=Binding();r=Motion(10,0,0,math.pi-.01,0,0,0)
        axes,_=b.axes(Motion(10,0,0,-math.pi+.01,0,0,0),r)
        self.assertAlmostEqual(axes[1].value,.02)
        with self.assertRaisesRegex(ValueError,'LOCAL_MODEL'):b.axes(replace(r,x=5.),r)

    def test_pose_only_derivatives_are_causal_and_use_real_capture_intervals(self):
        h=PoseHistory(tau=0)
        for t in (10.,10.08,10.2):
            u=t-10;out=h.update(Motion(t,u*u,3*u,math.pi-.05+u,0,0,0))
        self.assertAlmostEqual(out.vx,.4);self.assertAlmostEqual(out.ax,2.)
        self.assertAlmostEqual(out.vy,3.);self.assertAlmostEqual(out.yaw_rate,1.)
        self.assertAlmostEqual(out.yaw_acceleration,0.)
        with self.assertRaisesRegex(ValueError,'REWIND'):h.update(out)
        with self.assertRaisesRegex(ValueError,'GAP'):h.update(replace(out,stamp=11.))
        self.assertIsNone(h.update(replace(out,stamp=11.1)))

    def test_measured_twist_acceleration_history(self):
        h=MotionHistory(tau=0)
        self.assertIsNone(h.update(Motion(10,0,0,0,1,0,.2)))
        m=h.update(Motion(10.1,0,0,0,1.1,0,.3))
        self.assertAlmostEqual(m.ax,1.);self.assertAlmostEqual(m.yaw_acceleration,1.)


class ProtocolTests(unittest.TestCase):
    def preview(self):
        ss=tuple(Motion(10+i*.1,(i*.1)**2,0,0,2*i*.1,0,0,2,0,0) for i in range(3))
        return Preview(10,'map','datum','run','TRANSIT',True,.1,ss)

    def test_three_references_and_c2_interpolation(self):
        p=self.preview();p.validate(10.02,'map','datum')
        r=p.at_measurement(10.15)
        self.assertAlmostEqual(r.x,.15**2);self.assertAlmostEqual(r.vx,.3);self.assertAlmostEqual(r.ax,2.)
        for stamp in (9.8,10.3):
            with self.assertRaises(ValueError):p.at_measurement(stamp)

    def test_invalid_stale_future_wrong_datum_and_hold_are_not_motion_permissions(self):
        p=self.preview()
        for bad in (replace(p,valid=False),replace(p,phase='HOLD'),replace(p,stamp=11),replace(p,stamp=9),replace(p,frame='other'),replace(p,reference_id='other'),replace(p,samples=p.samples[:2])):
            with self.subTest(preview=bad):
                with self.assertRaises(ValueError):bad.validate(10.01,'map','datum')
