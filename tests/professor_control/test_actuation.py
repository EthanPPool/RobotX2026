import copy
import unittest
from robotx_professor_control.actuation import Calibration,Curve


def calibration_data():
    return dict(schema_version=1,calibrated=True,manual_mix_verified=True,
                battery_voltage_range_v=[14.,16.],model_force_to_forward_sign=[1,1],
                manual_to_motor_matrix=[1,-1,1,1],
                right=[dict(force_n=-10.,command=-1.,rpm=-100.,shaft_torque_nm=-.2),dict(force_n=0.,command=0.,rpm=0.,shaft_torque_nm=0.),dict(force_n=10.,command=1.,rpm=100.,shaft_torque_nm=.2)],
                left=[dict(force_n=-5.,command=-1.),dict(force_n=0.,command=0.),dict(force_n=5.,command=1.)])


class ActuationTests(unittest.TestCase):
    def test_independent_curves_correct_motor_imbalance_and_invert_verified_mixer(self):
        c=Calibration(calibration_data());v=c.map(5.,5.,15.)
        self.assertEqual(v['right']['command'],.5);self.assertEqual(v['left']['command'],1.)
        self.assertAlmostEqual(v['throttle'],.75);self.assertAlmostEqual(v['steering'],.25)
        self.assertEqual(v['right']['rpm'],50.);self.assertEqual(v['right']['shaft_torque_nm'],.1)
        self.assertNotIn('rpm',v['left'])

    def test_signed_reverse_neutral_and_model_axis_sign(self):
        data=calibration_data();data['model_force_to_forward_sign']=[-1,1]
        c=Calibration(data)
        self.assertEqual(c.map(5.,0.,15.)['right']['command'],-.5)
        self.assertEqual(c.map(0.,0.,15.)['throttle'],0.)

    def test_no_extrapolation_or_voltage_guessing(self):
        c=Calibration(calibration_data())
        for r,l,v in ((11,0,15),(0,6,15),(1,1,13.9),(1,1,float('nan'))):
            with self.assertRaises(ValueError):c.map(r,l,v)

    def test_incomplete_nonmonotone_unverified_or_singular_calibrations_refuse(self):
        for key,value in (('calibrated',False),('manual_mix_verified',False),('model_force_to_forward_sign',[0,1]),('manual_to_motor_matrix',[1,1,1,1]),('battery_voltage_range_v',[0,0]),('right',[{'force_n':0,'command':0}])):
            data=calibration_data();data[key]=value
            with self.assertRaises(ValueError):Calibration(data)
        data=calibration_data();data['left'][1]['command']=1.
        with self.assertRaises(ValueError):Calibration(data)
