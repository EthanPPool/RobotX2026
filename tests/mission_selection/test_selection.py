"""Exercise real mission switching methods without ROS or connected hardware."""
import ast
import importlib.util
import json
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as Box
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / 'boat_ws/src/boat_dashboard_bridge/boat_dashboard_bridge/bridge.py'
DASHBOARD = ROOT / 'ground_station_ws/src/robotx_dashboard/robotx_dashboard/dashboard.py'
spec = importlib.util.spec_from_file_location('mission_process', BRIDGE.with_name('mission_process.py'))
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


def real_class():
    tree = ast.parse(BRIDGE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BoatDashboardBridge')
    context = dict(Node=object, json=json, time=time, PROFILES=module.PROFILES,
                   launch_command=module.launch_command, Trigger=Box(Request=lambda: 'trigger'))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(BRIDGE), 'exec'), context)
    return context['BoatDashboardBridge']


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.child = Mock(pid=1000, returncode=7); self.child.poll.return_value = None
        self.spawn = Mock(return_value=self.child); self.signals = Mock()
        self.manager = module.MissionProcess(log_dir=self.temp.name, popen=self.spawn, signal_group=self.signals)
        self.addCleanup(self.manager.stop)

    def test_gate_launch_starts_disabled_and_without_shell(self):
        self.manager.start()
        args, kwargs = self.spawn.call_args
        self.assertEqual(args[0], ['ros2','launch','boat_mission','mission.launch.py','enabled:=false'])
        self.assertTrue(kwargs['start_new_session']); self.assertNotIn('shell', kwargs)

    def test_unknown_profile_cannot_execute_commands(self):
        for p in ['../../foo','two_gate; reboot',None,'uuv_follow']:
            with self.assertRaises(ValueError): self.manager.select(p)
        self.spawn.assert_not_called()

    def test_route_requires_config_and_uses_separate_launch(self):
        with self.assertRaises(ValueError): self.manager.select('uav_route')
        config = Path(self.temp.name) / 'route.yaml'; config.write_text('datum_configured: false')
        self.manager.route_config = str(config); self.manager.select('uav_route'); self.manager.start()
        self.assertEqual(self.spawn.call_args.args[0][2:4], ['boat_route_navigation','boat_route_navigation.launch.py'])

    def test_cannot_start_or_change_profile_while_running(self):
        self.manager.start()
        with self.assertRaises(RuntimeError):self.manager.start()
        with self.assertRaises(RuntimeError):self.manager.select('two_gate')
        self.assertEqual(self.spawn.call_count,1)

    def test_child_death_latches_fault_until_load(self):
        self.manager.start();self.child.poll.return_value=7
        self.assertEqual(self.manager.status()['state'],'fault')
        with self.assertRaises(RuntimeError):self.manager.start()
        self.manager.stop();self.manager.select('two_gate');self.manager.start()
        self.assertEqual(self.spawn.call_count,2)

    def test_stop_reaps_entire_group_even_if_parent_already_exited(self):
        self.manager.start();self.child.poll.return_value=7;self.manager.stop()
        self.assertEqual([x.args for x in self.signals.call_args_list],[(1000,signal.SIGINT),(1000,signal.SIGKILL)])
        self.assertIsNone(self.manager.process)

    def test_shutdown_escalates_unresponsive_group(self):
        self.manager.start();self.child.wait.side_effect=[subprocess.TimeoutExpired('ros2',3),None]
        self.manager.stop();self.assertEqual(self.child.wait.call_count,2)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = real_class().__new__(real_class())
        b=self.bridge;b.manage_missions=True;b.lock=threading.RLock();b.action_lock=threading.RLock()
        b.mission_manager=Mock(profile='two_gate', route_config='')
        b.mission_manager.status.return_value={'profile':'two_gate','state':'inactive','running':False,'error':''}
        b.managed_diagnostics={};b.managed_diagnostics_rx=None;b.managed_started=None
        b.telemetry={'bridge_forward':0.,'bridge_yaw':0.};b.managed_bridge_rx=None;b.bridge_last_rx=None
        b.managed_bridge_diagnostics={};b.operator_deadman=False
        b.local_status=Mock(return_value={'online':True,'armed':False})
        b.managed_controller_matches=Mock(return_value=True)
        b.managed_publishers=Mock(return_value=[]);b.execute_stop=Mock(return_value=(True,'stopped'))
        b.execute_fail_safe_stop=Mock();b.gate_enable_client=Mock();b.route_enable_client=Mock()
        b.reset_client=Mock();b.route_reset_client=Mock();b.wait_future=Mock(return_value=True)

    def test_loading_requires_fresh_disarmed_vehicle(self):
        for status in [{'online':False,'armed':False},{'online':True,'armed':True}]:
            self.bridge.local_status.return_value=status
            ok,_=self.bridge.managed_safe_stop(True);self.assertFalse(ok)
        self.bridge.execute_stop.assert_not_called()

    def test_failed_stop_prevents_replacing_mission(self):
        b=self.bridge;b.execute_stop.return_value=(False,'service rejected')
        self.assertFalse(b.execute_managed_process('select','two_gate')[0])
        b.mission_manager.stop.assert_not_called();b.mission_manager.select.assert_not_called()

    def test_stop_ack_requires_actual_neutral_after_request(self):
        b=self.bridge
        def stopped():
            t=time.monotonic()+.001;b.managed_bridge_rx=t;b.bridge_last_rx=t
            b.managed_bridge_diagnostics={'bridge_autonomy_enabled':False,'software_estop':True}
            return True,'stopped'
        b.execute_stop.side_effect=stopped
        self.assertTrue(b.managed_safe_stop(True)[0])
        b.telemetry['bridge_forward']=.1
        clock=Mock();clock.monotonic.side_effect=[10.,10.,12.]
        with patch.object(real_class().managed_safe_stop.__globals__['time'],'monotonic',clock.monotonic):
            b.execute_stop.side_effect=None;b.execute_stop.return_value=(True,'stopped')
            self.assertFalse(b.managed_safe_stop(True)[0])

    def test_unknown_profile_preserves_current_mission(self):
        self.assertFalse(self.bridge.execute_managed_process('select','injected')[0])
        self.bridge.execute_stop.assert_not_called()

    def test_load_stops_then_selects_without_start_or_arm(self):
        b=self.bridge;b.managed_safe_stop=Mock(return_value=(True,'safe'))
        self.assertTrue(b.execute_managed_process('select','two_gate')[0])
        b.mission_manager.stop.assert_called_once();b.mission_manager.select.assert_called_once_with('two_gate')
        b.mission_manager.start.assert_not_called()

    def test_conflicting_publisher_blocks_start(self):
        b=self.bridge;b.managed_safe_stop=Mock(return_value=(True,'safe'));b.managed_publishers.return_value=[Box(node_name='other')]
        self.assertFalse(b.execute_managed_process('start')[0]);b.mission_manager.start.assert_not_called()

    def test_exact_publisher_identity_required(self):
        b=self.bridge
        for names in [[Box(node_name='other',node_namespace='/')],
                      [Box(node_name='task1_gate_mission',node_namespace='/other')],
                      [Box(node_name='task1_gate_mission',node_namespace='/')]*2]:
            b.managed_publishers.return_value=names;self.assertFalse(b.managed_publisher_matches())
        b.managed_publishers.return_value=[Box(node_name='task1_gate_mission',node_namespace='/')]
        self.assertTrue(b.managed_publisher_matches())

    def test_route_readiness_does_not_require_gate_visibility(self):
        b=self.bridge;b.mission_manager.status.return_value={'profile':'uav_route','running':True,'error':''}
        b.managed_publisher_matches=Mock(return_value=True);b.managed_diagnostics_rx=time.monotonic()
        for reason,ready in [('ROUTE_TRACKING',True),('UAV_MAP_STALE',False),('ROUTE_DISABLED',False)]:
            b.managed_diagnostics={'route_state':reason};self.assertEqual(b.managed_mission_status()['ready'],ready)
        b.managed_diagnostics_rx-=1;self.assertFalse(b.managed_mission_status()['ready'])

    def test_gate_readiness_requires_selected_identity(self):
        b=self.bridge;b.mission_manager.status.return_value={'profile':'two_gate','running':True,'error':''}
        b.managed_publisher_matches=Mock(return_value=True);b.managed_diagnostics_rx=time.monotonic()
        b.managed_diagnostics={'mission_reason':'TRACKING_GATE','mission_phase':'TRACK_GATE','gate_identity_locked':True}
        self.assertTrue(b.managed_mission_status()['ready'])
        b.managed_diagnostics['mission_reason']='TRACK_GATE_MEASUREMENT_STALE'
        self.assertFalse(b.managed_mission_status()['ready'])

    def test_watchdog_revokes_authority_on_process_or_publisher_loss(self):
        b=self.bridge;b.managed_started=time.monotonic()-10
        b.managed_mission_status=Mock(return_value={'running':False,'diagnostics_age_sec':None})
        b.monitor_managed_mission();b.execute_fail_safe_stop.assert_called_once();b.mission_manager.stop.assert_called_once()
        self.assertIsNone(b.managed_started);self.assertIn('fault',b.mission_manager.fault)

    def test_route_reset_targets_route_service(self):
        b=self.bridge;b.mission_manager.profile='uav_route'
        b.route_reset_client.call_async.return_value.result.return_value=Box(success=True,message='route reset')
        self.assertTrue(b.call_reset_service()[0]);b.route_reset_client.call_async.assert_called_once()
        b.reset_client.call_async.assert_not_called()

    def test_start_handshake_leaves_mission_disabled(self):
        b=self.bridge;b.managed_safe_stop=Mock(return_value=(True,'safe'))
        b.managed_publisher_matches=Mock(return_value=True)
        b.mission_manager.status.return_value={'running':True}
        self.assertTrue(b.execute_managed_process('start')[0]);b.mission_manager.start.assert_called_once()
        b.gate_enable_client.call_async.assert_not_called();b.route_enable_client.call_async.assert_not_called()

    def test_enable_activates_selected_mission_before_authority(self):
        for profile in ['two_gate','uav_route']:
            b=self.bridge;b.mission_manager.profile=profile
            b.mission_manager.status.return_value={'running':True}
            b.local_status.return_value={'online':True,'armed':False,'mode':'GUIDED','bridge_alive':True}
            b.estop_client=Box(name='estop');b.autonomy_client=Box(name='authority')
            b.call_mode_service=Mock(return_value=(True,'guided'));b.managed_publisher_matches=Mock(return_value=True)
            calls=[]
            def call(client,value):
                calls.append((client,value));b.bridge_last_rx=time.monotonic();return True,'ok'
            b.call_bool_service=call
            self.assertTrue(b.execute_enable()[0])
            expected=b.route_enable_client if profile=='uav_route' else b.gate_enable_client
            self.assertLess(calls.index((expected,True)),calls.index((b.autonomy_client,True)))

    def test_route_arm_requires_route_ready_but_not_gate(self):
        b=self.bridge;b.mission_manager.profile='uav_route';b.control_state='AUTONOMY READY / DISARMED'
        b.local_status.return_value={'online':True,'armed':False,'mode':'GUIDED','gate_fresh':False,'bridge_command_active':True}
        b.managed_mission_status=Mock(return_value={'ready':False});b.call_arm_service=Mock(return_value=(True,'armed'))
        self.assertFalse(b.execute_arm()[0]);b.call_arm_service.assert_not_called()
        b.managed_mission_status.return_value={'ready':True};self.assertTrue(b.execute_arm()[0])
        b.call_arm_service.assert_called_once_with(True)

    def test_duplicate_publisher_watchdog_stops_running_mission(self):
        b=self.bridge;b.managed_started=time.monotonic()-10
        b.managed_mission_status=Mock(return_value={'running':True,'diagnostics_age_sec':.1})
        b.managed_publisher_matches=Mock(return_value=False);b.monitor_managed_mission()
        b.execute_fail_safe_stop.assert_called_once();b.mission_manager.stop.assert_called_once()

    def test_duplicate_controller_blocks_start(self):
        b=self.bridge;b.managed_safe_stop=Mock(return_value=(True,'safe'))
        b.managed_controller_matches.return_value=False
        self.assertFalse(b.execute_managed_process('start')[0]);b.mission_manager.start.assert_not_called()

    def test_arming_during_teardown_blocks_selection(self):
        b=self.bridge;b.managed_safe_stop=Mock(return_value=(True,'safe'))
        b.local_status.return_value={'online':True,'armed':True}
        self.assertFalse(b.execute_managed_process('select','two_gate')[0])
        b.mission_manager.select.assert_not_called()


class ApiTests(unittest.TestCase):
    def setUp(self):
        tree=ast.parse(DASHBOARD.read_text());func=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='api_vehicle_mission_process')
        func.decorator_list=[];self.client=Mock();self.client.command.return_value={'success':True}
        self.request=Box(method='POST',get_json=lambda **kw:{'action':'select','profile':'uav_route'})
        context={'dashboard_node':Box(vehicle_manager=Box(get_client=lambda _:self.client)),
                 'request':self.request,'jsonify':lambda x:x}
        exec(compile(ast.Module(body=[func],type_ignores=[]),str(DASHBOARD),'exec'),context)
        self.api=context[func.name]

    def test_valid_selection_reaches_jetson_with_profile(self):
        self.assertTrue(self.api('boat')['success'])
        self.client.command.assert_called_once_with('mission_process',{'action':'select','profile':'uav_route'})

    def test_invalid_profiles_vehicles_and_nonobjects_rejected(self):
        for data,vehicle in [({'action':'select','profile':'shell'},'boat'),
                             ({'action':'select','profile':'two_gate'},'uav'),([1],'boat')]:
            self.request.get_json=lambda **kw:data
            self.assertEqual(self.api(vehicle)[1],400)
        self.client.command.assert_not_called()

    def test_get_status_keeps_legacy_contract(self):
        self.request.method='GET';self.api('boat')
        self.client.command.assert_called_once_with('mission_process',{'action':'status'})


if __name__ == '__main__':unittest.main()
