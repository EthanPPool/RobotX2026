"""Whitelist-only mission process supervision. Motion authority stays in the bridge."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

PROFILES = {
    'two_gate': {'label': 'Two-gate mission', 'node': 'task1_gate_mission'},
    'uav_route': {'label': 'UAV-mapped USV route', 'node': 'survey_route_mission'},
}


def launch_command(profile, route_config=''):
    if profile == 'two_gate':
        return ['ros2', 'launch', 'boat_mission', 'mission.launch.py', 'enabled:=false']
    if profile == 'uav_route':
        if not route_config or not Path(route_config).is_file():
            raise ValueError('Configure route_config with a calibrated route YAML before loading this mission')
        return ['ros2', 'launch', 'boat_route_navigation', 'boat_route_navigation.launch.py',
                'config:=' + str(Path(route_config).resolve())]
    raise ValueError('Unknown mission profile')


class MissionProcess:
    def __init__(self, route_config='', log_dir=None, popen=subprocess.Popen, signal_group=os.killpg):
        self.profile = 'two_gate'
        self.route_config = route_config
        self.log_dir = Path(log_dir or Path.home() / 'robotx_logs')
        self.process = None
        self.log_file = None
        self.log_path = None
        self.fault = ''
        self.lock = threading.RLock()
        self.popen, self.signal_group = popen, signal_group

    def status(self):
        with self.lock:
            if self.process is not None and self.process.poll() is not None:
                self.fault = f'Mission process exited ({self.process.returncode}); load a mission to retry'
            return {'profile': self.profile, 'state': 'fault' if self.fault else
                    ('active' if self.process is not None else 'inactive'),
                    'running': self.process is not None and self.process.poll() is None,
                    'error': self.fault, 'log_file': self.log_path,
                    'profiles': [{'id': k, 'label': v['label']} for k, v in PROFILES.items()]}

    def select(self, profile):
        with self.lock:
            # Validate before disturbing the current process.
            launch_command(profile, self.route_config)
            if self.process is not None:
                raise RuntimeError('Stop the current mission before selecting another')
            self.profile = profile
            self.fault = ''

    def start(self):
        with self.lock:
            if self.process is not None or self.fault:
                raise RuntimeError('Mission already running or faulted; stop and load before retrying')
            command = launch_command(self.profile, self.route_config)
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.log_path = str(self.log_dir / f'mission_process_{self.profile}_{time.time_ns()}.log')
            self.log_file = open(self.log_path, 'ab', buffering=0)
            try:
                self.process = self.popen(command, stdout=self.log_file, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            except Exception:
                self.log_file.close()
                self.log_file = None
                raise

    def stop(self):
        with self.lock:
            if self.process is not None:
                # Signal the complete ros2 launch group, including child nodes.
                try:
                    self.signal_group(self.process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
                # Also reap lingering children if the launch parent exited first.
                try:
                    self.signal_group(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=2)
                self.process = None
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None
            self.fault = ''
