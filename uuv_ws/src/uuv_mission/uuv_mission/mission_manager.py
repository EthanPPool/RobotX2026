#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class UuvMissionManager(Node):
    """Safe mission shell. It manages enable/state but deliberately emits no motion."""

    def __init__(self):
        super().__init__('uuv_mission_manager')
        self.enabled = bool(self.declare_parameter('enabled', False).value)
        self.authorized = False
        self.state_pub = self.create_publisher(String, '/uuv/mission/state', 10)
        self.create_subscription(Bool, '/uuv/safety/authorized', self._auth_cb, 10)
        self.create_service(SetBool, '/uuv/control/set_enabled', self._set_enabled)
        self.create_service(Trigger, '/uuv/control/reset_mission', self._reset)
        self.timer = self.create_timer(0.2, self._publish)

    def _auth_cb(self, msg):
        self.authorized = bool(msg.data)

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = 'UUV autonomy enabled' if self.enabled else 'UUV autonomy disabled'
        return response

    def _reset(self, request, response):
        self.enabled = False
        response.success = True
        response.message = 'UUV mission reset to IDLE'
        return response

    def _publish(self):
        if not self.enabled:
            state = 'IDLE'
        elif not self.authorized:
            state = 'READY_WAITING_SAFETY'
        else:
            state = 'RUNNING'
        data = {
            'autonomy_enabled': self.enabled,
            'autonomy_state': state,
            'mission_state': state,
            'mission_healthy': True,
            'mission_process_running': True,
            'mission_process_state': 'active',
        }
        msg = String(); msg.data = json.dumps(data, separators=(',', ':')); self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = UuvMissionManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
