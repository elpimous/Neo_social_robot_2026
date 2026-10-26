import rclpy
import time
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from qbo_interface.web_qbo import QboWebServer, run_web_server
from qbo_msgs.msg import Nose
import smach
import subprocess
import threading
import signal


class InitSystem(smach.State):
    def __init__(self, supervisor):
        smach.State.__init__(self, outcomes=['ok', 'failed'], input_keys=['nodes_expected'])
        self.supervisor = supervisor
        self.supervisor.current_state = "INIT SYSTEM"

    def execute(self, userdata):
        self.supervisor.get_logger().info("[InitSystem] Checking required nodes...")

        # Attendre un peu le remplissage du graphe
        time.sleep(3.0)

        # Obtenir tous les nœuds visibles dans ROS
        node_list = self.supervisor.get_node_names()
        self.supervisor.get_logger().info(f"[InitSystem] Detected nodes: {node_list}")

        missing = [n for n in userdata.nodes_expected if n not in node_list]
        if missing:
            self.supervisor.get_logger().error(f"Missing nodes: {missing}")
            return 'failed'

        self.supervisor.get_logger().info("All required nodes are active.")
        return 'ok'


class DiagnosticRouter(smach.State):
    def __init__(self, supervisor):
        smach.State.__init__(self, outcomes=['exit'], output_keys=['warnAction', 'errorAction'])
        self.supervisor = supervisor
        self.supervisor.current_state = "DIAGNOSTIC ROUTER"
        self._warn_dict = {}
        self._error_dict = {}
        self._running = True
        self._last_led_state = None  # Pour éviter les publications inutiles

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self._shutdown_handler)

        self.sub = self.supervisor.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._diagnostic_callback,
            10
        )

    def _shutdown_handler(self, signum, frame):
        self.supervisor.get_logger().info("[DiagnosticRouter] Caught Ctrl+C, shutting down cleanly...")
        self._running = False
        self.supervisor.set_led_color(0)  # Éteindre ici seulement

    def _update_led_state(self):
        if self._error_dict:
            desired_color = 1  # Rouge
        elif self._warn_dict:
            desired_color = 5  # Orange
        else:
            desired_color = 0  # Off

        if desired_color != self._last_led_state:
            self.supervisor.get_logger().info(f"[LED] Desired color: {desired_color}, Last color: {self._last_led_state}")
            self.supervisor.set_led_color(desired_color)
            self._last_led_state = desired_color

    def _diagnostic_callback(self, msg):
        updated = False

        for status in msg.status:
            level = status.level
            if isinstance(level, bytes):
                level = int.from_bytes(level, 'little')

            key = f"{status.hardware_id}/{status.name}"

            if level == 1:
                if key not in self._warn_dict:
                    self._warn_dict[key] = (status.message,)
                    updated = True

            elif level == 2:
                if key not in self._error_dict:
                    self._error_dict[key] = (status.message,)
                    updated = True

            elif level == 0:
                if key in self._warn_dict:
                    del self._warn_dict[key]
                    updated = True
                if key in self._error_dict:
                    del self._error_dict[key]
                    updated = True

        # self.supervisor.get_logger().info(f"[Diag] WARNs: {list(self._warn_dict.keys())}")
        # self.supervisor.get_logger().info(f"[Diag] ERRORs: {list(self._error_dict.keys())}")
        # self.supervisor.get_logger().info(f"[LED] Update triggered - warn: {len(self._warn_dict)}, error: {len(self._error_dict)}")

        if updated:
            self._update_led_state()


    def execute(self, userdata):
        self.supervisor.get_logger().info("[DiagnosticRouter] Monitoring diagnostics...")
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.supervisor, timeout_sec=0.1)
        finally:
            userdata.warnAction = self._warn_dict
            userdata.errorAction = self._error_dict

        return 'exit'

class QboSupervisor(Node):
    def __init__(self):
        super().__init__('qbo_supervisor')
        self.nose_pub = self.create_publisher(Nose, '/cmd_nose', 10)
        self.current_state = 'UNKNOWN'
        self.warn_dict = {}
        self.error_dict = {}
        # Pour l'affichage graphique
        self.state_list = []
        self.transition_list = []

    def set_led_color(self, color):
        msg = Nose()
        msg.color = color
        self.nose_pub.publish(msg)

    def get_warn_dict(self):
        return self.warn_dict

    def get_error_dict(self):
        return self.error_dict

    def define_state_machine_structure(self, state_list, transition_list):
        self.state_list = state_list
        self.transition_list = transition_list

    def get_smach_graph(self):
        return {
            "states": self.state_list,
            "transitions": self.transition_list,
            "active": self.current_state
        }

# Utilisation dans ta machine principale :
def main():
    rclpy.init()
    node = QboSupervisor()

    # Lancer le serveur web avec le même node
    web_server = QboWebServer(node)
    web_thread = threading.Thread(target=run_web_server, args=(web_server,), daemon=True)
    web_thread.start()

    # Thread ROS 2
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sm = smach.StateMachine(outcomes=['end', 'failed', 'exit'])
    sm.userdata.nodes_expected = [
        'OrinA608Diag_node', 'analyzers', 'base_ctrl',
        'battery_ctrl', 'imu_ctrl', 'lcd_ctrl',
        'mouth_ctrl', 'nose_ctrl', 'qbo_arduqbo', 'qbo_dynamixel'
    ]
    sm.userdata.warnAction = {}
    sm.userdata.errorAction = {}

    with sm:
        smach.StateMachine.add('INIT_SYSTEM', InitSystem(node),
                               transitions={'ok': 'DIAGNOSTIC_ROUTER', 'failed': 'failed'})

        smach.StateMachine.add('DIAGNOSTIC_ROUTER', DiagnosticRouter(node),
                               transitions={'exit': 'end'})

    try:
        node.define_state_machine_structure(
            state_list=["INIT_SYSTEM", "DIAGNOSTIC_ROUTER", "SHUTDOWN"],
            transition_list=[
                {"from": "INIT_SYSTEM", "to": "DIAGNOSTIC_ROUTER", "label": "ok"},
                {"from": "INIT_SYSTEM", "to": "SHUTDOWN", "label": "failed"},
                {"from": "DIAGNOSTIC_ROUTER", "to": "SHUTDOWN", "label": "exit"}
            ]
        )
        outcome = sm.execute()
        node.get_logger().info(f"State machine ended with outcome: {outcome}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
