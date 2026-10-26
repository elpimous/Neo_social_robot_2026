#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from qbo_msgs.msg import Nose, LCD  # Adapter si message différent
import smach
import subprocess
import threading
from qbo_interface.utils.launch_utils import launch_ros_launch, stop_process


class QboSupervisor(Node):
    def __init__(self):
        super().__init__('qbo_supervisor')
        self.diagnostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostic_callback,
            10
        )
        self.launch_proc = None

        self.nose_pub = self.create_publisher(Nose, '/cmd_nose', 10)
        self.lcd_pub = self.create_publisher(LCD, '/cmd_lcd', 10)

        self.latest_level = 0  # 0=OK, 1=Warn, 2=Error

    def diagnostic_callback(self, msg):
        level = 0
        for status in msg.status:
            value = status.level
            if isinstance(value, bytes):
                value = int.from_bytes(value, byteorder='little')
            level = max(level, value)
        self.latest_level = level

    def set_led_color(self, color):
        msg = Nose()
        msg.color = color
        self.nose_pub.publish(msg)

    def set_lcd_message(self, text):
        msg = LCD()
        msg.text = text[:20]  # Troncature pour affichage LCD
        self.lcd_pub.publish(msg)


# -----------------------------
# SMACH States
# -----------------------------
class InitSystem(smach.State):
    def __init__(self, supervisor: QboSupervisor):
        smach.State.__init__(self, outcomes=['done'])
        self.supervisor = supervisor

    def execute(self, userdata):
        self.supervisor.current_state = 'INIT'
        self.supervisor.get_logger().info("Initialisation système...")
        return 'done'


class LaunchControllers(smach.State):
    def __init__(self, supervisor: QboSupervisor):
        smach.State.__init__(self, outcomes=['launched', 'already_running', 'launch_failed'])
        self.supervisor = supervisor

    def execute(self, userdata):
        self.supervisor.current_state = 'LAUNCH_CONTROLLERS'
        self.supervisor.get_logger().info("Vérification et lancement des contrôleurs Qbo...")

        proc = launch_ros_launch(self.supervisor, 'qbo_arduqbo', 'qbo_full.launch.py')
        self.supervisor.launch_proc = proc

        if proc is None:
            if is_controller_running(self.supervisor):
                return 'already_running'
            else:
                return 'launch_failed'
        return 'launched'



class MonitorDiagnostics(smach.State):
    def __init__(self, supervisor: QboSupervisor):
        smach.State.__init__(self, outcomes=['ok', 'warn', 'error'])
        self.supervisor = supervisor

    def execute(self, userdata):
        self.supervisor.get_logger().info("Surveillance des diagnostics...")

        # boucle de surveillance
        while rclpy.ok():
            level = self.supervisor.latest_level
            if level == 0:
                self.supervisor.set_led_color(4)  # Vert
                # self.supervisor.set_lcd_message("Status OK")
                return 'ok'
            elif level == 1:
                self.supervisor.set_led_color(5)  # Jaune
                # self.supervisor.set_lcd_message("Warning détecté")
                return 'warn'
            elif level == 2:
                self.supervisor.set_led_color(1)  # Rouge
                # self.supervisor.set_lcd_message("Erreur critique")
                return 'error'


def main():
    rclpy.init()
    node = QboSupervisor()

    # Thread pour garder ROS 2 actif pendant SMACH
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Construction de la machine
    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add('INIT', InitSystem(node), transitions={'done': 'START_CONTROLLERS'})
        smach.StateMachine.add(
                                    'START_CONTROLLERS',
                                    LaunchControllers(node),
                                    transitions={
                                        'launched': 'MONITOR',
                                        'already_running': 'MONITOR',
                                        'launch_failed': 'INIT'  # ou une transition d'erreur
                                    }
                                )
        smach.StateMachine.add('MONITOR', MonitorDiagnostics(node), transitions={
            'ok': 'MONITOR',
            'warn': 'MONITOR',
            'error': 'MONITOR'
        })

    # Exécution
    try:
        outcome = sm.execute()
        node.get_logger().info(f"Machine terminée avec l'état : {outcome}")
    finally:
        stop_process(node.launch_proc)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
