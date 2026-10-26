import subprocess
import rclpy
from typing import Optional
import time
from diagnostic_msgs.msg import DiagnosticArray


def is_controller_running(node) -> bool:
    """Vérifie via /diagnostics si le contrôleur Qbo est déjà actif."""
    result = {'ok': False}

    def cb(msg: DiagnosticArray):
        for status in msg.status:
            if (
                status.name == "/Qbo/System Status/Controller Status"
                and status.hardware_id == "qbo_arduqbo System"
                and "initialized" in status.message.lower()
                and status.level == 0
            ):
                result['ok'] = True

    sub = node.create_subscription(DiagnosticArray, '/diagnostics', cb, 10)

    # On attend jusqu'à 3s pour recevoir un message
    timeout = time.time() + 3.0
    while not result['ok'] and time.time() < timeout and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub)
    return result['ok']


def launch_ros_launch(node, package: str, launch_file: str) -> Optional[subprocess.Popen]:
    """Lance ros2 launch si le système n’est pas déjà actif, retourne le process ou None."""
    if is_controller_running(node):
        node.get_logger().info("Contrôleurs déjà actifs, pas besoin de relancer.")
        return None

    try:
        proc = subprocess.Popen(['ros2', 'launch', package, launch_file])
        node.get_logger().info(f"Lancement de {package}/{launch_file} réussi.")
        return proc
    except Exception as e:
        node.get_logger().error(f"Erreur lors du lancement de {package}: {e}")
        return None


def stop_process(proc: Optional[subprocess.Popen]):
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
