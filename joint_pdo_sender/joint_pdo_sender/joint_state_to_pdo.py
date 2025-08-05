# joint_state_to_pdo.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can

class JointStateToPDO(Node):
    def __init__(self):
        super().__init__('joint_state_to_pdo')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # oder /joint_trajectory_controller/state
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        if len(msg.position) < 5:
            self.get_logger().warn("Nicht genügend Gelenkpositionen empfangen.")
            return

        # Skalierung: 1 rad → 1000 (z. B. int32 mit drei Nachkommastellen)
        scale = 1000

        # PDO1: Achse 1 + 2
        self.send_pdo(0x201, [
            int(msg.position[0] * scale),
            #int(msg.position[1] * scale)
        ])
        self.send_pdo(0x204, [
            int(msg.position[1] * scale),
        ])

        # PDO2: Achse 3 + 4
        self.send_pdo(0x202, [
            int(msg.position[2] * scale),
            int(msg.position[3] * scale)
        ])

        # PDO3: Achse 5
        self.send_pdo(0x203, [
            int(msg.position[4] * scale)
        ])

    def send_pdo(self, cob_id, values):
        # Werte in little endian int32
        data = b''.join(v.to_bytes(4, 'little', signed=True) for v in values)
        msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            self.get_logger().info(f"PDO {hex(cob_id)} gesendet: {values}")
        except can.CanError:
            self.get_logger().error(f"Fehler beim Senden von PDO {hex(cob_id)}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPDO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
