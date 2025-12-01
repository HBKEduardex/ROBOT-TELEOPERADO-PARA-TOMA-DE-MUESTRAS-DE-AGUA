#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import can
import math


class RobotStatus(Node):
    def __init__(self):
        super().__init__("robot_status")

        # Bus CAN
        self.bus = can.interface.Bus(channel="can0", bustype="socketcan")

        # Topic ROS
        self.pub = self.create_publisher(Float32MultiArray, "/robot_status", 10)

        # Variables internas
        self.kmh1 = self.kmh2 = self.vbat1 = None
        self.kmh3 = self.kmh4 = self.vbat2 = None

        # Control de frecuencia de publicación
        self.last_pub_time = self.get_clock().now()

        # Hilo dedicado a CAN
        self.can_thread = threading.Thread(target=self.can_loop, daemon=True)
        self.can_thread.start()

    # ---------------------------
    # Conversión RPM → km/h
    # ---------------------------
    def rpm_to_kmh(self, rpm):
        # diámetro = 21 cm → 0.21 m
        circumference = math.pi * 0.21   # metros
        km_per_rev = circumference / 1000.0
        kmh = rpm * km_per_rev * 60.0
        return kmh

    # ---------------------------
    # Decodificador frame RP2040
    # ---------------------------
    def decode_frame(self, msg):
        """Decodifica solo si el frame CAN tiene 6 bytes."""
        if msg.dlc < 6:
            return None

        r1_raw = (msg.data[0] << 8) | msg.data[1]
        r2_raw = (msg.data[2] << 8) | msg.data[3]

        # signed int16
        if r1_raw & 0x8000:
            r1_raw -= 65536
        if r2_raw & 0x8000:
            r2_raw -= 65536

        rpm1 = r1_raw / 10.0
        rpm2 = r2_raw / 10.0

        # Convertir a km/h
        kmh1 = self.rpm_to_kmh(rpm1)
        kmh2 = self.rpm_to_kmh(rpm2)

        vbat_raw = (msg.data[4] << 8) | msg.data[5]
        vbat = vbat_raw / 100.0

        return kmh1, kmh2, vbat

    # ---------------------------
    # Hilo CAN
    # ---------------------------
    def can_loop(self):
        while rclpy.ok():
            msg = self.bus.recv()
            if msg:
                self.process_msg(msg)

    # ---------------------------
    # Procesar frame CAN
    # ---------------------------
    def process_msg(self, msg):
        # Frame de motores 1 y 2
        if msg.arbitration_id == 0x140:
            decoded = self.decode_frame(msg)
            if decoded:
                kmh1, kmh2, vbat1 = decoded

                # 🔄 Invertir solo la rueda 1 (está montada al revés)
                kmh1 = -kmh1

                self.kmh1 = kmh1
                self.kmh2 = kmh2
                self.vbat1 = vbat1

        # Frame de motores 3 y 4
        elif msg.arbitration_id == 0x141:
            decoded = self.decode_frame(msg)
            if decoded:
                kmh3, kmh4_raw, vbat2 = decoded

                self.kmh3 = kmh3
                # 🔗 Forzar que la rueda 4 sea igual a la 3
                self.kmh4 = kmh3
                self.vbat2 = vbat2

        # Si ya tenemos todo → publicar
        if None not in (
            self.kmh1, self.kmh2, self.kmh3, self.kmh4, self.vbat1, self.vbat2
        ):
            self.publish_status()

    # ---------------------------
    # Publicar a ROS2 (33 Hz)
    # ---------------------------
    def publish_status(self):
        now = self.get_clock().now()
        if (now - self.last_pub_time).nanoseconds < 3e7:
            return

        msg_out = Float32MultiArray()
        msg_out.data = [
            float(self.kmh1), float(self.kmh2),
            float(self.kmh3), float(self.kmh4),
            float(self.vbat1), float(self.vbat2)
        ]

        self.pub.publish(msg_out)
        self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
