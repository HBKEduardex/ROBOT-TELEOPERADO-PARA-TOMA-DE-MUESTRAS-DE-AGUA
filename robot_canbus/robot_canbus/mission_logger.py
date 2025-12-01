#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import os
import csv
import datetime


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        # Últimos valores recibidos
        # /robot_status → [Vm1, Vm2, Vm3, Vm4, vbat1, vbat2]
        self.last_vmotor1 = None
        self.last_vmotor2 = None
        self.last_vmotor3 = None
        self.last_vmotor4 = None
        self.last_vbat1 = None
        self.last_vbat2 = None

        # Velocidad lineal promedio
        self.last_v_lineal = None

        # /cpu_temperature
        self.last_temp = None

        # pH desde /sensor_value
        self.last_ph = None

        # Suscriptores
        self.create_subscription(
            Float32MultiArray,
            '/robot_status',
            self.status_callback,
            10
        )
        self.create_subscription(
            Float32,
            '/cpu_temperature',
            self.temp_callback,
            10
        )
        # pH desde el nodo CANPS4Bridge (/sensor_value)
        self.create_subscription(
            Float32,
            '/sensor_value',
            self.ph_callback,
            10
        )

        # CSV
        self.csv_file = None
        self.csv_writer = None
        self.init_csv()

        # Timer: log cada 2 s
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info(
            "MissionLogger listo (4 Vmotores, 2 baterías, temp, V_lineal y pH cada 2 s)."
        )

    # ---- Callbacks de suscripción ----
    def status_callback(self, msg: Float32MultiArray):
        """
        Espera msg.data = [Vmotor1, Vmotor2, Vmotor3, Vmotor4, Bateria_1, Bateria_2]
        publicado por /robot_status.
        """
        data = msg.data
        if len(data) < 6:
            self.get_logger().warn(
                f"/robot_status con longitud inesperada: {len(data)}"
            )
            return

        v1 = float(data[0])
        v2 = float(data[1])
        v3 = float(data[2])
        v4 = float(data[3])
        b1 = float(data[4])
        b2 = float(data[5])

        self.last_vmotor1 = v1
        self.last_vmotor2 = v2
        self.last_vmotor3 = v3
        self.last_vmotor4 = v4
        self.last_vbat1 = b1
        self.last_vbat2 = b2

        # Promedio de las 4 velocidades (velocidad lineal)
        self.last_v_lineal = (v1 + v2 + v3 + v4) / 4.0

    def temp_callback(self, msg: Float32):
        self.last_temp = float(msg.data)

    def ph_callback(self, msg: Float32):
        # pH leído desde /sensor_value
        self.last_ph = float(msg.data)

    # ---- Inicialización de CSV ----
    def init_csv(self):
        base_dir = "/home/j1nzo/csvs"
        os.makedirs(base_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(base_dir, f"logger_{ts}.csv")
        try:
            self.csv_file = open(filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            # Cabecera (respetando tus nombres + 2 nuevas columnas)
            self.csv_writer.writerow([
                "timestamp",
                "Vmotor1",
                "Vmotor2",
                "Vmotor3",
                "Vmotor4",
                "Bateria_1",
                "Bateria_2",
                "cpu_temp_C",
                "V_lineal",
                "pH",
            ])
            self.get_logger().info(f"MissionLogger guardando en: {filename}")
        except Exception as e:
            self.get_logger().error(f"No se pudo crear CSV: {e}")
            self.csv_file = None
            self.csv_writer = None

    # ---- Timer: log cada 2 s ----
    def timer_callback(self):
        if self.csv_writer is None:
            return

        # Si no hemos recibido nada todavía, no logeamos
        if (
            self.last_vmotor1 is None and
            self.last_vmotor2 is None and
            self.last_vmotor3 is None and
            self.last_vmotor4 is None and
            self.last_vbat1 is None and
            self.last_vbat2 is None and
            self.last_temp is None and
            self.last_v_lineal is None and
            self.last_ph is None
        ):
            return

        ts = datetime.datetime.now().isoformat()

        try:
            self.csv_writer.writerow([
                ts,
                self.last_vmotor1 if self.last_vmotor1 is not None else "",
                self.last_vmotor2 if self.last_vmotor2 is not None else "",
                self.last_vmotor3 if self.last_vmotor3 is not None else "",
                self.last_vmotor4 if self.last_vmotor4 is not None else "",
                self.last_vbat1 if self.last_vbat1 is not None else "",
                self.last_vbat2 if self.last_vbat2 is not None else "",
                self.last_temp if self.last_temp is not None else "",
                self.last_v_lineal if self.last_v_lineal is not None else "",
                self.last_ph if self.last_ph is not None else "",
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f"Error escribiendo CSV: {e}")

    def destroy_node(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
