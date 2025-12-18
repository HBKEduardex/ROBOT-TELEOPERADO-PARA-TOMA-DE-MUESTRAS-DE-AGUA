#!/usr/bin/env python3
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QProgressBar, QDial
)
from PySide6.QtCore import QTimer

import pyqtgraph as pg


# ==========================
#   CONSTANTES AJUSTABLES
# ==========================
MAX_SPEED_KMH = 4.0     # dial 0–4 km/h
BATTERY_V_MIN = 11.1
BATTERY_V_MAX = 12.6


def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))


# ====================================
#   NODO ROS2 RECEPTOR
# ====================================
class DashboardStatusNode(Node):
    def __init__(self):
        super().__init__('dashboard_status_node')
        self.gui = None

        # --- robot_status: velocidades y baterías ---
        self.sub_status = self.create_subscription(
            Float32MultiArray,
            '/robot_status',
            self.status_callback,
            10
        )

        # --- cpu_temperature ---
        self.sub_temp = self.create_subscription(
            Float32,
            '/cpu_temperature',
            self.temp_callback,
            10
        )

        self.get_logger().info("DashboardStatusNode listo (/robot_status + /cpu_temperature).")

    def status_callback(self, msg: Float32MultiArray):
        if self.gui is None:
            return

        data = msg.data
        if len(data) < 6:
            return

        v1 = float(data[0])
        v2 = float(data[1])
        v3 = float(data[2])
        v4 = float(data[3])
        b1 = float(data[4])
        b2 = float(data[5])

        v_lineal_kmh = (v1 + v2 + v3 + v4) / 4.0
        v_batt = min(b1, b2)

        self.gui.update_speed_and_battery(v_lineal_kmh, v_batt)

    def temp_callback(self, msg: Float32):
        if self.gui is None:
            return
        self.gui.update_temperature(float(msg.data))


# ====================================
#   DASHBOARD GUI
# ====================================
class DashboardGUI(QWidget):
    def __init__(self, node: DashboardStatusNode):
        super().__init__()
        self.node = node
        self.node.gui = self

        self.setWindowTitle("Dashboard Robot 🎛️ Velocidad, Batería y Temperatura")

        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        # ===== TEXTO Velocidad =====
        self.label_speed = QLabel("Velocidad lineal: 0.00 km/h")
        left_layout.addWidget(self.label_speed)

        # ===== TACÓMETRO (QDial) =====
        self.speed_dial = QDial()
        self.speed_dial.setNotchesVisible(True)
        self.speed_dial.setMinimum(0)
        self.speed_dial.setMaximum(int(MAX_SPEED_KMH * 10))  # 0..40 → 0..4.0 km/h
        self.speed_dial.setEnabled(False)
        left_layout.addWidget(self.speed_dial)

        # ===== BATERÍA =====
        self.label_batt = QLabel("Batería: 0.0 V (0 %)")
        left_layout.addWidget(self.label_batt)

        self.batt_bar = QProgressBar()
        self.batt_bar.setRange(0, 100)
        self.batt_bar.setValue(0)
        self.batt_bar.setFormat("0 %")
        left_layout.addWidget(self.batt_bar)

        # ===== TEMPERATURA =====
        self.label_temp = QLabel("Temperatura CPU: 0.0 °C")
        left_layout.addWidget(self.label_temp)

        # ===== GRÁFICO =====
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('bottom', "Tiempo", units='s')
        self.plot_widget.setLabel('left', "Velocidad lineal", units='km/h')
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)

        self.t = deque(maxlen=200)
        self.v_hist = deque(maxlen=200)
        self.t0 = time.time()

        self.curve = self.plot_widget.plot([], [], name='V lineal (km/h)')
        right_layout.addWidget(self.plot_widget)

        # Ensamblar Layout
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

        # Timers
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(10)

    # =====================================
    #   ACTUALIZAR VELOCIDAD + BATERÍA
    # =====================================
    def update_speed_and_battery(self, v_lineal_kmh, v_batt):

        # ----- Velocidad -----
        self.label_speed.setText(f"Velocidad lineal: {v_lineal_kmh:.2f} km/h")
        # Valor absoluto para que el dial suba aunque la velocidad sea negativa
        v_abs = abs(v_lineal_kmh)

        # Limitar a 0–MAX_SPEED_KMH
        v_abs = clamp(v_abs, 0.0, MAX_SPEED_KMH)

        # El dial trabaja en décimas (0–40 si MAX_SPEED_KMH = 4)
        self.speed_dial.setValue(int(v_abs * 10))

        t_now = time.time() - self.t0
        self.t.append(t_now)
        self.v_hist.append(v_lineal_kmh)

        # ----- Batería -----
        batt_pct = (v_batt - BATTERY_V_MIN) / (BATTERY_V_MAX - BATTERY_V_MIN) * 100
        batt_pct = clamp(batt_pct, 0, 100)

        self.batt_bar.setValue(int(batt_pct))
        self.batt_bar.setFormat(f"{batt_pct:.0f} %")
        self.label_batt.setText(f"Batería: {v_batt:.2f} V ({batt_pct:.0f} %)")

    # =====================================
    #       ACTUALIZAR TEMPERATURA
    # =====================================
    def update_temperature(self, temp_c):
        self.label_temp.setText(f"Temperatura CPU: {temp_c:.1f} °C")

    # =====================================
    #       GRÁFICA TIEMPO REAL
    # =====================================
    def update_plot(self):
        if len(self.t) == 0:
            return
        self.curve.setData(list(self.t), list(self.v_hist))
        self.plot_widget.setXRange(min(self.t), max(self.t))

    # =====================================
    #       ROS SPIN
    # =====================================
    def ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


# ====================================
#   MAIN
# ====================================
def main():
    rclpy.init()
    app = QApplication(sys.argv)

    node = DashboardStatusNode()
    gui = DashboardGUI(node)
    gui.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
