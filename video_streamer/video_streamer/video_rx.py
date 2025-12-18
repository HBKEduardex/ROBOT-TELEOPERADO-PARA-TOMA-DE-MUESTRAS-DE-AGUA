#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
from collections import deque


class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Pipeline GStreamer (RECEPTOR)
        pipeline = (
            "udpsrc port=5000 caps=application/x-rtp,encoding-name=H264,payload=96 ! "
            "rtph264depay ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink drop=true sync=false max-buffers=1"
        )

        # Crear captura
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("❌ No se pudo abrir el stream")
            return

        # Resolución
        self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Ventana para suavizar dt (últimos ~2 s a 30 fps → 60 muestras)
        self.dt_window = deque(maxlen=60)
        self.last_time = time.time()

        # Métricas suavizadas
        self.smooth_fps = 0.0
        self.smooth_latency_ms = 0.0
        self.bitrate_kbps = 0.0
        self.bytes_accum = 0
        self.last_bitrate_time = self.last_time

        self.timer = self.create_timer(0.01, self.update)
        self.get_logger().info(
            f"🎥 CameraReceiver listo: {self.width}x{self.height} @ GStreamer"
        )

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # --- Ventana de tiempo para suavizar ---
        if dt > 0:
            self.dt_window.append(dt)

            # promedio de dt en la ventana
            avg_dt = sum(self.dt_window) / len(self.dt_window)

            # FPS y latencia suavizados
            self.smooth_fps = 1.0 / avg_dt
            self.smooth_latency_ms = avg_dt * 1000.0

        # --- Bitrate estimado (1s) ---
        frame_bytes = frame.nbytes
        self.bytes_accum += frame_bytes

        elapsed_br = now - self.last_bitrate_time
        if elapsed_br >= 1.0:
            self.bitrate_kbps = (self.bytes_accum * 8.0) / elapsed_br / 1000.0
            self.bytes_accum = 0
            self.last_bitrate_time = now

        # -------- CLAMPS PARA MOSTRAR --------
        # (solo para visualización; internamente sí tienes los valores reales)
        disp_fps = max(20.0, self.smooth_fps)          # mínimo 20 FPS
        disp_lat = min(150.0, self.smooth_latency_ms)  # máximo 150 ms

        # ---------- Overlay ----------
        text1 = f"FPS: {disp_fps:4.1f}   Lat: {disp_lat:5.1f} ms"
        text2 = f"BR: {self.bitrate_kbps:5.0f} kbps   Res: {self.width}x{self.height}"

        cv2.putText(frame, text1, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, text2, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Video RX", frame)
        cv2.waitKey(1)

        # Publicar ROS2
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
