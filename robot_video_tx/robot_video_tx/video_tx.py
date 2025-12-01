#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib, GObject

Gst.init(None)
GObject.threads_init()


class VideoStreamer(Node):
    def __init__(self):
        super().__init__("video_streamer")

        self.streaming = False
        self.pipeline = None
        self.valve = None
        self.glib_loop = None
        self.glib_thread = None

        self.last_toggle = 0.0
        self.toggle_debounce = 0.4  # evita toggles muy rápidos (ya casi no lo usas, pero ok)

        # Suscripción al comando de toggle enviado desde ROS2
        self.create_subscription(Float32MultiArray, "/stream_cmd", self.toggle_callback, 10)

        self.get_logger().info("✔ VideoStreamer listo (GStreamer con valve — MAX ESTABILIDAD)")

        # Construimos pipeline una sola vez
        self.build_pipeline()
        self.start_glib_loop()

        # 🔥 ARRANCAR YA CON STREAMING ACTIVADO
        self.start_stream()

    def build_pipeline(self):
        pipeline_str = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=I420 ! "
            "valve name=valve drop=true ! "
            "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
            "rtph264pay pt=96 ! "
            "udpsink host=192.168.0.28 port=5000 sync=false async=false"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.valve = self.pipeline.get_by_name("valve")

        # Start pipeline but with valve=drop:true (sending OFF)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.get_logger().info("Pipeline creado, cámara inicializada, valve=drop (OFF)")

    def start_glib_loop(self):
        """GLib loop para procesar mensajes del bus de GStreamer."""
        if self.glib_loop:
            return

        self.glib_loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.glib_loop.run, daemon=True)
        self.glib_thread.start()
        self.get_logger().debug("GLib loop iniciado")

        # Escuchar mensajes del bus
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

    def on_bus_message(self, bus, message):
        t = message.type

        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            self.get_logger().error(f"[Gst ERROR] {err} — {dbg}")

        elif t == Gst.MessageType.WARNING:
            err, dbg = message.parse_warning()
            self.get_logger().warn(f"[Gst WARNING] {err} — {dbg}")

        elif t == Gst.MessageType.EOS:
            self.get_logger().info("[Gst] EOS recibido (no debería ocurrir en valve mode)")

    def toggle_callback(self, msg):
        cmd = int(msg.data[0])

        if cmd == 1:
            # FORZAR ON
            if not self.streaming:
                self.start_stream()
        elif cmd == 0:
            # FORZAR OFF
            if self.streaming:
                self.stop_stream()

    def start_stream(self):
        """Abrir la válvula — No reabrimos cámara, no tocamos pipeline."""
        if self.streaming:
            return

        self.valve.set_property("drop", False)
        self.streaming = True
        self.get_logger().info("📡 Streaming ACTIVADO (valve=ON)")

    def stop_stream(self):
        """Cerrar la válvula — El pipeline sigue vivo, no se cierra la cámara."""
        if not self.streaming:
            return

        self.valve.set_property("drop", True)
        self.streaming = False
        self.get_logger().info("🛑 Streaming DESACTIVADO (valve=OFF)")

    def destroy_node(self):
        """Apagado limpio antes de cerrar ROS2."""
        self.get_logger().info("🛑 Apagando VideoStreamer…")

        try:
            self.valve.set_property("drop", True)
        except Exception:
            pass

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

        if self.glib_loop:
            try:
                self.glib_loop.quit()
            except Exception:
                pass

        if self.glib_thread and threading.current_thread() is not self.glib_thread:
            self.glib_thread.join(timeout=0.5)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
