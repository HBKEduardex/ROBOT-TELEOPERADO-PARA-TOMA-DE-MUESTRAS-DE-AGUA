import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import can
import os
import time
import serial  # UART

# =======================
#   INTENTO OPCIONAL DE GPIO (BUZZER)
# =======================
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

BUZZER_PIN = 37  # pin físico 37 (modo BOARD)


# =======================
#   RESET AUTOMÁTICO DEL CANBUS
# =======================

def reset_can_interface():
    print("[CAN] Reiniciando interfaz CAN0...")

    os.system("sudo ip link set can0 down")
    os.system("sudo ip link set can0 type can bitrate 500000 loopback off")
    os.system("sudo ip link set can0 up")
    time.sleep(0.1)

    print("[CAN] CAN0 listo en modo NORMAL (sin loopback).")


reset_can_interface()

# ================================================
# CONFIGuración del bus CAN
# ================================================
bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_pwm(m1, m2, m3, m4):
    """Enviar 4 PWM por CAN (-100 a 100)"""

    m1 = max(-100, min(100, int(m1)))
    m2 = max(-100, min(100, int(m2)))
    m3 = max(-100, min(100, int(m3)))
    m4 = max(-100, min(100, int(m4)))

    msg12 = can.Message(
        arbitration_id=0x120,
        data=[m1 & 0xFF, m2 & 0xFF],
        is_extended_id=False
    )

    msg34 = can.Message(
        arbitration_id=0x121,
        data=[m3 & 0xFF, m4 & 0xFF],
        is_extended_id=False
    )

    bus.send(msg12)
    bus.send(msg34)


class CANPS4Bridge(Node):
    def __init__(self):
        super().__init__('can_ps4_bridge')

        # Suscripción PS4
        self.create_subscription(Float32MultiArray, '/Data_PS4', self.ps4_callback, 10)

        # Suscripción a estado del robot (RPM + baterías)
        self.create_subscription(Float32MultiArray, '/robot_status', self.status_callback, 10)

        # Publicador de streaming toggle
        self.pub_stream = self.create_publisher(Float32MultiArray, '/stream_cmd', 10)

        # Publicador para debug PWM
        self.pub_debug = self.create_publisher(Float32MultiArray, '/pwm_debug', 10)

        # Publicador para el valor del sensor leído por UART
        self.pub_sensor = self.create_publisher(Float32, '/sensor_value', 10)

        # Publicador temp CPU
        self.pub_temp = self.create_publisher(Float32, '/cpu_temperature', 10)

        # Estado de botones para flancos
        self.last_share = 0
        self.last_circle = 0    # para 's'
        self.last_triangle = 0  # para 'h'
        self.last_cross = 0     # para modo ECO (Cross/X)
        self.last_options = 0   # para STOP (Options)

        # Estado de modos
        self.eco_mode = False
        self.emergency_stop = False

        # Estado de alarms
        self.low_batt = False
        self.comm_lost = False
        self.overtemp = False

        # Últimos valores
        self.last_joy_time = time.time()

        # Baterías desde /robot_status
        self.vbat1 = None
        self.vbat2 = None

        # =======================
        #   UART /dev/ttyUSB0
        # =======================
        self.serial = None
        try:
            self.serial = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.05
            )
            self.get_logger().info("UART /dev/ttyUSB0 abierto correctamente.")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir UART: {e}")
            self.serial = None

        # =======================
        #   GPIO BUZZER
        # =======================
        self.buzzer_enabled = False
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.HIGH)
                self.buzzer_enabled = True
                self.get_logger().info("Buzzer inicializado en pin 37 (BOARD).")
            except Exception as e:
                self.get_logger().error(f"No se pudo inicializar buzzer: {e}")
                self.buzzer_enabled = False
        else:
            self.get_logger().warn("RPi.GPIO no disponible, buzzer deshabilitado.")

        # =======================
        #   TIMERS
        # =======================
        self.uart_timer = self.create_timer(0.05, self.read_uart_sensor)
        self.health_timer = self.create_timer(1.0, self.check_health)

        self.get_logger().info("Nodo CAN-PS4 iniciado ✔ (con UART, ECO, STOP y alarmas)")

    # ================================
    # CALLBACK /robot_status (RPM + VBAT)
    # ================================
    def status_callback(self, msg: Float32MultiArray):
        """
        msg.data = [rpm1, rpm2, rpm3, rpm4, vbat1, vbat2]
        """
        if len(msg.data) < 6:
            return

        self.vbat1 = float(msg.data[4])
        self.vbat2 = float(msg.data[5])
        # Si quieres ver:
        # self.get_logger().info(f"VBAT1={self.vbat1:.2f}  VBAT2={self.vbat2:.2f}")

    # ================================
    # UART: enviar comandos a la ESP32
    # ================================
    def send_uart_command(self, command_char: str):
        if self.serial is None:
            self.get_logger().warn("UART no inicializado, no se puede enviar comando.")
            return
        try:
            self.serial.write(command_char.encode('utf-8'))
            self.get_logger().info(f"UART -> enviado: {command_char}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error enviando por UART: {e}")

    # ================================
    # UART: leer datos de sensor desde ESP32
    # ================================
    def read_uart_sensor(self):
        """Lee datos del sensor desde la ESP32 y publica en /sensor_value (solo para telemetría, NO buzzer)."""
        if self.serial is None:
            return

        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line:
                    try:
                        value = float(line)
                        msg = Float32()
                        msg.data = value
                        self.pub_sensor.publish(msg)
                    except ValueError:
                        pass
        except serial.SerialException as e:
            self.get_logger().error(f"Error leyendo UART: {e}")

    # ================================
    # CHEQUEO DE SALUD + BUZZER
    # ================================
    def read_cpu_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                t_milli = int(f.read().strip())
            temp_c = t_milli / 1000.0
            msg = Float32()
            msg.data = float(temp_c)
            self.pub_temp.publish(msg)
            return temp_c
        except Exception:
            return None

    def check_health(self):
        # Comunicación PS4
        now = time.time()
        self.comm_lost = (now - self.last_joy_time) > 3.0  # más permisivo

        # Temperatura CPU
        temp_c = self.read_cpu_temperature()
        if temp_c is not None:
            self.overtemp = temp_c > 70.0  # umbral ejemplo

        # Batería baja DESDE /robot_status (vbat1, vbat2)
        if (self.vbat1 is not None) and (self.vbat2 is not None):
            vbat_min = min(self.vbat1, self.vbat2)
            self.low_batt = vbat_min < 11.1   # UMBRAL DEFINITIVO
        else:
            self.low_batt = False

        # Buzzer solo por alarmas críticas (no UART, no ECO)
        alarm_active = self.low_batt or self.comm_lost or self.overtemp or self.eco_mode
        

        if self.buzzer_enabled:
            try:
                GPIO.output(BUZZER_PIN, GPIO.HIGH if alarm_active else GPIO.LOW)
            except Exception as e:
                self.get_logger().error(f"Error controlando buzzer: {e}")

    # ================================
    # LÓGICA DE MOVIMIENTO
    # ================================
    def compute_pwm(self, LX, R2, L2, turbo_btn, turn_right, turn_left):


        MAX_PWM = 70
        ford_max=50
        TURBO = 20
        TURN_PWM = 30

        if R2 and not L2:
            direction = 1
        elif L2 and not R2:
            direction = -1
        else:
            direction = 0

        base = direction * ford_max

        if turbo_btn:
            base = min(MAX_PWM, base + TURBO)

        left_factor = 1.0 - LX
        right_factor = 1.0 + LX

        pwm_left = base * left_factor
        pwm_right = base * right_factor

        if turn_right and not turn_left:
            pwm_left = TURN_PWM
            pwm_right = -TURN_PWM
        elif turn_left and not turn_right:
            pwm_left = -TURN_PWM
            pwm_right = TURN_PWM

        pwm_left = int(max(-MAX_PWM, min(MAX_PWM, pwm_left)))
        pwm_right = int(max(-MAX_PWM, min(MAX_PWM, pwm_right)))

        return pwm_left, pwm_right

    # ================================
    # CALLBACK PS4
    # ================================
    def ps4_callback(self, msg):
        data = msg.data

        if len(data) < 20:
            self.get_logger().warn("Mensaje PS4 incompleto.")
            return

        self.last_joy_time = time.time()

        LX = data[0]
        L2 = data[4]
        R2 = data[5]

        Cross    = data[6]   # X -> ECO
        Circle   = data[7]   # 's'
        Triangle = data[8]   # 'h'
        Square   = data[9]   # turbo
        L1       = data[10]
        R1       = data[11]
        Share    = data[12]
        Options  = data[13]  # STOP

        # STOP EMERGENCIA PRIMERO
        options_pressed = Options == 1
        if options_pressed and not self.last_options:
            self.emergency_stop = not self.emergency_stop
            self.send_uart_command('X')
            if self.emergency_stop:
                self.get_logger().warn("STOP de emergencia ACTIVADO.")
            else:
                self.get_logger().warn("STOP de emergencia DESACTIVADO.")
        self.last_options = options_pressed

        if self.emergency_stop:
            left_pwm = 0
            right_pwm = 0
            send_pwm(left_pwm, right_pwm, left_pwm, right_pwm)

            debug_msg = Float32MultiArray()
            debug_msg.data = [float(left_pwm), float(right_pwm)]
            self.pub_debug.publish(debug_msg)
            return

        # MODO ECO
        cross_pressed = Cross == 1
        if cross_pressed and not self.last_cross:
            self.eco_mode= not self.eco_mode
            cmd = Float32MultiArray()
            if self.eco_mode:
                cmd.data = [0.0]   # streaming OFF
                self.get_logger().info("ECO ON → limitando velocidad y apagando streaming.")
            else:
                cmd.data = [1.0]   # streaming ON
                self.get_logger().info("ECO OFF → restaurando streaming.")
            self.pub_stream.publish(cmd)
        self.last_cross = cross_pressed

        # STREAM manual con SHARE
        if Share == 1 and self.last_share == 0:
            cmd = Float32MultiArray()
            cmd.data = [1.0]
            self.pub_stream.publish(cmd)
            self.get_logger().info("SHARE pressed → TOGGLE STREAM (ON).")
            self.last_share = 1
        elif Share == 0:
            self.last_share = 0

        # UART 's' y 'h'
        circle_pressed = Circle == 1
        triangle_pressed = Triangle == 1

        if circle_pressed and not self.last_circle:
            self.send_uart_command('s')
        self.last_circle = circle_pressed

        if triangle_pressed and not self.last_triangle:
            self.send_uart_command('h')
        self.last_triangle = triangle_pressed

        # Movimiento
        left_pwm, right_pwm = self.compute_pwm(
            LX=LX,
            R2=R2,
            L2=L2,
            turbo_btn=Square,
            turn_right=R1,
            turn_left=L1,
        )

        if self.eco_mode:
            ECO_LIMIT = 30
            left_pwm = max(-ECO_LIMIT, min(ECO_LIMIT, left_pwm))
            right_pwm = max(-ECO_LIMIT, min(ECO_LIMIT, right_pwm))

        send_pwm(left_pwm, right_pwm, left_pwm, right_pwm)

        debug_msg = Float32MultiArray()
        debug_msg.data = [float(left_pwm), float(right_pwm)]
        self.pub_debug.publish(debug_msg)

    # -------------------------------
    # CIERRE LIMPIO
    # -------------------------------
    def destroy_node(self):
        if self.buzzer_enabled:
            try:
                GPIO.output(BUZZER_PIN, GPIO.LOW)
                GPIO.cleanup()
            except Exception:
                pass

        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANPS4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
