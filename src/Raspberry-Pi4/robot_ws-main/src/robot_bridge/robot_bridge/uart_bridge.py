#!/usr/bin/env python3
"""
Nodo puente UART entre ROS 2 y myRIO.
Protocolo ASCII:
  Status (myRIO -> RPi):   <A1B2C3D4E5>       (sin cambios)
  Comandos (RPi -> myRIO):
    a<lin.x>b<lin.y>c<ang.z>d   → velocidad
    R<pos>                     → cremallera (aún por adaptar)
    G<0/1>                     → gripper
    E                          → emergencia
    P                          → ping
"""

import time
import re
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, UInt8, String, Int32

class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        self.declare_parameter('port', 'cat /dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('status_timeout', 0.5)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('status_timeout').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir {port}: {e}")
            raise

        self.last_status_time = time.time()
        self.connected = False
        self.fault_published = False
        self.rx_buffer = ""

        # Publicadores (datos recibidos del myRIO)
        self.pub_A = self.create_publisher(Int32, '/myrio/A', 10)
        self.pub_B = self.create_publisher(Int32, '/myrio/B', 10)
        self.pub_C = self.create_publisher(Int32, '/myrio/C', 10)
        self.pub_D = self.create_publisher(Int32, '/myrio/D', 10)
        self.pub_E = self.create_publisher(Int32, '/myrio/E', 10)
        self.pub_fault = self.create_publisher(String, '/fault_myrio', 10)

        # Suscriptores
        self.sub_cmd_vel  = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_rack     = self.create_subscription(Int8, '/rack_cmd', self.rack_cb, 10)
        self.sub_gripper  = self.create_subscription(Bool, '/gripper_cmd', self.gripper_cb, 10)
        self.sub_flags    = self.create_subscription(UInt8, '/robot_flags', self.flags_cb, 10)

        # Hilo de recepción
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.create_timer(0.2, self.check_connection)

        self.get_logger().info("UART Bridge (formato aXbYcZd) iniciado.")

    # ------------------------------------------------------------------
    # Envío de comandos (cmd_vel adaptado al nuevo formato)
    # ------------------------------------------------------------------
    def _send_line(self, line: str):
        """Envía una línea por UART con '\n' al final."""
        with threading.Lock():
            try:
                self.ser.write((line + '\n').encode('ascii'))
                self.ser.flush()
            except serial.SerialException:
                self.get_logger().error("Error al escribir en UART")

    def cmd_vel_cb(self, msg: Twist):
        """
        Convierte Twist a la trama: a<lin.x>b<lin.y>c<ang.z>d
        Ejemplo: a0.20b0.00c-0.50d
        """
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        ang_z = msg.angular.z
        # Formato requerido: a0.20b0.00c-0.50d
        self._send_line(f"a{lin_x:.2f}b{lin_y:.2f}c{ang_z:.2f}d")

    def rack_cb(self, msg: Int8):
        # Mantenido sin cambios por ahora
        pos = max(0.0, min(1.0, float(msg.data)))
        self._send_line(f"R{pos:.2f}")

    def gripper_cb(self, msg: Bool):
        val = 1 if msg.data else 0
        self._send_line(f"G{val}")

    def flags_cb(self, msg: UInt8):
        if msg.data & 0x01:
            self._send_line("E")

    # ------------------------------------------------------------------
    # Recepción de tramas de estado (formato <A...E>)
    # ------------------------------------------------------------------
    def rx_loop(self):
        while rclpy.ok():
            try:
                byte = self.ser.read().decode('ascii', errors='ignore')
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not byte:
                continue
            if byte == '\n':
                line = self.rx_buffer.strip()
                self.rx_buffer = ""
                if line:
                    self._process_status(line)
            else:
                self.rx_buffer += byte

    def _process_status(self, line: str):
        self.last_status_time = time.time()
        if not self.connected:
            self.connected = True
            self.get_logger().info("Comunicación con myRIO establecida.")
            self.fault_published = False

        match = re.match(r'^<A(\d+)B(\d+)C(\d+)D(\d+)E(\d+)>$', line)
        if not match:
            self.get_logger().warn(f"Trama no reconocida: '{line}'")
            return

        try:
            a = int(match.group(1))
            b = int(match.group(2))
            c = int(match.group(3))
            d = int(match.group(4))
            e = int(match.group(5))
            self.pub_A.publish(Int32(data=a))
            self.pub_B.publish(Int32(data=b))
            self.pub_C.publish(Int32(data=c))
            self.pub_D.publish(Int32(data=d))
            self.pub_E.publish(Int32(data=e))
        except ValueError as ex:
            self.get_logger().warn(f"Error al convertir: {ex}")

    def check_connection(self):
        now = time.time()
        if (now - self.last_status_time) > self.timeout:
            if self.connected:
                self.connected = False
                self.get_logger().error("Timeout de comunicación con myRIO.")
            if not self.fault_published:
                self.pub_fault.publish(String(data="LOST_COMMUNICATION"))
                self.fault_published = True
        else:
            if not self.connected:
                self.connected = True
                self.get_logger().info("Comunicación reestablecida.")
                self.fault_published = False

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()