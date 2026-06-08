#!/usr/bin/env python3
"""
Nodo ROS2 que ofrece el servicio /detect_pipe.
Ejecuta el pipeline: captura → panorama → Roboflow → detección.
"""

import cv2
import requests
import json
import base64
import os
import time
import subprocess
import re
import numpy as np

import rclpy
from rclpy.node import Node
from camera_detection_interfaces.srv import DetectPipe


API_KEY = "BqGtHqYl1bk4vT4Vshza"
URL_ROBOFLOW = "https://serverless.roboflow.com/calebs-workspace-lepra/workflows/pvc-pipe-navigation-pipeline-1776989466746"
TARGET_CAMERA = "Logitech BRIO"
RES_CAPTURA = (1280, 720)
CARPETA_FOTOS = "temp_fotos"

class PipeDetectionService(Node):
    def __init__(self):
        super().__init__('pipe_detection_service')

        # Parámetros opcionales para cambiar comportamiento desde launch
        self.declare_parameter('num_photos', 12)
        self.declare_parameter('photo_interval', 0.8)  # segundos entre fotos
        self.declare_parameter('panorama_binary', 'panorama_robusto')
        self.declare_parameter('temp_folder', CARPETA_FOTOS)

        self.num_photos = self.get_parameter('num_photos').value
        self.interval = self.get_parameter('photo_interval').value
        self.pano_bin = self.get_parameter('panorama_binary').value
        self.temp_folder = self.get_parameter('temp_folder').value

        # Asegurar carpeta temporal
        if not os.path.exists(self.temp_folder):
            os.makedirs(self.temp_folder)

        self.srv = self.create_service(DetectPipe, 'detect_pipe', self.handle_detect_pipe)
        self.get_logger().info('Servicio de detección de tubería listo.')

    def handle_detect_pipe(self, request, response):
        self.get_logger().info("Iniciando pipeline de detección...")

        fotos = self.capturar_secuencia()
        if len(fotos) < 2:
            response.message = "No se pudieron capturar suficientes fotos"
            return response

        pano_path = self.procesar_panorama(fotos)
        if pano_path is None:
            response.message = "Fallo al unir las imágenes"
            return response

        res_json = self.analizar_roboflow(pano_path)
        if res_json is None:
            response.message = "Error en la comunicación con Roboflow"
            return response

        found, angle, conf, count = self.extraer_detecciones(res_json, pano_path)
        response.found = found
        response.angle = angle
        response.confidence = conf
        response.num_pipes = count
        response.message = "Pipeline completado"
        self.get_logger().info(f"Detección finalizada. Encontrado: {found}, ángulo: {angle:.1f}°")
        return response

    # ------------------------------------------------------------------------
    # Métodos adaptados de tu código original
    # ------------------------------------------------------------------------
    def conectar_camara(self):
        self.get_logger().info(f"Buscando '{TARGET_CAMERA}'...")
        try:
            output = subprocess.check_output("v4l2-ctl --list-devices", shell=True).decode()
            bloques = output.split("\n\n")
            for bloque in bloques:
                if TARGET_CAMERA in bloque:
                    match = re.search(r"/dev/video(\d+)", bloque)
                    if match:
                        idx = int(match.group(1))
                        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                        if cap.isOpened():
                            self.get_logger().info(f"Cámara en /dev/video{idx}")
                            return cap
        except Exception as e:
            self.get_logger().warn(f"Error buscando cámara: {e}")

        self.get_logger().warn("Usando /dev/video0 por defecto")
        return cv2.VideoCapture(0)

    def capturar_secuencia(self):
        cap = self.conectar_camara()
        if not cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")
            return []

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, RES_CAPTURA[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_CAPTURA[1])

        archivos = []
        self.get_logger().info(f"Capturando {self.num_photos} fotos...")
        for i in range(self.num_photos):
            time.sleep(0.5)  # estabilización
            for _ in range(5): cap.grab()  # limpiar buffer
            ret, frame = cap.read()
            if ret:
                ruta = os.path.abspath(os.path.join(self.temp_folder, f"f_{i}.jpg"))
                cv2.imwrite(ruta, frame)
                archivos.append(ruta)
                if i < self.num_photos - 1:
                    time.sleep(self.interval)
            else:
                self.get_logger().warn(f"Fallo en captura {i}")
        cap.release()
        return archivos

    def procesar_panorama(self, archivos):
        # El binario debe estar junto a este script (carpeta camera_detection)
        bin_path = os.path.join(os.path.dirname(__file__), 'panorama_robusto')
        if not os.path.exists(bin_path):
            self.get_logger().error(f"No existe el binario: {bin_path}")
            return None
        try:
            self.get_logger().info("Uniendo fotos...")
            subprocess.run([bin_path] + archivos, check=True, timeout=120)
            return "panorama_perfecto.jpg"
        except subprocess.CalledProcessError:
            self.get_logger().error("Fallo en la costura")
            return None

    def analizar_roboflow(self, ruta_img):
        if not os.path.exists(ruta_img):
            return None
        self.get_logger().info("Enviando a Roboflow...")
        with open(ruta_img, "rb") as f:
            img_b64 = base64.b64encode(f.read()).decode('utf-8')
        payload = {
            "api_key": API_KEY,
            "inputs": {"image": {"type": "base64", "value": img_b64}}
        }
        try:
            r = requests.post(URL_ROBOFLOW, json=payload, timeout=60)
            if r.status_code == 200:
                res = r.json()
                with open("resultado.json", "w") as fjson:
                    json.dump(res, fjson, indent=4)
                return res
            else:
                self.get_logger().error(f"Roboflow respondió {r.status_code}: {r.text}")
                return None
        except Exception as e:
            self.get_logger().error(f"Fallo de conexión: {e}")
            return None

    def extraer_detecciones(self, res_json, ruta_img):
        try:
            output = res_json.get('outputs', [{}])[0]

            # Guardar imagen anotada si viene en la respuesta
            if 'annotated_image' in output:
                img_b64 = output['annotated_image'].get('value', "")
                if img_b64:
                    with open("resultado_roboflow.jpg", "wb") as f:
                        f.write(base64.b64decode(img_b64))

            # Buscar predicciones dinámicamente
            predictions = []
            for key, value in output.items():
                if isinstance(value, list) and len(value) > 0:
                    if isinstance(value[0], dict) and 'class' in value[0]:
                        predictions = value
                        self.get_logger().info(f"Predicciones en llave: '{key}'")
                        break

            if not predictions:
                # Sin detecciones, dibujamos la imagen original limpia
                img = cv2.imread(ruta_img)
                if img is not None:
                    cv2.imwrite("panorama_analizado_local.jpg", img)
                return False, 0.0, 0.0, 0

            # Leer imagen original para dibujar y calcular ángulos
            img = cv2.imread(ruta_img)
            if img is None:
                return False, 0.0, 0.0, 0
            h, w = img.shape[:2]
            centro_x = w / 2.0

            angles = []
            confs = []
            for pred in predictions:
                x = pred.get('x', 0)
                y = pred.get('y', 0)
                w_box = pred.get('width', 0)
                h_box = pred.get('height', 0)
                conf = pred.get('confidence', 0)
                confs.append(conf)

                # Ángulo proporcional: ±30° en función del desplazamiento horizontal
                angle = (x - centro_x) / (w / 2.0) * 30.0
                angles.append(angle)

                # Dibujar rectángulo y texto
                x1, y1 = int(x - w_box/2), int(y - h_box/2)
                x2, y2 = int(x + w_box/2), int(y + h_box/2)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                label = pred.get('class', 'Objeto')
                cv2.putText(img, f"{label} {conf:.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imwrite("panorama_analizado_local.jpg", img)
            avg_angle = float(np.mean(angles))
            avg_conf = float(np.mean(confs))
            count = len(predictions)
            return True, avg_angle, avg_conf, count

        except Exception as e:
            self.get_logger().error(f"Error en extracción: {e}")
            return False, 0.0, 0.0, 0

def main(args=None):
    rclpy.init(args=args)
    node = PipeDetectionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()