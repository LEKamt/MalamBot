#include <PRIZM_PRO.h>
PRIZM prizm;

const int RX_PIN = 18;
const int TX_PIN = 17;

unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 100;

// Buffer para recepción no bloqueante
String bufferRX = "";

// ---------------------------------------- DECLARACIÓN DE VARIABLES ----------------------------------------
// >> Velocidades (rad/s)
float wl_cmd = 0.0;              // velocidad angular izquierda (rad/s)
float wr_cmd = 0.0;              // velocidad angular derecha (rad/s)
const float rad2deg = 57.29578;  // 180/pi

void setup() {
  prizm.PrizmBegin();
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("Listo");
  //prizm.setServoSpeed(1,25);  // set servo 1 speed to 25%
  delay(100);
  prizm.setMotorSpeed(1, 70);  // 50% potencia
  delay(100);
  prizm.setMotorSpeed(2, 70);
  delay(1000);
  prizm.setMotorSpeed(1, 0);
  delay(100);
  prizm.setMotorSpeed(2, 0);
  delay(1000);
  prizm.setServoPosition(1, 0);
  delay(100);

}

void loop() {
  
  //Serial.print(prizm.getSonicSensor(1, UNITS_CM));   // print the distance measured in centimeters on sensor port 3 to the serial monitor. 
  //Serial.print(prizm.getSonicSensor(2, UNITS_CM));   // print the distance measured in centimeters on sensor port 3 to the serial monitor. 

  //Serial.println(" Centimeters");                    // print " Centimeters"

  // --- ENVIAR cada 100ms sin bloquear ---
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = millis();

    float dist1 = prizm.readEncoderDegrees(1);//prizm.getSonicSensor(5, UNITS_CM);
    float dist2 = prizm.readEncoderDegrees(2);//prizm.getSonicSensor(6, UNITS_CM);
    float dist3 = 30.89 + (millis() % 20);

    // En el bloque de envío (cada 100 ms)
    Serial1.print('a');        // minúscula
    Serial1.print(dist1, 2);
    Serial1.print('b');
    Serial1.print(dist2, 2);
    Serial1.print('c');
    Serial1.print(dist3, 2);
    Serial1.print('d');
    Serial1.print("\r\n");

  }

  // --- RECIBIR sin bloquear ---
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\n') {
      bufferRX.trim();
      if (bufferRX.length() > 0) {
        // Usar int para los índices (no float)
        int idxA = bufferRX.indexOf('A');
        int idxB = bufferRX.indexOf('B');
        int idxC = bufferRX.indexOf('C');

        if (idxA != -1 && idxB != -1 && idxC != -1) {
          // El valor de A (servo) se queda entero (sin dividir)
          float valorA = bufferRX.substring(idxA + 1, idxB).toFloat();
          
          // B y C: se extraen como flotantes pero representan centésimas
          float raw_wl = bufferRX.substring(idxB + 1, idxC).toFloat();
          float raw_wr = bufferRX.substring(idxC + 1).toFloat();

          // Convertir de centésimas a rad/s (dividir entre 100)
          wl_cmd = raw_wl / 100.0;
          wr_cmd = raw_wr / 100.0;

          // Limitar rangos (-12.566 a 12.566 rad/s)
          wl_cmd = constrain(wl_cmd, -12.566, 12.566);
          wr_cmd = constrain(wr_cmd, -12.566, 12.566);

          // Depuración
          Serial.print("A (servo)="); Serial.println(valorA);
          Serial.print("B (wl en rad/s)="); Serial.println(wl_cmd);
          Serial.print("C (wr en rad/s)="); Serial.println(wr_cmd);

          // Aplicar a los actuadores
          prizm.setServoPosition(1, (int)valorA);
          delay(20);
          prizm.setMotorSpeed(1, (int)(wl_cmd * rad2deg));
          delay(20);
          prizm.setMotorSpeed(2, (int)(wr_cmd * rad2deg));
          delay(20);

        }
      }
      bufferRX = "";
    } else {
      bufferRX += c;
    }
  }
             
//delay(10);
  // Sin delay bloqueante
}
