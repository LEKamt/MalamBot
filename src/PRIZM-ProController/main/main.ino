#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PRIZM_PRO.h>
#include <Ticker.h>

PRIZM prizm;
Adafruit_VL53L0X lox;

const int RX_PIN = 18;
const int TX_PIN = 17;

// ---------------------------------------- DECLARACIÓN DE VARIABLES ----------------------------------------
// >>> Buffer y recepción
String bufferRX = "";

// >>> Timer de control
Ticker controlTimer;
volatile bool aplicarControl = false;  // Flag modificado en ISR

// >>> Variables compartidas
volatile int riel_cmd = 0;
volatile int servo_cmd = 0;
volatile float wl_cmd = 0.0f;
volatile float wr_cmd = 0.0f;
const float rad2deg = 57.29578f;

// >>> Máquina de estados para sensores
enum class SensorStep { TOF,
                        SONAR1,
                        SONAR2,
                        ENCODERS,
                        SEND };
SensorStep sensorStep = SensorStep::TOF;
unsigned long lastSensorTime = 0;  // Para controlar el intervalo total
const int SENSOR_INTERVAL = 100;   // ms entre envíos completos

float distanciaTOF = -1;
float dist1 = -1, dist2 = -1;
float Encoder1 = 0, Encoder2 = 0;


// ----------------------------------------------- FUNCIONES -----------------------------------------------
// >>> ISR del timer
void IRAM_ATTR onControlTimer() {
  aplicarControl = true;
}

// >>> Aplicación de comandos a los actuadores
void aplicarActuadores() {
  prizm.setServoPosition(1, servo_cmd);
  delay(1);
  prizm.setMotorSpeed(1, (int)(wl_cmd * rad2deg));
  delay(1);
  prizm.setMotorSpeed(2, (int)(wr_cmd * rad2deg));
  delay(1);
  prizm.setMotorPower(3, riel_cmd);
  delay(1);
}


// ----------------------------------------- CONFIGURACIÓN INICIAL -----------------------------------------
void setup() {
  Serial.begin(115200);

  // >>> Buffer de recepción UART1
  Serial1.setRxBufferSize(512);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // >>> VL53L0X
  Wire.begin(16, 15);
  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("Fallo VL53L0X");
    while (1)
      ;
  }

  // >>> Reducir tiempo de medición a 20 ms
  // lox.setMeasurementTimingBudgetMicroSeconds(20000);
  Serial.println("VL53L0X OK");

  // >>> Configurar PRIZM
  prizm.PrizmBegin();
  prizm.setMotorPower(1, 0);
  prizm.setMotorPower(2, 0);
  prizm.setMotorPower(3, 0);
  prizm.setServoPosition(1, servo_cmd);

  // >>> Configurar timer cada 20 ms (50 Hz)
  controlTimer.attach_ms(20, onControlTimer);

  // >>> Mostrar mensaje de finalización
  Serial.println("PRIZM listo");
}


// -------------------------------------------- BUCLE PRINCIPAL --------------------------------------------
void loop() {
  // >>> 1. Procesar recepción UART
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      bufferRX.trim();
      int idxB = bufferRX.indexOf('B');
      int idxC = bufferRX.indexOf('C');
      int idxD = bufferRX.indexOf('D');
      int idxE = bufferRX.indexOf('E');

      if (idxB != -1 && idxC != -1 && idxD != -1 && idxE != -1) {
        // >> Leer y saturar las variables de control
        int servoRaw = bufferRX.substring(idxB + 1, idxC).toInt();
        int powerRaw = bufferRX.substring(idxC + 1, idxD).toInt();
        float raw_wl = bufferRX.substring(idxD + 1, idxE).toFloat() / 1000.0f;
        float raw_wr = bufferRX.substring(idxE + 1).toFloat() / 1000.0f;

        servo_cmd = constrain(servoRaw / 1000.0f, 0, 180);
        riel_cmd = constrain(powerRaw / 1000.0f, -20, 20);
        wl_cmd = constrain(raw_wl, -12.566f, 12.566f);
        wr_cmd = constrain(raw_wr, -12.566f, 12.566f);
      }
      bufferRX = "";
    } else {
      bufferRX += c;
    }
  }

  // >>> 2. Si el timer ha disparado, aplicar actuadores
  if (aplicarControl) {
    aplicarControl = false;
    aplicarActuadores();
  }

  // >>> 3. Máquina de estados de sensores (se lee un sensor por iteración)
  unsigned long now = millis();
  if (now - lastSensorTime >= SENSOR_INTERVAL / 5) {  // Reparte en ~5 pasos para cubrir los 100 ms
    lastSensorTime = now;                             // Se usa temporizadores independientes para cada paso
    switch (sensorStep) {
      case SensorStep::TOF:
        {
          VL53L0X_RangingMeasurementData_t measure;
          lox.rangingTest(&measure, false);
          distanciaTOF = (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 1000.0f : -1.0f;
          sensorStep = SensorStep::SONAR1;
          break;
        }
      case SensorStep::SONAR1:
        dist1 = prizm.getSonicSensor(5, UNITS_CM);
        sensorStep = SensorStep::SONAR2;
        break;
      case SensorStep::SONAR2:
        dist2 = prizm.getSonicSensor(6, UNITS_CM);
        sensorStep = SensorStep::ENCODERS;
        break;
      case SensorStep::ENCODERS:
        Encoder1 = prizm.readEncoderDegrees(1);
        Encoder2 = prizm.readEncoderDegrees(2);
        sensorStep = SensorStep::SEND;
        break;
      case SensorStep::SEND:
        // Enviar trama completa
        Serial1.print('A');
        Serial1.print(dist1 * 1000.0f, 3);
        Serial1.print('B');
        Serial1.print(dist2 * 1000.0f, 3);
        Serial1.print('C');
        Serial1.print(distanciaTOF * 1000.0f, 3);
        Serial1.print('D');
        Serial1.print(Encoder1 * 1000.0f, 0);
        Serial1.print('E');
        Serial1.print(Encoder2 * 1000.0f, 0);
        Serial1.print('F');
        Serial1.print("\r\n");

        sensorStep = SensorStep::TOF;  // Reiniciar ciclo
        break;
    }
  }
}
