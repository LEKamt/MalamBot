// Código para leer ToF y desplazar el riel con el objetivo de tomar data
// para modelar la planta y controlar posición del riel

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PRIZM_PRO.h>

PRIZM prizm;
Adafruit_VL53L0X lox;

const uint32_t Ts = 10;  // 10 ms
uint32_t t_anterior = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(16, 15);

  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("Fallo VL53L0X");
    while (1)
      ;
  }

  Serial.println("VL53L0X OK");
  Serial.println("Tiempo_ms, Potencia, Distancia_mm");
  prizm.PrizmBegin();
  prizm.setMotorPower(3, 0);
  delay(100);
}

void loop() {

  if (millis() - t_anterior >= Ts) {

    t_anterior += Ts;

    int potencia = -10;

    prizm.setMotorPower(3, potencia);

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    uint32_t tiempo = millis();

    if (measure.RangeStatus != 4) {

      Serial.print(tiempo);
      Serial.print(",");

      Serial.print(potencia);
      Serial.print(",");

      Serial.println(measure.RangeMilliMeter);

    } else {

      Serial.print(tiempo);
      Serial.print(",");

      Serial.print(potencia);
      Serial.println(",-1");
    }
  }
}