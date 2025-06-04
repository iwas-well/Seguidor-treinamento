#include "HardwareSerial.h"
#include <Arduino.h>
#include <DC-Motor-Arduino.h>
#include <QTRSensors.h>

// Código para Teste de Buzzer com base na litura do sensor lateral TCRT5000, se
// linha branca um bip por 200ms. Código teste para motores: Indo pra frent;
// fazendo curva a esquerda, curva a direita, motores para trás. Fazer a leitura
// e impressão no serial do ensor frontal QTR-8A, leitura analogica ou digital.

#define pinTct 10
#define TESTE 1

QTRSensors qtr;
uint16_t sensors[8];

void testa_trt();
void testa_qrt();
void testa_motores();

void setup() {
  Serial.begin(9600);
  pinMode(pinTct, INPUT);

  // Initialize the sensors.
  // In this example we have three sensors on pins A0 - A2.
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);
}

void loop() {
  switch (TESTE) {
  case 1:
    testa_qrt();
    break;
  case 2:
    testa_trt();
    break;
  case 3:
    testa_motores();
    break;
  default:
    break;
  }

  delay(50);
}
void testa_motores() {}

void testa_trt() {
  int result = digitalRead(pinTct);
  Serial.print(result);
}

void testa_qrt() {
  qtr.readLineWhite(sensors);

  for (int i = 0; i < 8; i++) {
    Serial.print(sensors[i]);
    Serial.print(" ");
  }

  Serial.print("\n");
}
