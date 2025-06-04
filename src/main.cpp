#include "HardwareSerial.h"
#include <Arduino.h>
#include <DC-Motor-Arduino.h>
#include <QTRSensors.h>

// Código para Teste de Buzzer com base na litura do sensor lateral TCRT5000, se
// linha branca um bip por 200ms.
//
// Código teste para motores: Indo pra frente, fazendo curva a esquerda, curva a
// direita, motores para trás.

enum { QRT, TRT, MOTOR };
#define TESTE QRT

#define BUZZER_PIN 11
#define TCRT_PIN 10
#define EMITER_PIN 11
const uint8_t qrt_pins[] { A0, A1, A2, A3, A4, A5, A6, A7 };

// motor pins
#define PWM1 1
#define M1_PIN1 2
#define M1_PIN2 3
#define PWM2 4
#define M2_PIN1 5
#define M2_PIN2 6

QTRSensors qtr;
uint16_t sensors[8];

void testa_tcrt();
void testa_qrt();
void testa_motores();

DC_Motor m1(PWM1, M1_PIN1, M1_PIN2);
DC_Motor m2(PWM2, M2_PIN2, M2_PIN2);

void setup()
{
    Serial.begin(9600);
    pinMode(TCRT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Initialize the sensors.
    qtr.setTypeAnalog();
    qtr.setSensorPins(qrt_pins, sizeof(qrt_pins));
    qtr.setEmitterPin(EMITER_PIN);
}

void loop()
{
    switch (TESTE) {
    case QRT:
        testa_qrt();
        break;
    case TRT:
        testa_tcrt();
        break;
    case MOTOR:
        testa_motores();
        break;
    default:
        break;
    }

    delay(50);
}

void testa_motores()
{
    uint8_t speed = 255;
    // forward
    m1.forward(speed);
    m2.forward(speed);
    delay(400);

    // left
    m1.forward(-speed);
    m2.forward(speed);
    delay(400);

    // right
    m1.forward(speed);
    m2.forward(-speed);
    delay(400);

    m1.stop();
    m2.stop();
    delay(400);
}

void testa_tcrt()
{
    int result = digitalRead(TCRT_PIN);
    Serial.print(result);

    if (result == HIGH) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
    }

    digitalWrite(BUZZER_PIN, LOW);
}

void testa_qrt()
{
    qtr.readLineWhite(sensors);

    for (int i = 0; i < 8; i++) {
        Serial.print(sensors[i]);
        Serial.print(" ");
    }

    Serial.print("\n");
}
