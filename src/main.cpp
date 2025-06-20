// codigo simples para teste dos modulos TCRT5000, QRT-8A e motores do seguidor de linha

#include "HardwareSerial.h"
#include <Arduino.h>
#include <DC-Motor-Arduino.h>
#include <QTRSensors.h>

enum testType { QRT, TRT, MOTOR };
testType teste = QRT;

// motor pins
#define PWM1 3
#define M1_PIN1 5
#define M1_PIN2 6
#define PWM2 9
#define M2_PIN1 10
#define M2_PIN2 11

// sensors pins
#define TCRT_D 2
#define TCRT_E 4
#define QRT_ON_PIN 7 // if used, the qrt emmiter is only turned on when reading
const uint8_t qrt_pins[] { A0, A1, A2, A3, A4, A5, A6, A7 };

QTRSensors qtr;
uint16_t sensors[8];

DC_Motor m1(PWM1, M1_PIN1, M1_PIN2);
DC_Motor m2(PWM2, M2_PIN2, M2_PIN2);

void testa_tcrt();
void testa_qrt();
void testa_motores();

void setup()
{
    Serial.begin(9600);
    pinMode(TCRT_D, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize the sensors.
    qtr.setTypeAnalog();
    qtr.setSensorPins(qrt_pins, sizeof(qrt_pins));
    qtr.setEmitterPin(QRT_ON_PIN);
}

void loop()
{
    switch (teste) {
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

// testa motores andando para frente, para traz, para esquerda e para a direita
void testa_motores()
{
    uint8_t speed = 255;
    // forward
    m1.forward(speed);
    m2.forward(speed);
    delay(400);

    // backwards
    m1.forward(-speed);
    m2.forward(-speed);
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

// testa sensor TCRT5000
// ao dectectar linha branca liga o pino do buzzer
void testa_tcrt()
{
    int result1 = digitalRead(TCRT_E);
    Serial.print("Left Sensor: ");
    Serial.print(result1);
    Serial.print("\n");

    int result2 = digitalRead(TCRT_D);
    Serial.print("Right Sensor: ");
    Serial.print(result2);
    Serial.print("\n");

    if ((result1 == HIGH) || (result2 == HIGH)) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
    }

    digitalWrite(LED_BUILTIN, LOW);
}

// testa sensor QRT-8A
//  le sensores e escreve na saida serial
void testa_qrt()
{
    qtr.readLineWhite(sensors);

    for (int i = 0; i < 8; i++) {
        Serial.print(sensors[i]);
        Serial.print(" ");
    }
    Serial.print("\n");
}
