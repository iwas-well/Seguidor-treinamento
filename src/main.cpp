// codigo simples para teste dos modulos TCRT5000, QTR-8A e motores do seguidor de linha
#include "HardwareSerial.h"
#include <Arduino.h>
#include <DC-Motor-Arduino.h>
#include <QTRSensors.h>

enum testType { QTR, TRT, MOTOR };
testType teste = TRT;

#define BUZZER_PIN A0

// motor pins
#define M1_PIN1 A4
#define M1_PIN2 A5
#define PWM1 A6
#define M2_PIN1 A3
#define M2_PIN2 A2
#define PWM2 A1

// sensors pins
#define TCRT_D 2
#define TCRT_E 13
#define QTR_ON_PIN 11 // if used, the qtr emmiter is only turned on when reading
const uint8_t qtr_pins[] { 10, 9, 8, 7, 6, 5, 4, 3 };

QTRSensors qtr;
uint16_t sensors[8];

DC_Motor m1(PWM1, M1_PIN1, M1_PIN2);
DC_Motor m2(PWM2, M2_PIN1, M2_PIN2);

void testa_tcrt();
void testa_qtr();
void testa_motores();

void setup()
{
    Serial.begin(9600);
    pinMode(TCRT_E, INPUT);
    pinMode(TCRT_D, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Initialize the sensors.
    // qtr.setTypeAnalog();
    qtr.setTypeRC();
    qtr.setSensorPins(qtr_pins, sizeof(qtr_pins));
    qtr.setEmitterPin(QTR_ON_PIN);
}

void loop()
{
    switch (teste) {
    case QTR:
        testa_qtr();
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
// em intervalos de 3 segundos
void testa_motores()
{
    uint8_t speed = 50;

    // forward
    m1.forward(speed);
    m2.forward(speed);
    delay(3000);

    // backwards
    m1.forward(-speed);
    m2.forward(-speed);
    delay(3000);

    // left
    m1.forward(-speed);
    m2.forward(speed);
    delay(3000);

    // right
    m1.forward(speed);
    m2.forward(-speed);
    delay(3000);

    m1.stop();
    m2.stop();
    delay(3000);
}

// testa sensor TCRT5000
// ao dectectar linha branca liga o pino do buzzer
void testa_tcrt()
{
    int result1 = digitalRead(TCRT_E);
    Serial.print("Sensor Esquerdo: ");
    Serial.print(result1);
    Serial.print("\n");

    int result2 = digitalRead(TCRT_D);
    Serial.print("Sensor Direito: ");
    Serial.print(result2);
    Serial.print("\n");

    if ((result1 == LOW) || (result2 == LOW)) {
        // if (result2 == LOW) {
        Serial.print("Buzz on\n");
        tone(BUZZER_PIN, 200); // beep at 2 kHz
        delay(200);
        noTone(BUZZER_PIN);
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        noTone(BUZZER_PIN);
        digitalWrite(LED_BUILTIN, LOW);
    }
    delay(800);
}

// testa sensor QTR-8A
//  le sensores e escreve na saida serial
void testa_qtr()
{
    delay(1500);
    // calibrate sensor
    tone(BUZZER_PIN, 400); // beep at 2 kHz
    delay(400);
    noTone(BUZZER_PIN);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("calibrating\n");
    for (int i = 0; i < 500; i++)
        qtr.calibrate();

    digitalWrite(LED_BUILTIN, LOW);
    tone(BUZZER_PIN, 200); // beep at 2 kHz
    delay(400);
    noTone(BUZZER_PIN);

    while (1) {
        int pos = qtr.readLineWhite(sensors);

        Serial.print(pos);
        Serial.print("\t");
        for (int i = 0; i < 8; i++) {
            Serial.print(sensors[i]);
            Serial.print(" ");
        }
        Serial.print("\n");
    }
}
