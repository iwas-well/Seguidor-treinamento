#include <stdio.h>
#include <stdlib.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DC-Motor-Arduino.h>
#include <alastor.h>

// -- QTR Sensors declarations --
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// -- Motors declarations --
DC_Motor leftMotor(PWM_PIN_1, CW_PIN_1, CCW_PIN_1);
DC_Motor rightMotor(PWM_PIN_2, CW_PIN_2, CCW_PIN_2);
int leftMotorSpeed;
int rightMotorSpeed;

// -- PID declarations --
int error, lastError = 0;
int P, D;
float KP = 0.19, KD = 0.2;


// -- Motors velocity --
int minMotorSpeed = 55;
int maxMotorSpeed = 70;

int curveCounter = 0;
int curveSensorValue = 1;
int lastState = 1;
int stopSensorValue = 1;
int lastStateStop = 1;

// -- Timers
unsigned long curveTime = 0;
unsigned long stopReadTime = 0;
unsigned long lastStopUpdate = 0;

bool readCurveSensor = false;
bool readStopSensor = false;
bool isStopMark = false;


void setup()
{
    
    Serial.begin(9600);

    //setup motor A.
    ledcSetup(0, 500, 8);
    ledcAttachPin(PWM_PIN_1, 0);

    //setup motor B.
    ledcSetup(1, 500, 8);
    ledcAttachPin(PWM_PIN_2, 1);

    // QTR sensors setup
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){4, 16, 17, 5, 18, 19}, SENSOR_COUNT);
    qtr.setEmitterPin(21);
    

    // Setting pins
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(STOP_SENSOR, INPUT);
    pinMode(CURVE_SENSOR, INPUT);


    calibrateSensor();

    // Time to postion the robot
    
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);

    delay(2000);
    
}

void loop()
{   

    
    int input = qtr.readLineWhite(sensorValues);
    error = input - 1800;
    Serial.println(input);

    //delay(1500);
    P = error;
    D = error - lastError;
    lastError = error;

    //wouldnt 'I' in this case be Ki times the area of the triangle (0,err),(1,lerr),(1,0) plus
    //the square (0,0),(0,min(err,lerr)),(1,0),(1,1) ?
    int correction = (P * KP) + (D * KD);


    leftMotorSpeed = minMotorSpeed - correction;
    rightMotorSpeed = minMotorSpeed + correction;

    setMotors(leftMotorSpeed, rightMotorSpeed);


    curveSensorValue = digitalRead(CURVE_SENSOR);
    stopSensorValue = digitalRead(STOP_SENSOR);



    /*
    Serial.print("Curva: ");
    Serial.println(curveSensorValue);
    Serial.print("Parada: ");
    Serial.println(stopSensorValue);
    if (curveSensorValue == WHITE_LINE && lastState == BLACK_LINE)
    {
        lastState = WHITE_LINE;
        readCurveSensor = true;
        curveCounter++;
        
        // if the robot reads a curve, the buzzer will sound
        digitalWrite(BUZZER_PIN, HIGH);

        // storage the time that the robot reads a curve
        curveTime = millis();
    }

    if (curveSensorValue == BLACK_LINE && lastState == WHITE_LINE)
    {
        lastState = BLACK_LINE;
        digitalWrite(BUZZER_PIN, LOW);
    }
   
    stopSensorValue = digitalRead(STOP_SENSOR);
    if (stopSensorValue == WHITE_LINE && lastStateStop == BLACK_LINE)
    {
        readStopSensor = true;
        lastStateStop = WHITE_LINE;
    }

    if (stopSensorValue == BLACK_LINE && lastStateStop == WHITE_LINE)
        lastStateStop = BLACK_LINE;

    if (curveCounter > CURVE_COUNTER_THRESHOLD)
    {
        stopReadTime = millis();

        if (stopReadTime - curveTime < INTERVAL_TIME)
            isStopMark = false;

        if (stopReadTime - lastStopUpdate > INTERVAL_TIME && isStopMark)
            stopRobot();

        if (stopSensorValue == WHITE_LINE && stopReadTime - curveTime > INTERVAL_TIME)
        {
            lastStopUpdate = millis();
            isStopMark = true;
        }
    }
    */
    
    if (millis() > 40000 && stopSensorValue == WHITE_LINE) {
        stopRobot(); 
    }
    
    
}

void stopRobot()
{
    while (1)
    {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed)
{
    if (leftMotorSpeed > maxMotorSpeed)
        leftMotorSpeed = maxMotorSpeed;
    if (rightMotorSpeed > maxMotorSpeed)
        rightMotorSpeed = maxMotorSpeed;
    if (leftMotorSpeed < 0)
        leftMotorSpeed = 0;
    if (rightMotorSpeed < 0)
        rightMotorSpeed = 0;

    digitalWrite(CW_PIN_1, LOW);
    digitalWrite(CCW_PIN_1, HIGH);
    digitalWrite(CW_PIN_2, LOW);
    digitalWrite(CCW_PIN_2, HIGH);

    ledcWrite(1, leftMotorSpeed);
    ledcWrite(0, rightMotorSpeed);
}

void calibrateSensor()
{
    for (uint16_t i = 0; i < 150; i++)
    {
        qtr.calibrate();
        if (i == 75)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(3000);
            digitalWrite(BUZZER_PIN, LOW);
        }
        delay(10);
    }
}
