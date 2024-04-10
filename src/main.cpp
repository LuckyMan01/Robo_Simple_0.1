#include <AFMotor.h>
#include <Arduino.h>
#include <NewPing.h>

AF_DCMotor MotorL(1);  // Motor for drive Left on M1
AF_DCMotor MotorR(2);  // Motor for drive Right on M2

const int trigPin = A0;  // trig pin connected to Arduino's pin A0
const int echoPin = A1;  // echo pin connected to Arduino's pin A1

const int sensorPin = A2;   // Pin for IR sensor
const int threshold = 300;  // value for detecting white and black
const int minSpeed = 160;
const int maxSpeed = 220;
const long interval = 130;
const int distanceToWall = 7;

NewPing sonar(trigPin, echoPin, 400);

void stop() {
    MotorL.run(RELEASE);
    MotorR.run(RELEASE);
}

void speedMotors(int speed) {
    MotorL.setSpeed(speed);
    MotorR.setSpeed(speed);
}

void setup() {
    Serial.begin(9600);  // set up Serial library at 9600 bps
    Serial.println("SMARS Obstacle Avoidance Mod");

    pinMode(trigPin, OUTPUT);   // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);    // Sets the echoPin as an Input
    pinMode(sensorPin, INPUT);  // Sets the lineSensorPin as an Input

    speedMotors(maxSpeed);
    stop();
}

bool checkLine() {
    int sensorValue = analogRead(sensorPin);
    if (sensorValue >= threshold) {
        Serial.println("Stop");
        stop();
        return true;  // Line detected
    } else {
        Serial.println("Go");
        return false;  // No line detected
    }
}

// RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
int getDistance() {
    static unsigned long lastTurnTime = 0;
    unsigned long currentMillis = millis();
    int distanseCM = sonar.ping_cm();

//asinhroni method without delay
    if (currentMillis - lastTurnTime >= 100) {
        while (distanseCM == 0) {
            distanseCM = sonar.ping_cm();
        }
        return distanseCM;
    }
}

void run(int speed, bool direction) {
    speedMotors(speed);
    if (direction) {
        MotorL.run(BACKWARD);
        MotorR.run(BACKWARD);

    } else {
        MotorL.run(FORWARD);
        MotorR.run(FORWARD);
    }
}

void turnRight() {
    static unsigned long lastTurnTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastTurnTime >= interval) {
        MotorL.run(BACKWARD);
        MotorR.run(FORWARD);
        lastTurnTime = currentMillis;
    }
}

void loop() {
    int distanceCm = getDistance();
    Serial.println(distanceCm);
    if (!checkLine()) {
        if (distanceCm <= distanceToWall) {
            int beckwordDistanceCM = distanceCm;
            // go back if distance to wall equals 7 cm until distance by more 17cm
            while (distanceCm + 10 >= beckwordDistanceCM) {
                beckwordDistanceCM = getDistance();
                run(maxSpeed, false);
            }

            turnRight();
        } else {
            run(maxSpeed, true);
        }
    }
}
