#include <Arduino.h>
#include <Servo.h>

// Ultrasonic Sensor
#define LEFT_TRIG_PIN 48
#define LEFT_ECHO_PIN 49
#define RIGHT_TRIG_PIN 52
#define RIGHT_ECHO_PIN 53
#define FRONT_TRIG_PIN 51
#define FRONT_ECHO_PIN 50
#define TARGET_DISTANCE_FRONT 30
#define TARGET_DISTANCE_LEFT 10
#define TARGET_DISTANCE_RIGHT 10

// Motors
#define MOTOR_DIR 4
#define MOTOR_PWM 5

// Servo
#define SERVO_PIN 9
#define NEUTRAL_ANGLE 70
#define FORWARD_SPEED 150

Servo servo_motor;
bool isLeftLane = true;
unsigned long startTime;

// Encoder
volatile long encoder0Pos = 0;
int encoder0PinA = 2;
int encoder0PinB = 3;
int encoder0PinALast = LOW;
int n = LOW;

void encoderInterruptLeft() {
    if (digitalRead(encoder0PinA) == LOW) {
        if (digitalRead(encoder0PinB) == HIGH) {
            encoder0Pos--;
        } else {
            encoder0Pos++;
        }
    } else {
        if (digitalRead(encoder0PinB) == LOW) {
            encoder0Pos--;
        } else {
            encoder0Pos++;
        }
    }
}

void setup() {
    pinMode(LEFT_TRIG_PIN, OUTPUT);
    pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);
    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoder0PinA), encoderInterruptLeft, RISING);

    servo_motor.attach(SERVO_PIN);
    servo_motor.write(NEUTRAL_ANGLE);
    Serial.begin(115200);
    startTime = millis();
}

void loop() {
    int left_distance = getDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    int right_distance = getDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    int front_distance = getDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);

    if (front_distance > 0 && front_distance <= TARGET_DISTANCE_FRONT) {
        turnRightAndGoStraight();
    } else {
        if (isLeftLane) {
            followLeftWall(left_distance);
        } else {
            followRightWall(right_distance);
        }
    }

    moveForward(MOTOR_PWM, FORWARD_SPEED, encoder0Pos, 6000);
    delay(50);

    // Encoder 값 출력
    Serial.print("Encoder Pos: ");
    Serial.println(encoder0Pos);

    // 엔코더 값이 6000을 넘어가면 동작 멈춤
    if (encoder0Pos >= 6000) {
        stopMovement();
    }
}

int getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    return distance;
}

void followLeftWall(int leftDistance) {
    int leftDistanceError = TARGET_DISTANCE_LEFT - leftDistance;
    int servo_angle = NEUTRAL_ANGLE + leftDistanceError * 3;
    servo_angle = constrain(servo_angle, 60, 800);
    servo_motor.write(servo_angle);
}

void followRightWall(int rightDistance) {
    int rightDistanceError = TARGET_DISTANCE_RIGHT - rightDistance;
    int servo_angle = NEUTRAL_ANGLE - rightDistanceError * 10;
    servo_angle = constrain(servo_angle, -250, 80);
    servo_motor.write(servo_angle);
}

void changeToRightLane() {
    isLeftLane = !isLeftLane;
}

void turnRightAndGoStraight() {
    servo_motor.write(100); // 우회전 각도 지정
    delay(400);
    servo_motor.write(NEUTRAL_ANGLE); // 직진 각도 지정
    delay(1000);
}

void moveForward(int motorPin, int speed_value, long current_encoder_pos, long stop_position) {
    if (current_encoder_pos >= stop_position) {
        analogWrite(motorPin, 0);
    } else {
        digitalWrite(MOTOR_DIR, HIGH);
        analogWrite(motorPin, speed_value);
    }
}

void stopMovement() {
    analogWrite(MOTOR_PWM, 0);
}
