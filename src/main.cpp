#include <Arduino.h>
#include <Encoder.h>

#define buttonClockwise 10
#define buttonCounterclockwise 11
#define button45Degrees 12
#define button90Degrees 13

const int EnablePin = 9;       // L298N ENA
const int In1Pin = 7;          // L298N IN1
const int In2Pin = 8;          // L298N IN2
const int EncoderAPin = 3;     // Encoder Channel A
const int EncoderBPin = 2;     // Encoder Channel B

Encoder myEncoder(EncoderAPin, EncoderBPin);

long previousMillis = 0;
long currentMillis = 0;

volatile long currentEncoderValue = 0;
volatile long previousEncoderValue = 0;
volatile long rnEncoderValue = 0;
int desiredEncoderValue;

int rotation = 0;
float previousRotationSpeed = 0;
int lastStateA;
int encoderValue = 0;
int motorSpeed = 0;
bool clockwise = true;

void setup() {
  pinMode(In1Pin, OUTPUT);
  pinMode(In2Pin, OUTPUT);
  pinMode(EncoderAPin, INPUT_PULLUP);
  pinMode(EncoderBPin, INPUT_PULLUP);
  //input button
  pinMode(buttonClockwise, INPUT);
  pinMode(buttonCounterclockwise, INPUT);
  pinMode(button90Degrees, INPUT);
  pinMode(button45Degrees, INPUT); 

  lastStateA = digitalRead(EncoderAPin);
  Serial.begin(9600);
  Serial.print("Please press the button");
}

float updateEncoder(void) {
  currentEncoderValue = myEncoder.read();
  rnEncoderValue = myEncoder.read();
  const int encoderCountsPerRevolution = 3393; 

  float rotationSpeed;
  const int updateInterval = 1000;
  currentMillis = millis();

  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;
    rotationSpeed = (float)((currentEncoderValue - previousEncoderValue) * 60 / encoderCountsPerRevolution);
    previousEncoderValue = currentEncoderValue;
    return rotationSpeed;
  }
  delay(100);
}

void changeDegree() {
  //rotasi full 3393 steps = 360 degrees
  if (digitalRead(button45Degrees) == HIGH) {
    // rotasi 45 degrees = 3393/8 = 424 steps
    //debounce to 250 steps
    if (clockwise) {
      desiredEncoderValue = rnEncoderValue + 250;
    }
    else {
      desiredEncoderValue = rnEncoderValue - 250;
    }
    Serial.print("45 degrees ");
    delay(1000);
  }
  else if (digitalRead(button90Degrees) == HIGH) {
    //rotasi 90 degrees = 3393/4 = 848 steps
    //debounce to 700 steps
    if (clockwise) {
      desiredEncoderValue = rnEncoderValue + 700;
    }
    else {
      desiredEncoderValue = rnEncoderValue - 700;
    }
    Serial.print("90 degrees ");
    delay(1000);
  }
  else if (digitalRead(buttonClockwise) == HIGH) {
    motorSpeed = 0;
    Serial.print("Clockwise ");
    clockwise = true;
    digitalWrite(In1Pin, LOW);
    digitalWrite(In2Pin, HIGH);
    delay(1000);
  }
  else if (digitalRead(buttonCounterclockwise) == HIGH) {
    motorSpeed = 0;
    Serial.print("Counterclockwise ");
    clockwise = false;
    digitalWrite(In1Pin, HIGH);
    digitalWrite(In2Pin, LOW);
    delay(1000);
  }
}

void loop() {
  analogWrite(EnablePin, motorSpeed);
  float newRotationSpeed = updateEncoder();
  changeDegree();
  currentEncoderValue = abs(currentEncoderValue);

  if (newRotationSpeed != previousRotationSpeed) {
    Serial.print("Rotation Speed: ");
    Serial.print(newRotationSpeed);
    Serial.println(" RPM");
    previousRotationSpeed = newRotationSpeed;
  }

  if (clockwise) {
    if (rnEncoderValue <= desiredEncoderValue) {
      digitalWrite(In1Pin, LOW);
      digitalWrite(In2Pin, HIGH);
      motorSpeed = 128;
    }
    else {
      motorSpeed = 0;
    }
  }
  else if (!clockwise) {
    if (rnEncoderValue >= desiredEncoderValue) {
      digitalWrite(In1Pin, HIGH);
      digitalWrite(In2Pin, LOW);
      motorSpeed = 128;
    }
    else {
      motorSpeed = 0;
    }
  }
}
