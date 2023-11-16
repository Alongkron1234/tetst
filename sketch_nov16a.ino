#include <Arduino.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int E1 = 10;
int M1 = 12;
int E2 = 11;
int M2 = 13;

int sensorA = 0;
int sensorB = 0;
int sensorC = 0;

int buzzer = 4;

int step = 0;

void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(buzzer, OUTPUT);
}

void stop() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 0);  // PWM
  analogWrite(E2, 0);  // PWM
}

void forward() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 100);  // PWM
  analogWrite(E2, 100);  // PWM
}

void backward() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, 125);  // PWM
  analogWrite(E2, 125);  // PWM
}

void left() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 255);  // PWM
  analogWrite(E2, 255);  // PWM
}

void right() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, 255);  // PWM
  analogWrite(E2, 255);  // PWM
}

void loop() {
  digitalWrite(4, LOW);

  int analogA = analogRead(A4);
  int analogB = analogRead(A3);
  int analogC = analogRead(A5);

  Serial.println(analogC);

  if(analogA > 700) {
    sensorA = 1;
  } else {
    sensorA = 0;
  }

  if(analogB > 700) {
    sensorB = 1;
  } else {
    sensorB = 0;
  }

  if(analogC > 700) {
    sensorC = 1;
  } else {
    sensorC = 0;
  }

  // Serial.println(sensorC);

  // Serial.println(step);

  int randdomSpeed = random(2);

  // Serial.println(random(2));

  if (sensorC == 1) {
    if (sensorB == 0 & sensorA == 1) {
      left();
    } else if (sensorB == 1 & sensorA == 0) {
      right();
    } else {
      forward();
    }
  } else {
    if(analogC >= 300 & analogC <= 435) {
      digitalWrite(4, 0x1);
    } else {
      digitalWrite(4, 0x0);
    }
    if (step < 3) {
      step++;
      stop();
      backward();
      delay(310);
      stop();
      right();
      delay(310);
    } else if (step < 4) {
      step++;
      stop();
      backward();
      delay(310);
      stop();
      left();
      delay(310);
    } else if (step < 5) {
      step++;
      stop();
      backward();
      delay(1000);
      stop();
      left();
      delay(300);
    } else if (step < 6) {
      step++;
      stop();
      backward();
      delay(300);
      left();
      delay(300);
      forward();
      delay(500);
      right();
    } else if(step >= 6) {
      step++;
      stop();
      backward();
      delay(300);
      if(randdomSpeed == 0) {
        stop();
        right();
        delay(300);
      } else {
        stop();
        left();
        delay(300);
      }
    }
  }
}
