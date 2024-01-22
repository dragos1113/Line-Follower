#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed;
int m2Speed;
int baseSpeed = 255;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];

struct PID
{
  float kp;
  float ki;
  float kd;

  float error;
  float derivative;
  float integral;

  unsigned long lastCall;
  PID(float _kp, float _ki, float _kd)
  {
    kp = _kp;
    ki = _ki;
    kd = _kd;

    error = 0;
    integral = 0;
    derivative = 0;

    lastCall = 0ul;
  }

  float pass(float newError)
  {
    unsigned long now = millis();

    float delta = (now - lastCall) / 1000.0f;

    integral += newError * delta;
    derivative = (newError - error) / delta;
    error = newError;

    lastCall = now;

    return kp * error + ki * integral + kd * derivative;
  }
};

PID pid(-500, 0, -50);

void setup()
{
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) { A0, A1, A2, A3, A4, A5 }, sensorCount);

  calibrate();
}

void loop()
{
  float error = getError();

  float val = pid.pass(error);

  changeSpeed(val);
}

void calibrate()
{
  for (int i = 0; i < 10; i++)
  {
    setMotorSpeed(255, -255);
    delay(30);
    setMotorSpeed(0, 0);
    delay(100);

    qtr.calibrate();
  }

  delay(200);

  for (int i = 0; i < 10; i++)
  {
    setMotorSpeed(-255, 255);
    delay(30);
    setMotorSpeed(0, 0);
    delay(100);

    qtr.calibrate();
  }

  delay(200);

  for (int i = 0; i < 10; i++)
  {
    setMotorSpeed(-255, 255);
    delay(30);
    setMotorSpeed(0, 0);
    delay(100);

    qtr.calibrate();
  }

  delay(200);

  for (int i = 0; i < 10; i++)
  {
    setMotorSpeed(255, -255);
    delay(30);
    setMotorSpeed(0, 0);
    delay(100);

    qtr.calibrate();
  }
}

void setMotorSpeed(int speed1, int speed2)
{
  if (speed1 == 0)
  {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, speed1);
  }
  else if (speed1 > 0)
  {
    digitalWrite(m11Pin, HIGH);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, speed1);
  }
  else
  {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, HIGH);
    analogWrite(m1Enable, -speed1);
  }

  if (speed2 == 0)
  {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, speed2);
  }
  else if (speed2 > 0)
  {
    digitalWrite(m21Pin, HIGH);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, speed2);
  }
  else
  {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, HIGH);
    analogWrite(m2Enable, -speed2);
  }
}

float getError()
{
  int read = qtr.readLineBlack(sensorValues) - 2500;

  return read / 2500.0f;
}

void changeSpeed(float val)
{
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  m1Speed += val;
  m2Speed -= val;

  m1Speed = constrain(m1Speed, -255, 255);
  m2Speed = constrain(m2Speed, -255, 255);

  setMotorSpeed(m1Speed, m2Speed);
}
