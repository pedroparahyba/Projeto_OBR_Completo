#include <Adafruit_PWMServoDriver.h>
#include <QTRSensors.h>
#include <stdlib.h>
#include <Wire.h>
#include <cmath>

#define QTR0 15
#define QTR1 16
#define QTR2 21
#define QTR3 38
#define QTR4 39
#define QTR5 40
#define QTR6 41
#define QTR7 42

#define SDA 47
#define SCL 48

int FME_IN1 = 0;
int FME_IN2 = 1;
int FMD_IN1 = 2;
int FMD_IN2 = 3;

int TME_IN1 = 4;
int TME_IN2 = 5;
int TMD_IN1 = 6;
int TMD_IN2 = 7;

QTRSensors qtr;
const uint8_t Count = 8;
uint16_t Values[Count];
const uint8_t Pins[Count] = {QTR0,QTR1,QTR2,QTR3,QTR4,QTR5,QTR6,QTR7};

Adafruit_PWMServoDriver pca(0x40);

int PWM_Limit = 255;

float P = 1.2;
float I = 0.0;
float D = 0.6;
float PID = 0.0;

int erro = 0;
int ultimo_erro = 0;
int integral = 0;
int derivado = 0;
int RS = 0;
int LS = 0;
int velocidade_inicial = 105;

int joselino = 0;

void setup() {
  Serial.begin(115200);

  qtr.setTypeRC();
  qtr.setSensorPins(Pins, Count);
  
  Wire.begin(SDA, SCL);
  pca.begin();
  pca.setPWMFreq(1000);

  for (uint16_t i = 0; i < 10; i++) {
    qtr.calibrate();
  }
  
  for (uint8_t i = 0; i < Count; i++) {
    qtr.calibrationOn.minimum[i] = 400;
    qtr.calibrationOn.maximum[i] = 3500;
  }
}

void ePWM(uint8_t canal, uint8_t valor) {
  uint16_t pwm = (uint32_t)valor * 4095 / 255;
  pca.setPin(canal, pwm);
}

void fPID(){
  uint16_t position = qtr.readLineBlack(Values);

  erro = position - 3500;

  if (abs(erro) < 500) {
    integral = integral + erro;
  } else {
    integral = 0;
  }

  derivado = erro - ultimo_erro;

  PID = ((P*erro) + (I*integral) + (D*derivado));
  ultimo_erro = erro;
  LS = static_cast<int>(std::round(velocidade_inicial + PID));
  RS = static_cast<int>(std::round(velocidade_inicial - PID));
}

void RLmotors(int R, int L){
  if (L > PWM_Limit)  L = PWM_Limit;
  if (L < -PWM_Limit) L = -PWM_Limit;
  if (R > PWM_Limit)  R = PWM_Limit;
  if (R < -PWM_Limit) R = -PWM_Limit;

  if (L >= 0) {
    ePWM(FME_IN1, L);
    ePWM(FME_IN2, 0);
    ePWM(TME_IN1, L);
    ePWM(TME_IN2, 0);

  } else if (L < 0) {
    joselino = L;
    L = abs(joselino);
    ePWM(FME_IN1, 0);
    ePWM(FME_IN2, L);
    ePWM(TME_IN1, 0);
    ePWM(TME_IN2, L);
  }

  if (R >= 0) {
    ePWM(FMD_IN1, R);
    ePWM(FMD_IN2, 0);
    ePWM(TMD_IN1, R);
    ePWM(TMD_IN2, 0);

  } else if (R < 0) {
    joselino = R;
    R = abs(joselino);
    ePWM(FMD_IN1, 0);
    ePWM(FMD_IN2, R);
    ePWM(TMD_IN1, 0);
    ePWM(TMD_IN2, R);
  }
}

void loop() {
  fPID();
  RLmotors(RS, LS);
}