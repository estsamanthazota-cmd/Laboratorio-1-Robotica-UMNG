#include <Arduino.h>

/* =======================
   DEFINICIÓN DE PINES
======================= */
// 🔵 MOTOR 1
#define ENC1_A     34
#define ENC1_B     35
#define PWM1       27
#define DIR1_1     14
#define DIR2_1     12 

// 🟢 MOTOR 2
#define ENC2_A     25
#define ENC2_B     26
#define PWM2       21
#define DIR1_2     18
#define DIR2_2     19

// 🔴 MOTOR 3 
#define ENC3_A     32
#define ENC3_B     33
#define PWM3       13
#define DIR1_3     22
#define DIR2_3     23

#define PIN_TIMER_TEST 2 

/* =======================
   PARÁMETROS
======================= */
const float PPR = 588.0 * 2.0; 
const float U_SAT = 1.0;

/* =======================
   CONTROLADOR
======================= */
float q0 = 0.3613;
float q1 = -0.6839;
float q2 = 0.3235;
float s0 = 0.7741;

/* =======================
   VARIABLES MOTOR 1
======================= */
float e1=0,e1_1=0,e1_2=0;
float u1=0,u1_1=0,u1_2=0;
volatile long count1 = 0;
float ref1 = 0;

/* =======================
   VARIABLES MOTOR 2
======================= */
float e2=0,e2_1=0,e2_2=0;
float u2=0,u2_1=0,u2_2=0;
volatile long count2 = 0;
float ref2 = 0;

/* =======================
   VARIABLES MOTOR 3
======================= */
float e3=0,e3_1=0,e3_2=0;
float u3=0,u3_1=0,u3_2=0;
volatile long count3 = 0;
float ref3 = 0;

/* =======================
   TIMER
======================= */
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* =======================
   ENCODERS
======================= */
void IRAM_ATTR enc1_isr() {
  if (digitalRead(ENC1_B)) count1++;
  else count1--;
}

void IRAM_ATTR enc2_isr() {
  if (digitalRead(ENC2_B)) count2++;
  else count2--;
}

void IRAM_ATTR enc3_isr() {
  if (digitalRead(ENC3_B)) count3++;
  else count3--;
}

/* =======================
   TIMER ISR
======================= */
void ARDUINO_ISR_ATTR onTimer() {
  digitalWrite(PIN_TIMER_TEST, !digitalRead(PIN_TIMER_TEST));
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

/* =======================
   CONTROL MOTOR
======================= */
void controlMotor(float u, int pwm, int d1, int d2){

  if (u > U_SAT) u = U_SAT;
  if (u < -U_SAT) u = -U_SAT;

  int duty = fabs(u)*255;
  analogWrite(pwm,duty);

  if(u>=0){
    digitalWrite(d1,HIGH);
    digitalWrite(d2,LOW);
  } else {
    digitalWrite(d1,LOW);
    digitalWrite(d2,HIGH);
  }
}

/* =======================
   SERIAL (3 REFERENCIAS)
======================= */
// Formato: 90,180,45
void checkSerial() {
  if (Serial.available() > 0) {

    ref1 = Serial.parseFloat();
    ref2 = Serial.parseFloat();
    ref3 = Serial.parseFloat();

    while (Serial.available() > 0) Serial.read();
  }
}

/* =======================
   SETUP
======================= */
void setup() {

  Serial.begin(115200);
  Serial.setTimeout(5);

  // Encoders
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, RISING);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, RISING);

  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3_isr, RISING);

  // Motores
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1_1, OUTPUT);
  pinMode(DIR2_1, OUTPUT);

  pinMode(PWM2, OUTPUT);
  pinMode(DIR1_2, OUTPUT);
  pinMode(DIR2_2, OUTPUT);

  pinMode(PWM3, OUTPUT);
  pinMode(DIR1_3, OUTPUT);
  pinMode(DIR2_3, OUTPUT);

  pinMode(PIN_TIMER_TEST, OUTPUT);

  timerSemaphore = xSemaphoreCreateBinary();

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 20000, true, 0);
}

/* =======================
   LOOP
======================= */
void loop() {

  checkSerial();

  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {

    long c1, c2, c3;

    portENTER_CRITICAL(&mux);
    c1 = count1;
    c2 = count2;
    c3 = count3;
    portEXIT_CRITICAL(&mux);

    // POSICIÓN
    float th1 = (c1 / PPR) * 360.0;
    float th2 = (c2 / PPR) * 360.0;
    float th3 = (c3 / PPR) * 360.0;

    // ERRORES
    e1 = ref1 - th1;
    e2 = ref2 - th2;
    e3 = ref3 - th3;

    // Limitar
    if(e1>180) e1=180; if(e1<-180) e1=-180;
    if(e2>180) e2=180; if(e2<-180) e2=-180;
    if(e3>180) e3=180; if(e3<-180) e3=-180;

    // CONTROL
    u1 = q0*e1 + q1*e1_1 + q2*e1_2 - (s0-1)*u1_1 + s0*u1_2;
    u2 = q0*e2 + q1*e2_1 + q2*e2_2 - (s0-1)*u2_1 + s0*u2_2;
    u3 = q0*e3 + q1*e3_1 + q2*e3_2 - (s0-1)*u3_1 + s0*u3_2;

    controlMotor(u1, PWM1, DIR1_1, DIR2_1);
    controlMotor(u2, PWM2, DIR1_2, DIR2_2);
    controlMotor(u3, PWM3, DIR1_3, DIR2_3);

    // HISTÓRICOS
    e1_2=e1_1; e1_1=e1;
    e2_2=e2_1; e2_1=e2;
    e3_2=e3_1; e3_1=e3;

    u1_2=u1_1; u1_1=u1;
    u2_2=u2_1; u2_1=u2;
    u3_2=u3_1; u3_1=u3;

    
    Serial.print(ref1); Serial.print(",");
    Serial.print(th1); Serial.print(",");
    Serial.print(ref2); Serial.print(",");
    Serial.print(th2); Serial.print(",");
    Serial.print(ref3); Serial.print(",");
    Serial.print(th3); Serial.print(",");
    Serial.println(u1);
  }
}