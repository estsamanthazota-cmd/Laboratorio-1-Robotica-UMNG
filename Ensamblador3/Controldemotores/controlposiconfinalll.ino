#include <Arduino.h>

const float PPR = 7100; 
const float Ts = 0.05;
const float U_SAT = 1.0;

float q0 = 0.2;//Ya me asegura el ess=0
float q1 = -0.035;
float q2 = 0.00001;
float s0 = - 0.00001;

struct Motor {
  int ENC_A, ENC_B, PWM, DIR1, DIR2;
  volatile long encoderCount;

  float theta;   // 🔥 posición en grados

  float e_k, e_k1, e_k2;
  float u_k, u_k1;

  float referencia; // 🔥 ahora en grados
};

// MOTORES
Motor m1 = {34, 35, 27, 14, 12, 0, 0, 0, 0, 0, 0, 0.0};
Motor m2 = {25, 26, 21, 18, 19, 0, 0, 0, 0, 0, 0, 0.0};
Motor m3 = {32, 33, 13, 23, 22, 0, 0, 0, 0, 0, 0, 0.0};

hw_timer_t *timer = NULL;
SemaphoreHandle_t timerSemaphore;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* =======================
   ENCODERS
======================= */
void IRAM_ATTR enc1() { 
  if (digitalRead(m1.ENC_B)) m1.encoderCount++; 
  else m1.encoderCount--; 
}

void IRAM_ATTR enc2() { 
  if (digitalRead(m2.ENC_B)) m2.encoderCount++; 
  else m2.encoderCount--; 
}

void IRAM_ATTR enc3() { 
  if (digitalRead(m3.ENC_B)) m3.encoderCount++; 
  else m3.encoderCount--; 
}

/* =======================
   TIMER
======================= */
void IRAM_ATTR onTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

/* =======================
   CONTROL POSICIÓN
======================= */
void controlMotor(Motor &m) {

  long count;

  portENTER_CRITICAL(&mux);
  count = m.encoderCount;
  portEXIT_CRITICAL(&mux);

// 🔥 POSICIÓN EN GRADOS (-180 a 180)
m.theta = (count * 360.0) / PPR;

while (m.theta > 180.0)  m.theta -= 360.0;
while (m.theta < -180.0) m.theta += 360.0;
  // 🔥 ERROR DE POSICIÓN
  m.e_k = m.referencia - m.theta;

  // 🔥 ECUACIÓN EN DIFERENCIAS
  m.u_k = (-s0 * m.u_k1)
        + (q0 * m.e_k)
        + (q1 * m.e_k1)
        + (q2 * m.e_k2);

  // Saturación
  if (m.u_k > U_SAT) m.u_k = U_SAT;
  if (m.u_k < -U_SAT) m.u_k = -U_SAT;

  // PWM
  analogWrite(m.PWM, (int)(fabs(m.u_k) * 255));
  digitalWrite(m.DIR1, (m.u_k >= 0));
  digitalWrite(m.DIR2, !(m.u_k >= 0));

  // 🔄 actualizar estados
  m.u_k1 = m.u_k;

  m.e_k2 = m.e_k1;
  m.e_k1 = m.e_k;
}

/* =======================
   SERIAL
======================= */
void checkSerial() {
  if (Serial.available()) {

    float ref1 = Serial.parseFloat();
    float ref2 = Serial.parseFloat();
    float ref3 = Serial.parseFloat();

    m1.referencia = ref1;
    m2.referencia = ref2;
    m3.referencia = ref3;

    while (Serial.available()) Serial.read();

    Serial.print("Refs: ");
    Serial.print(ref1); Serial.print(",");
    Serial.print(ref2); Serial.print(",");
    Serial.println(ref3);
  }
}
/* =======================
   SETUP
======================= */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  Motor* grupo[] = {&m1, &m2, &m3};

  for(int i=0; i<3; i++) {
    pinMode(grupo[i]->ENC_A, INPUT_PULLUP);
    pinMode(grupo[i]->ENC_B, INPUT_PULLUP);
    pinMode(grupo[i]->PWM, OUTPUT);
    pinMode(grupo[i]->DIR1, OUTPUT);
    pinMode(grupo[i]->DIR2, OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(m1.ENC_A), enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(m2.ENC_A), enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(m3.ENC_A), enc3, RISING);

  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 60000, true, 0); 
}

/* =======================
   LOOP
======================= */
void loop() {
  checkSerial();

  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY)) {
    controlMotor(m1);
    controlMotor(m2);
    controlMotor(m3);
    
Serial.print(m1.referencia); Serial.print(",");
Serial.print(m1.theta); Serial.print(",");

Serial.print(m2.referencia); Serial.print(",");
Serial.print(m2.theta); Serial.print(",");

Serial.print(m3.referencia); Serial.print(",");
Serial.println(m3.theta);
  }
}
