#include <Arduino.h>

const float PPR = 7100; 
const float Ts = 0.05;
const float U_SAT = 1.0;

float q0 =  0.0011;

float q1 = -0.0002;
float q2=;

float s0 = - 0.7;


struct Motor {
  int ENC_A, ENC_B, PWM, DIR1, DIR2;
  volatile long encoderCount;
  long lastCount;
  float rpmRaw; 
  float e_k, e_k1, u_k, u_k1, u_k2;
  float referencia;
};

// MOTOR 1
Motor m1 = {34, 35, 27, 14, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0.0};

// MOTOR 2
Motor m2 = {25, 26, 21, 18, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0.0};

// MOTOR 3 
Motor m3 = {32, 33, 13, 23, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0.0};

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
   CONTROL
======================= */
void controlMotor(Motor &m) {
  long currentCount;

  portENTER_CRITICAL(&mux);
  currentCount = m.encoderCount;
  portEXIT_CRITICAL(&mux);

  long delta = currentCount - m.lastCount;
  m.lastCount = currentCount;

  m.rpmRaw = ((float)delta / PPR) * (60.0f / Ts) * 2;
  m.e_k = m.referencia - m.rpmRaw;
  m.u_k = (q0 * m.e_k) + (q1 * m.e_k1)
        - ((s0 - 1.0f) * m.u_k1)
        + (s0 * m.u_k2);

  if (m.u_k > U_SAT) m.u_k = U_SAT;
  if (m.u_k < -U_SAT) m.u_k = -U_SAT;

  analogWrite(m.PWM, (int)(fabs(m.u_k) * 255));
  digitalWrite(m.DIR1, (m.u_k >= 0));
  digitalWrite(m.DIR2, !(m.u_k >= 0));

  m.u_k2 = m.u_k1; 
  m.u_k1 = m.u_k; 
  m.e_k1 = m.e_k;
}

/* =======================
   SERIAL
======================= */
void checkSerial() {
  if (Serial.available()) {
    float val = Serial.parseFloat();

    // Misma referencia para los 3 motores
    m1.referencia = val;
    m2.referencia = val;

    m3.referencia = val;

    while (Serial.available()) Serial.read();

    Serial.print("Nueva Referencia OK: "); 
    Serial.println(val);
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
    
    // Telemetría: Ref, M1, M2, M3
    Serial.print(m1.referencia); Serial.print(",");
    Serial.print(m1.rpmRaw); Serial.print(",");
    Serial.print(m2.rpmRaw); Serial.print(",");
    Serial.println(m3.rpmRaw);
  }
}
