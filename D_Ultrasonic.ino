// ===================== HC-SR04 (khong chan vong dieu khien) =====================
volatile uint32_t echoRise = 0, echoWidth = 0;
volatile bool echoReady = false;
void IRAM_ATTR echoISR() {
  uint32_t t = micros();
  if (digitalRead(PIN_ECHO)) echoRise = t;
  else { echoWidth = t - echoRise; echoReady = true; }
}
uint32_t sonarTrigUs = 0, lastSonarMs = 0;
bool sonarPending = false;
float dist3[3] = {999, 999, 999};
int dist3i = 0;
void pushDist(float cm) { dist3[dist3i] = cm; dist3i = (dist3i + 1) % 3; }
float distMax3() { float m = dist3[0]; if (dist3[1] > m) m = dist3[1]; if (dist3[2] > m) m = dist3[2]; return m; }
float distMin3() { float m = dist3[0]; if (dist3[1] < m) m = dist3[1]; if (dist3[2] < m) m = dist3[2]; return m; }

void sonarService() {
  uint32_t nowMs = millis();
  if (echoReady) {
    noInterrupts();
    uint32_t w = echoWidth;
    echoReady = false;
    interrupts();
    pushDist((w >= 100 && w <= 25000) ? (float)w / 58.0f : 999.0f);
    sonarPending = false;
  }
  if (sonarPending && (uint32_t)(micros() - sonarTrigUs) > 30000UL) { pushDist(999); sonarPending = false; }
  if (!sonarPending && (uint32_t)(nowMs - lastSonarMs) >= 60) {
    lastSonarMs = nowMs;
    sonarPending = true;
    echoReady = false;
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    sonarTrigUs = micros();
  }
}

bool obstacleNear()  { return P.obscm >= 1 && distMax3() < P.obscm; }
bool obstacleClear() { return P.obscm < 1 || distMin3() > P.obscm + 5; }

