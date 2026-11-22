

int buttonEvent() {
  static bool down = false, lastRaw = HIGH;
  static uint32_t changedAt = 0, downAt = 0;
  bool r = digitalRead(PIN_BUTTON);
  uint32_t ms = millis();
  if (r != lastRaw) { lastRaw = r; changedAt = ms; }
  if (ms - changedAt < 30) return 0;
  if (!down && r == LOW)  { down = true; downAt = ms; }
  if (down && r == HIGH)  { down = false; return (ms - downAt > 800) ? 2 : 1; }
  return 0;
}
void buttonService() {
  int e = buttonEvent();
  if (!e) return;
  if (state == ST_RUN || state == ST_OBSTACLE) { stopRun("nut BOOT"); return; }
  if (e == 2 || !calibrated) { doCalibrate(); return; }
  delay(500);                       
  doRun();
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  pwmSetupPin(PIN_PWMA, CH_A);
  pwmSetupPin(PIN_PWMB, CH_B);
  setDefaults();
  setMotors(0, 0);

  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  analogReadResolution(12);              

  bool pOk = loadParams();
  if (P.amb > 0.5f) irSet(true);
  bool cOk = loadCal();
  Serial.println();
  Serial.println("XE DO LINE ESP32-S3 V2.9-SMALL8");
  Serial.println(pOk ? "Da nap thong so da luu." : "Dung thong so mac dinh (go 'save' de luu).");
  if (cOk) { calibrated = true; state = ST_READY; Serial.println("Da nap hieu chinh da luu. Bam BOOT de CHAY (giu nut >0,8s de hieu chinh lai)."); }
  else Serial.println("Dat xe len line, bam BOOT de hieu chinh. Go 'help' de xem lenh.");
}

void loop() {
  serialService();
  buttonService();
  sonarService();
  watchService();
  switch (state) {
    case ST_RUN:      runTick();      break;
    case ST_OBSTACLE: obstacleTick(); break;
    default: break;
  }
}
