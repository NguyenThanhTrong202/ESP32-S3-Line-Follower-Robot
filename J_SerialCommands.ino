// ===================== LENH SERIAL =====================
void printParams() {
  for (int i = 0; i < PT_COUNT; i++) Serial.printf("  %-8s = %-8.3f  %s\n", PT[i].name, *PT[i].v, PT[i].help);
}
void printHelp() {
  Serial.println("LENH:");
  Serial.println("  <ten> <gia tri>  doi thong so (vd: kp 0.09). Chi go <ten> de xem gia tri.");
  Serial.println("  get              xem tat ca thong so      save   luu thong so   defaults  ve mac dinh");
  Serial.println("  cal              hieu chinh               run    chay          stop      dung");
  Serial.println("  testl / testr    quay thu banh TRAI / PHAI 0,7 giay (NANG XE LEN CAO)");
  Serial.println("  watch 1|0        bat/tat hien thi truc tiep   timing   toc do vong dieu khien");
  Serial.println("  sensors          doc cam bien 1 lan       status    trang thai");
  Serial.println("  noise            do nhieu tung mat khi xe dung yen (SNR = chenh line/nen chia nhieu)");
  Serial.println("  dump             in nhat ky 3 giay cuoi cua lan chay vua roi (xe tu ghi, khong can cam day luc chay)");
}
void printTiming() {
  uint32_t el = millis() - st.startMs;
  if (!st.count || !el) { Serial.println("Chua co so lieu: hay cho xe chay it nhat vai giay roi go timing."); return; }
  float hz = st.count * 1000.0f / el;
  Serial.printf("vong dieu khien: %.0f Hz (muc tieu %.0f Hz)\n", hz, 1e6f / P.period);
  Serial.printf("cong viec moi vong: TB %lu us, toi da %lu us (quet cam bien toi da %lu us)\n",
                (unsigned long)(st.workSum / st.count), (unsigned long)st.workMax, (unsigned long)st.scanMax);
  Serial.printf("tre %lu lan, bo lo %lu chu ky\n", (unsigned long)st.late, (unsigned long)st.missed);
  if (st.workMax > P.period * 8 / 10) Serial.println("=> Cong viec gan het chu ky: tang 'period' hoac dat 'filt 1'.");
}

void noiseTest() {
  if (state == ST_RUN || state == ST_OBSTACLE) { Serial.println("Hay go 'stop' truoc."); return; }
  const int N = 200;
  double sum[8] = {0}, sq[8] = {0}; int mn[8], mx[8];
  for (int p = 0; p < 8; p++) { mn[p] = 4095; mx[p] = 0; }
  Serial.println("Dang do nhieu ~1 giay, de xe DUNG YEN...");
  for (int k = 0; k < N; k++) {
    readRaw();
    for (int p = 0; p < 8; p++) {
      sum[p] += raw[p]; sq[p] += (double)raw[p] * raw[p];
      if (raw[p] < mn[p]) mn[p] = raw[p];
      if (raw[p] > mx[p]) mx[p] = raw[p];
    }
    delay(4);
  }
  bool weak = false;
  for (int p = 0; p < 8; p++) {
    double m = sum[p] / N, sd = sqrt(sq[p] / N - m * m); if (!(sd > 0.5)) sd = 0.5;
    Serial.printf("  D%d TB=%6.0f  dao dong=%4d  do lech chuan=%5.1f", p + 1, m, mx[p] - mn[p], sd);
    if (calibrated) {
      int range = vmax[p] - vmin[p];
      double snr = range / sd;
      Serial.printf("  chenh line/nen=%4d  SNR=%5.1f%s", range, snr, snr < 8 ? "  <-- NHIEU" : "");
      if (snr < 8) weak = true;
    }
    Serial.println();
  }
  if (calibrated) Serial.println(weak ? "=> SNR < 8: tang 'filt'/'ema', thu 'amb 1', ha thap mat do, che sang. Tang 'nfloor' neu nen bao '#'."
                                      : "=> SNR on (>= 8).");
  else Serial.println("Chua hieu chinh: chi co so do nhieu tho.");
}
void testWheel(bool leftWheel) {
  if (state == ST_RUN || state == ST_OBSTACLE) { Serial.println("Hay go 'stop' truoc."); return; }
  Serial.println(leftWheel ? "Quay thu banh TRAI tien 0,7 giay..." : "Quay thu banh PHAI tien 0,7 giay...");
  Serial.println("  Dung banh khong?  Sai banh -> doi swap.  Quay lui -> doi lrev (trai) / rrev (phai).");
  motorsEnable(true);
  if (leftWheel) setMotors(90, 0); else setMotors(0, 90);
  delay(700);
  motorsEnable(false);
}
void doCalibrate() {
  if (state == ST_RUN || state == ST_OBSTACLE) { Serial.println("Hay dung xe truoc."); return; }
  Serial.println("Dang hieu chinh (xe xoay trai/phai ~4 giay)...");
  state = runCalibration() ? ST_READY : ST_IDLE;
  if (state == ST_READY) Serial.println("Bam BOOT hoac go 'run' de CHAY");
  else Serial.println("Bam BOOT de hieu chinh lai (dat mach do line len line, ha thap 3-8 mm)");
}
void doRun() {
  if (state == ST_RUN || state == ST_OBSTACLE) return;
  if (!calibrated) { Serial.println("Chua hieu chinh: dang hieu chinh truoc."); doCalibrate(); return; }
  startRun();
}
void printStatus() {
  const char *names[] = {"CHUA HIEU CHINH", "SAN SANG", "DANG CHAY", "DUNG VI VAT CAN"};
  Serial.printf("trang thai: %s, hieu chinh: %s, vat can: %.0f cm\n", names[state], calibrated ? "co" : "chua", distMin3());
}

void handleCommand(char *line) {
  char t0[20] = "", t1[20] = "", t2[20] = "";
  int n = sscanf(line, "%19s %19s %19s", t0, t1, t2);
  if (n <= 0) return;
  const char *name = t0, *arg = t1;
  if (strcmp(t0, "set") == 0 && n >= 2) { name = t1; arg = t2; }
  
  for (int i = 0; i < PT_COUNT; i++) {
    if (strcmp(name, PT[i].name) != 0) continue;
    if (arg[0] == 0) { Serial.printf("%s = %.3f\n", PT[i].name, *PT[i].v); return; }
    float x = (float)atof(arg);
    float c = clampf(x, PT[i].lo, PT[i].hi);
    *PT[i].v = c;
    sanitizeParams();
    if (c != x) Serial.printf("%s = %.3f (da gioi han trong khoang %.3f..%.3f)\n", PT[i].name, *PT[i].v, PT[i].lo, PT[i].hi);
    else Serial.printf("%s = %.3f (go 'save' de luu)\n", PT[i].name, *PT[i].v);
    return;
  }
  if      (strcmp(name, "help") == 0)     printHelp();
  else if (strcmp(name, "get") == 0)      printParams();
  else if (strcmp(name, "save") == 0)     { saveParams(); Serial.println("Da luu thong so."); }
  else if (strcmp(name, "defaults") == 0) { setDefaults(); Serial.println("Da ve mac dinh (chua luu: go 'save' de luu)."); }
  else if (strcmp(name, "cal") == 0)      doCalibrate();
  else if (strcmp(name, "run") == 0)      doRun();
  else if (strcmp(name, "stop") == 0)     { if (state == ST_RUN || state == ST_OBSTACLE) stopRun("lenh stop"); else motorsEnable(false); }
  else if (strcmp(name, "testl") == 0)    testWheel(true);
  else if (strcmp(name, "testr") == 0)    testWheel(false);
  else if (strcmp(name, "watch") == 0)    { watchMode = (arg[0] == '0') ? 0 : 1; Serial.printf("watch = %d\n", watchMode); }
  else if (strcmp(name, "timing") == 0)   printTiming();
  else if (strcmp(name, "status") == 0)   printStatus();
  else if (strcmp(name, "dump") == 0)     logDump();
  else if (strcmp(name, "noise") == 0)    noiseTest();
  else if (strcmp(name, "sensors") == 0) {
    readRaw();
    Serial.print("raw D1..D8:");
    for (int i = 0; i < 8; i++) Serial.printf(" %4d", raw[i]);
    Serial.println();
    if (calibrated) { computeLine(); printBar(); Serial.printf(" pos=%.0f\n", pos); }
    Serial.print("bu nen D1..D8:");
    for (int i = 0; i < 8; i++) Serial.printf(" %4.0f", bOff[i]);
    Serial.println();
  }
  else Serial.println("Lenh khong hieu. Go 'help'.");
}

char cmdBuf[80];
int cmdLen = 0;
void serialService() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = (char)tolower(c);
    }
  }
}

