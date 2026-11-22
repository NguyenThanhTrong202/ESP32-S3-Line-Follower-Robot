
const char *BARCH = " .:-=+*#%@";
void printBar() {
  char s[9];
  for (int i = 0; i < 8; i++) { int v = val[i] / 112; if (v > 9) v = 9; s[i] = BARCH[v]; }
  s[8] = 0;
  Serial.print("["); Serial.print(s); Serial.print("]");
}
uint32_t lastWatchMs = 0;
void watchService() {
  if (!watchMode) return;
  uint32_t ms = millis();
  if (state == ST_RUN) {
    if (ms - lastWatchMs < 100) return;
    lastWatchMs = ms;
    printBar();
    const char *mn[] = {"", " VAO-GOC", " QUA-GOC", " QUAY", " PHANH", " O-DAC"};
    Serial.printf(" pos=%4.0f corr=%5.0f L=%4.0f R=%4.0f base=%3.0f dist=%.0f%s%s\n",
                  pos, lastCorr, lastL, lastR, baseNow, distMin3(), lostActive ? " MAT-LINE" : "", mn[mode]);
  } else if (state == ST_READY || state == ST_IDLE) {
    if (ms - lastWatchMs < 250) return;
    lastWatchMs = ms;
    readRaw();
    if (calibrated) { computeLine(); printBar(); Serial.printf(" pos=%4.0f nOn=%d dist=%.0f\n", pos, nOn, distMin3()); }
    else {
      Serial.print("raw D1..D8:");
      for (int i = 0; i < 8; i++) Serial.printf(" %4d", raw[i]);
      Serial.printf("  dist=%.0f\n", distMin3());
    }
  }
}

