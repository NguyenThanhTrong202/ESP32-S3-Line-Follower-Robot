
bool runCalibration() {
  for (int i = 0; i < 8; i++) { vmin[i] = 4095; vmax[i] = 0; }
  resetAdapt();
  motorsEnable(true);
  uint32_t t0 = millis();
  
  const int dirs[4] = {1, -1, 1, -1};
  const uint32_t ends[4] = {600, 1800, 3000, 3600};
  int seg = 0;
  while (seg < 4) {
    uint32_t t = millis() - t0;
    while (seg < 4 && t >= ends[seg]) seg++;
    if (seg >= 4) break;
    setMotors(-dirs[seg] * P.calspd, dirs[seg] * P.calspd);
    readRaw();
    for (int i = 0; i < 8; i++) {
      if (raw[i] < vmin[i]) vmin[i] = raw[i];
      if (raw[i] > vmax[i]) vmax[i] = raw[i];
    }
    delay(3);
  }
  setMotors(0, 0);
  bool ok = true;
  for (int i = 0; i < 8; i++) if (vmax[i] - vmin[i] < (int)P.calmin) ok = false;
  Serial.println(ok ? "Hieu chinh OK" : "Hieu chinh LOI: co mat khong thay chenh lech line/nen");
  for (int p = 0; p < 8; p++)
    Serial.printf("  D%d min=%4d max=%4d%s\n", p + 1, vmin[p], vmax[p], (vmax[p] - vmin[p] < (int)P.calmin) ? "   <-- LOI" : "");
  if (ok) Serial.println("  Go 'noise' (xe dung yen tren NEN, roi tren LINE) de kiem tra nhieu.");
  if (ok) {
    calibrated = true;
    saveCal();
    
    uint32_t t1 = millis();
    while (millis() - t1 < 1500) {
      readRaw();
      bool on = computeLine();
      if (on && fabsf(pos - 3500) < 400) break;
      int dir = (pos < 3500) ? 1 : -1;
      if (!on) dir = 1;
      setMotors(-dir * P.calspd, dir * P.calspd);
      delay(3);
    }
  } else {
    calibrated = false;
  }
  motorsEnable(false);
  return ok;
}



void adaptBackground() {
  if (P.adapt <= 0) return;
  bool rev = P.linerev > 0.5f;
  float sgn = (P.linehigh > 0.5f) ? 1.0f : -1.0f;
  float target = P.nfloor * 0.5f;
  for (int i = 0; i < 8; i++) {
    if (val[i] != 0) continue;
    if (i > 0 && val[i - 1] != 0) continue;
    if (i < 7 && val[i + 1] != 0) continue;
    int p = rev ? 7 - i : i;
    float range = (float)(vmax[p] - vmin[p]);
    bOff[p] += P.adapt * sgn * (vnorm[i] - target) * range / 1000.0f;
    bOff[p] = clampf(bOff[p], -range, range);
  }
}

