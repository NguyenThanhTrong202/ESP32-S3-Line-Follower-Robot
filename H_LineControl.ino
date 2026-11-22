
void runTick() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;

  float dt = (float)(uint32_t)(now - lastTickUs) * 1e-6f;
  lastTickUs = now;
  if (!pidInit) dt = P.period * 1e-6f;
  if (dt < 0.0005f) dt = 0.0005f;
  if (dt > 0.05f) dt = 0.05f;
  uint32_t period = (uint32_t)P.period;
  if ((uint32_t)(now - nextTick) > period / 2) st.late++;
  nextTick += period;
  if ((int32_t)(now - nextTick) >= 0) { nextTick = now + period; st.missed++; }

  
  if (obstacleNear()) {
    setMotors(0, 0);
    state = ST_OBSTACLE;
    Serial.printf("VAT CAN %.0f cm -> dung cho\n", distMin3());
    return;
  }

  readRaw();
  bool seen = computeLine();
  uint32_t nowMs = millis();

  
  int cl = 0; while (cl < 8 && val[cl] > 500) cl++;
  int cr = 0; while (cr < 8 && val[7 - cr] > 500) cr++;

  if (mode != M_PID) {
    cornerTick(nowMs, seen, cl, cr);
    if (state == ST_RUN) logAdd(nowMs, false);
    return;
  }
  if (P.corner > 0.5f && seen && !lostActive && cl >= 3 && cr >= 3) {
    setMode(M_BLOCK, nowMs); cornerCnt = 0;
    if (watchMode) Serial.println("O DAC / NGA TU -> di thang");
    setMotors(P.tapp, P.tapp);
    logAdd(nowMs, false);
    return;
  }
  if (P.corner > 0.5f && seen && !lostActive) {
    int side = 0;
    bool narrow = false;
    
    if (cl >= (int)P.tdet && cr < 2) side = -1;
    else if (cr >= (int)P.tdet && cl < 2) side = 1;
    else {
      
      
      bool noCentre = !centreLineSeen();
      bool leftTip = noCentre && val[0] > SMALL_EDGE_HI && val[7] < SMALL_OPP_MAX
                     && pos < SMALL_EDGE_POS && nOn <= 3;
      bool rightTip = noCentre && val[7] > SMALL_EDGE_HI && val[0] < SMALL_OPP_MAX
                      && pos > 7000.0f - SMALL_EDGE_POS && nOn <= 3;
      if (leftTip && !rightTip) { side = -1; narrow = true; }
      else if (rightTip && !leftTip) { side = 1; narrow = true; }
    }
    if (side != 0) {
      if (side == cornerSide) cornerCnt++;
      else { cornerSide = side; cornerCnt = 1; }
    } else { cornerCnt = 0; cornerSide = 0; }
    if (side != 0 && cornerCnt >= (narrow ? SMALL_CONFIRM_TICKS : 2)) {
      turnSide = side; cornerCnt = 0; centreHits = 0;
      if (narrow) {
        
        
        setMode(M_FWD, nowMs);
        setMotors(P.tapp, P.tapp);
      } else {
        setMode(M_APPROACH, nowMs);
        motorBrake();
      }
      if (watchMode) Serial.printf("GOC 90 %s (%s)\n", side < 0 ? "TRAI" : "PHAI", narrow ? "MAT NGOAI" : "VACH NGANG");
      logAdd(nowMs, false);
      return;
    }
  }

  
  if (seen) {
    bool le = val[0] > 500, re = val[7] > 500;
    if (le && !re) { edgeSide = -1; edgeMs = nowMs; }
    else if (re && !le) { edgeSide = 1; edgeMs = nowMs; }
  }
  
  bool centre = centreLineSeen();
  if (lostActive && seen && !centre) seen = false;

  if (!seen) {
    
    if (!lostActive) {
      lostActive = true; lostSinceMs = nowMs;
      
      if (edgeSide != 0 && (uint32_t)(nowMs - edgeMs) < 400) pivotSide = edgeSide;
      else pivotSide = (lastPos < 3500) ? -1 : 1;
      if (watchMode) Serial.printf("MAT LINE -> quay %s\n", pivotSide < 0 ? "TRAI" : "PHAI");
    }
    uint32_t lostFor = nowMs - lostSinceMs;
    
    bool edgeRecent = edgeSide != 0 && (uint32_t)(lostSinceMs - edgeMs) < 300;
    bool wasStraight = fabsf(lastCorr) < 60.0f && !edgeRecent;
    if (wasStraight && lostFor < (uint32_t)P.lostms) {
      float h = 0.5f * (lastL + lastR);
      if (h < P.tapp) h = P.tapp;
      setMotors(h, h);
    } else if (lostFor < (uint32_t)P.losttmo) {
      int dir = -pivotSide;                           
      setMotors(-dir * P.search, dir * P.search);
    } else {
      stopRun("MAT LINE qua lau");
      return;
    }
    baseNow = 0;
  } else {
    if (lostActive) {                                
      lostActive = false;
      iAcc = 0; pidInit = false;
      baseNow = P.base * 0.4f;
    }
    if (!pidInit) { lastPos = pos; dFilt = 0; pidInit = true; }
    // ---- PID (D theo vi tri do, loc thong thap) ----
    float err = pos - 3500.0f;
    float rawD = (pos - lastPos) / dt;
    float a = dt / (P.dtau * 0.001f + dt);
    dFilt += a * (rawD - dFilt);
    lastPos = pos;
    float dTerm = P.kd * dFilt * 0.005f;             
    iAcc += err * dt;
    iAcc = clampf(iAcc, -2000.0f, 2000.0f);
    float errP = err;                                
    if (errP > P.dead) errP -= P.dead; else if (errP < -P.dead) errP += P.dead; else errP = 0;
    float corr = P.kp * errP + P.ki * iAcc + dTerm;
    lastCorr = corr;

    
    float k = 1.0f - P.curve * fabsf(err) / 3500.0f;
    if (k < 0.25f) k = 0.25f;
    float target = P.base * k;
    if (fabsf(err) > P.cerr && target > P.cspd) target = P.cspd;   
    target /= (1.0f + P.dslow * fabsf(dFilt) / 20000.0f);          
    if (target > baseNow) { baseNow += P.rise * dt; if (baseNow > target) baseNow = target; }
    else baseNow = target;

    float revLim = P.rev * P.maxpwm;
    float L = baseNow + corr, R = baseNow - corr;
    bool sat = (L > P.maxpwm) || (R > P.maxpwm) || (L < -revLim) || (R < -revLim);
    
    if (L > P.maxpwm) { R -= (L - P.maxpwm); L = P.maxpwm; }
    if (R > P.maxpwm) { L -= (R - P.maxpwm); R = P.maxpwm; }
    L = clampf(L, -revLim, P.maxpwm);
    R = clampf(R, -revLim, P.maxpwm);
    if (sat) iAcc -= err * dt;                       
    setMotors(L, R);
    adaptBackground();
  }

  logAdd(nowMs, !seen);
  uint32_t work = (uint32_t)(micros() - now);
  st.count++; st.workSum += work;
  if (work > st.workMax) st.workMax = work;
  if (scanUsLast > st.scanMax) st.scanMax = scanUsLast;
}

void obstacleTick() {
  setMotors(0, 0);
  if (obstacleClear()) {
    Serial.println("Het vat can -> CHAY tiep");
    iAcc = 0; dFilt = 0; baseNow = 0; lostActive = false; pidInit = false; edgeSide = 0;
    mode = M_PID; cornerCnt = 0;
    nextTick = micros(); lastTickUs = nextTick;
    state = ST_RUN;
  }
}

