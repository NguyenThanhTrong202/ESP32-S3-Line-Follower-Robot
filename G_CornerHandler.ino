
// xong goc: quay lai do line PID, tang toc tu tu
void resumePid() {
  mode = M_PID; cornerCnt = 0; centreHits = 0;
  lostActive = false; iAcc = 0; pidInit = false;
  baseNow = P.base * 0.4f;
}
void cornerTick(uint32_t nowMs, bool seen, int cl, int cr) {
  uint32_t inMode = nowMs - modeMs;
  bool centre = seen && centreLineSeen();
  bool both = (cl >= 3 && cr >= 3);              
  switch (mode) {
    case M_BLOCK:                                
      setMotors(P.tapp, P.tapp);
      if (!both) {
        if (centre) resumePid();                 
        else if (cl >= 2 && cr < 2) { turnSide = -1; setMode(M_FWD, nowMs); }
        else if (cr >= 2 && cl < 2) { turnSide = 1;  setMode(M_FWD, nowMs); }
        else {                                   
          turnSide = (int)P.jside;
          if (watchMode) Serial.printf("NGA BA T -> re %s\n", turnSide < 0 ? "TRAI" : "PHAI");
          setMode(M_FWD, nowMs);
        }
      } else if (inMode > (uint32_t)P.losttmo) stopRun("O DAC: di thang qua lau");
      break;
    case M_APPROACH:
      if (both) { setMode(M_BLOCK, nowMs); break; }   
      
      if (inMode < (uint32_t)P.tbrk) { motorBrake(); break; }
      setMotors(P.tapp, P.tapp);
      if (cl < 2 && cr < 2) {                    
        if (centre && P.jbranch < 0.5f) { if (watchMode) Serial.println("NHANH RE: line thang van con -> di thang"); resumePid(); }
        else setMode(M_FWD, nowMs);              
      } else if (inMode > (uint32_t)P.tappmax) {
        centreClear = false; setMode(M_PIVOT, nowMs);
      }
      break;
    case M_FWD:                                  
      setMotors(P.tapp, P.tapp);
      if (inMode >= (uint32_t)P.tfwd) { centreClear = !centre; centreHits = 0; setMode(M_PIVOT, nowMs); }
      break;
    case M_PIVOT:
      setMotors(turnSide * P.tspd, -turnSide * P.tspd);   
      if (!centre) { centreClear = true; centreHits = 0; }
      else if (centreClear && inMode >= SMALL_PIVOT_MIN_MS) {
        
        if (++centreHits >= 2) { setMode(M_SETTLE, nowMs); motorBrake(); }
      } else centreHits = 0;
      if (mode == M_PIVOT && inMode > (uint32_t)P.losttmo)
        stopRun("GOC 90: quay khong thay line");
      break;
    case M_SETTLE:                               
      motorBrake();
      if (inMode >= (uint32_t)P.tbrk) resumePid();
      break;
    default: break;
  }
}

