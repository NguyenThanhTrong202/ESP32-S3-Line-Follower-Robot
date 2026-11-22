
enum State { ST_IDLE, ST_READY, ST_RUN, ST_OBSTACLE };
State state = ST_IDLE;
int watchMode = 0;

// PID / lai
float lastPos = 3500, dFilt = 0, iAcc = 0, baseNow = 0, lastCorr = 0;
uint32_t lastTickUs = 0, nextTick = 0, lostSinceMs = 0;
bool lostActive = false;
bool pidInit = false;      
int  edgeSide = 0;         
uint32_t edgeMs = 0;
int  pivotSide = 0;        


enum Mode { M_PID, M_APPROACH, M_FWD, M_PIVOT, M_SETTLE, M_BLOCK };
Mode mode = M_PID;
uint32_t modeMs = 0;
int  turnSide = 0, cornerSide = 0, cornerCnt = 0;
bool centreClear = false;
int centreHits = 0;         


const int SMALL_EDGE_HI = 720;        
const int SMALL_OPP_MAX = 260;
const float SMALL_EDGE_POS = 1050.0f; 
const int SMALL_CONFIRM_TICKS = 3;
const uint32_t SMALL_PIVOT_MIN_MS = 12;

bool centreLineSeen() {
  
  return val[3] > 320 || val[4] > 320 || val[2] > 520 || val[5] > 520;
}

void setMode(int m, uint32_t ms) { mode = (Mode)m; modeMs = ms; }   


struct LogRec { uint32_t ms; int16_t pos, L, R, base; uint8_t mask, flags; };
const int LOG_N = 400;
LogRec logBuf[LOG_N];
int logHead = 0, logCount = 0, logDiv = 0;
void logAdd(uint32_t ms, bool lost) {
  if (++logDiv < 2) return;
  logDiv = 0;
  LogRec &r = logBuf[logHead];
  r.ms = ms; r.pos = (int16_t)pos; r.L = (int16_t)lastL; r.R = (int16_t)lastR; r.base = (int16_t)baseNow;
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) if (val[i] > 400) m |= (1 << i);
  r.mask = m; r.flags = (lost ? 1 : 0) | ((mode == M_APPROACH || mode == M_FWD) ? 2 : 0) | ((mode == M_PIVOT || mode == M_SETTLE) ? 4 : 0) | (mode == M_BLOCK ? 8 : 0);
  logHead = (logHead + 1) % LOG_N;
  if (logCount < LOG_N) logCount++;
}
void logDump() {
  if (!logCount) { Serial.println("Chua co nhat ky: hay cho xe chay roi dung."); return; }
  Serial.println("NHAT KY (cu -> moi). Cot: thoi gian(ms), cam bien TRAI->PHAI (#=thay line), pos, L, R, base, M=mat line G=vao goc Q=quay B=o dac/nga tu");
  int start = (logHead - logCount + LOG_N) % LOG_N;
  uint32_t t0 = logBuf[start].ms;
  for (int k = 0; k < logCount; k++) {
    LogRec &r = logBuf[(start + k) % LOG_N];
    char bar[9];
    for (int i = 0; i < 8; i++) bar[i] = (r.mask & (1 << i)) ? '#' : '.';
    bar[8] = 0;
    Serial.printf("%5lu [%s] pos=%5d L=%4d R=%4d base=%3d %s%s%s%s\n", (unsigned long)(r.ms - t0), bar, r.pos, r.L, r.R, r.base,
                  (r.flags & 1) ? "M" : "", (r.flags & 2) ? "G" : "", (r.flags & 4) ? "Q" : "", (r.flags & 8) ? "B" : "");
    delay(2);
  }
  Serial.println("HET NHAT KY");
}

struct Stats { uint32_t count, late, missed; uint32_t workMax, scanMax; uint64_t workSum; uint32_t startMs; };
Stats st;
void statsReset() { memset(&st, 0, sizeof(st)); st.startMs = millis(); }

void startRun() {
  iAcc = 0; dFilt = 0; baseNow = 0; lastCorr = 0; lostActive = false; pidInit = false; edgeSide = 0; pivotSide = 0; centreHits = 0;
  mode = M_PID; cornerCnt = 0; cornerSide = 0;
  resetAdapt();
  lastPos = 3500; lastL = lastR = 0;
  statsReset();
  logHead = 0; logCount = 0; logDiv = 0;
  motorsEnable(true);
  nextTick = micros();
  lastTickUs = nextTick;
  state = ST_RUN;
  Serial.println("CHAY");
}
void stopRun(const char *why) {
  motorsEnable(false);
  state = ST_READY;
  Serial.print("DUNG: "); Serial.println(why);
}

