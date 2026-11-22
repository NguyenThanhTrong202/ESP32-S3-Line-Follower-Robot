
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

// ===================== SO DO CHAN =====================
const int PIN_PWMA = 13, PIN_AIN2 = 12, PIN_AIN1 = 11, PIN_STBY = 10;
const int PIN_BIN1 = 9,  PIN_BIN2 = 46, PIN_PWMB = 14;      
const int PIN_LINE[8] = {18, 17, 16, 15, 7, 6, 5, 4};        
const int PIN_TRIG = 48, PIN_ECHO = 47;
const int PIN_BUTTON = 0;
const int PIN_IR = 21;      

const int   PWM_HARD_MAX = 255;    

// ===================== THONG SO  =====================
const uint32_t PARAM_MAGIC = 0x58444C3C;   
struct Params {
  uint32_t magic;
  float kp, ki, kd, dtau;          
  float base, maxpwm, curve, rise, rev;  
  float period, filt;              
  float dead;                      
  float cspd, cerr;                
  float dslow, minpwm;           
  float corner, tdet, tapp, tbrk, tfwd, tspd, tappmax;   
  float calmin, nfloor, ema, amb, irus, irhigh;       
  float jside, jbranch, adapt;                         
  float swap, lrev, rrev, linerev, linehigh;
};
Params P;

struct PInfo { const char *name; float *v; float lo, hi; const char *help; };
PInfo PT[] = {
  {"kp",      &P.kp,      0.0f,  2.0f,   "Kp: tang neu cua cham; giam neu lac"},
  {"ki",      &P.ki,      0.0f,  0.01f,  "Ki: thuong de 0"},
  {"kd",      &P.kd,      0.0f,  5.0f,   "Kd: tang de giam lac, ~5 x Kp"},
  {"dtau",    &P.dtau,    0.5f,  50.0f,  "loc nhieu cua D (ms)"},
  {"base",    &P.base,    20.0f, 255.0f, "toc do nen (PWM). De thap hon maxpwm ~30-50 de con luc re"},
  {"maxpwm",  &P.maxpwm,  50.0f, 255.0f, "PWM toi da moi banh (191 = 75%, an toan cho motor 6V; 255 = het cong suat)"},
  {"curve",   &P.curve,   0.0f,  0.9f,   "giam toc vao cua (0 = khong giam, 0.65 = con 35%)"},
  {"rise",    &P.rise,    50.0f, 2000.0f,"toc do tang toc lai sau cua (PWM/giay)"},
  {"rev",     &P.rev,     0.0f,  1.0f,   "banh trong cua duoc quay nguoc toi bao nhieu % maxpwm"},
  {"dead",    &P.dead,    0.0f,  800.0f, "vung chet cua lech: lech < dead thi khong be lai. Tang neu xe giat khi chay thang (thu 100-250)"},
  {"cspd",    &P.cspd,    20.0f, 255.0f, "toc do nen toi da khi vao cua (chan tren). Thap = bat kip line, cham hon"},
  {"cerr",    &P.cerr,    500.0f,3500.0f,"do lech (0..3500) tu do tinh la 'vao cua' va ap cspd. Thap = phanh som hon"},
  {"dslow",   &P.dslow,   0.0f,  3.0f,   "giam toc khi line troi nhanh (vao cua). 0 = tat, tang neu cua cong van van ra ngoai"},
  {"minpwm",  &P.minpwm,  0.0f,  100.0f, "bu vung chet motor: PWM nho nhat banh bat dau quay (thu 25-45). 0 = tat"},
  {"corner",  &P.corner,  0.0f,  1.0f,   "1 = bat che do goc 90 do"},
  {"tdet",    &P.tdet,    3.0f,  7.0f,   "so mat LIEN TIEP tu mep thay line thi tinh la goc 90 (4; giam 3 neu line manh/mat thua)"},
  {"tapp",    &P.tapp,    20.0f, 200.0f, "toc do di thang qua goc truoc khi quay"},
  {"tbrk",    &P.tbrk,    0.0f,  150.0f, "phanh bao nhieu ms khi gap goc va khi quay xong"},
  {"tfwd",    &P.tfwd,    0.0f,  400.0f, "di them bao nhieu ms sau khi qua vach ngang roi moi quay (tang neu quay xong bi lech ra ngoai)"},
  {"tspd",    &P.tspd,    40.0f, 255.0f, "toc do quay tai cho o goc 90"},
  {"tappmax", &P.tappmax, 50.0f, 1000.0f,"di thang qua goc toi da bao nhieu ms (an toan)"},
  {"jside",   &P.jside,  -1.0f,  1.0f,   "nga ba chu T (qua vach ngang ma phia truoc trang): 1 = re PHAI, -1 = re TRAI"},
  {"jbranch", &P.jbranch, 0.0f,  1.0f,   "nhanh re 1 phia ma line thang van con: 0 = di thang, 1 = re vao nhanh"},
  {"adapt",   &P.adapt,   0.0f,  0.02f,  "toc do tu bu muc nen khi chay (0 = tat). 0.003 ~ thich nghi trong ~1-2 giay"},
  {"search",  &P.search,  40.0f, 255.0f, "toc do xoay tim line khi mat line"},
  {"lostms",  &P.lostms,  0.0f,  500.0f, "mat line luc dang di thang: giu huong cu bao nhieu ms (cau noi vach dut). O goc re: khong giu"},
  {"losttmo", &P.losttmo, 300.0f,10000.0f,"mat line qua bao nhieu ms thi dung han"},
  {"calspd",  &P.calspd,  40.0f, 150.0f, "toc do xoay khi hieu chinh"},
  {"obscm",   &P.obscm,   0.0f,  100.0f, "vat can gan hon (cm) thi dung; 0 = tat"},
  {"period",  &P.period,  1000.0f,10000.0f,"chu ky dieu khien (us). 4000 = 250Hz"},
  {"filt",    &P.filt,    1.0f,  16.0f,  "so mau moi mat (1..16): bo min/max roi lay trung binh. Nhieu -> tang 8-12 (xem 'timing')"},
  {"ema",     &P.ema,     0.0f,  0.8f,   "loc theo thoi gian (0 = tat). 0.3-0.5 giam nhieu, cao qua xe phan ung tre"},
  {"nfloor",  &P.nfloor,  0.0f,  600.0f, "san nhieu (0..1000): duoi muc nay coi la NEN. Tang neu nen bi bao '#' khi khong co line"},
  {"calmin",  &P.calmin,  20.0f, 1000.0f,"chenh lech toi thieu line/nen moi mat khi hieu chinh (don vi ADC)"},
  {"amb",     &P.amb,     0.0f,  1.0f,   "1 = tru anh sang moi truong (can noi chan IR vao GPIO21)"},
  {"irus",    &P.irus,    50.0f, 2000.0f,"cho bao nhieu us sau khi bat/tat LED IR roi moi doc"},
  {"irhigh",  &P.irhigh,  0.0f,  1.0f,   "1 = chan IR muc CAO thi LED sang; 0 = nguoc lai"},
  {"swap",    &P.swap,    0.0f,  1.0f,   "1 = motor A la banh PHAI, B la banh TRAI"},
  {"lrev",    &P.lrev,    0.0f,  1.0f,   "1 = dao chieu banh TRAI"},
  {"rrev",    &P.rrev,    0.0f,  1.0f,   "1 = dao chieu banh PHAI"},
  {"linerev", &P.linerev, 0.0f,  1.0f,   "1 = D1 nam ben PHAI xe"},
  {"linehigh",&P.linehigh,0.0f,  1.0f,   "1 = line den cho gia tri CAO"},
};
const int PT_COUNT = sizeof(PT) / sizeof(PT[0]);

void setDefaults() {
  // ---- THONG SO CHAY THU AN TOAN CHO CUM CAM BIEN 8 MAT NGAN ----
  P.magic = PARAM_MAGIC;
  P.kp = 0.09f; P.ki = 0.0f; P.kd = 0.45f; P.dtau = 9.0f;
  P.base = 115; P.maxpwm = 191; P.curve = 0.80f; P.rise = 350; P.rev = 0.45f;
  P.dead = 40; P.cspd = 65; P.cerr = 850; P.search = 90; P.lostms = 140; P.losttmo = 1500; P.calspd = 90; P.obscm = 15;
  P.period = 4000; P.filt = 5;
  P.jside = 1; P.jbranch = 0; P.adapt = 0.003f;
  P.calmin = 60; P.nfloor = 250; P.ema = 0.12f; P.amb = 0; P.irus = 300; P.irhigh = 1;
  P.dslow = 1.2f; P.minpwm = 0;
  P.corner = 1; P.tdet = 4; P.tapp = 55; P.tbrk = 45; P.tfwd = 20; P.tspd = 105; P.tappmax = 260;
  P.swap = 1; P.lrev = 0; P.rrev = 0; P.linerev = 1; P.linehigh = 1;
}

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void sanitizeParams() {
  for (int i = 0; i < PT_COUNT; i++) {
    float v = *PT[i].v;
    if (!(v == v)) v = PT[i].lo;                 
    *PT[i].v = clampf(v, PT[i].lo, PT[i].hi);
  }
  P.filt = (float)(int)lroundf(P.filt);
  P.tdet = (float)(int)lroundf(P.tdet);
  P.jside = (P.jside < 0) ? -1.0f : 1.0f;
}

// ===================== LUU / NAP =====================
Preferences prefs;
struct CalData { uint32_t magic; int16_t mn[8]; int16_t mx[8]; };
const uint32_t CAL_MAGIC = 0x43414C33;  

int  raw[8], vmin[8], vmax[8], val[8];
bool calibrated = false;

void saveParams() {
  prefs.begin("xdl2", false);
  prefs.putBytes("p", &P, sizeof(P));
  prefs.end();
}
bool loadParams() {
  bool ok = false;
  prefs.begin("xdl2", false);
  if (prefs.getBytesLength("p") == sizeof(Params)) {
    Params t;
    prefs.getBytes("p", &t, sizeof(t));
    if (t.magic == PARAM_MAGIC) { P = t; ok = true; }
  }
  prefs.end();
  if (ok) sanitizeParams();
  return ok;
}
void saveCal() {
  CalData c; c.magic = CAL_MAGIC;
  for (int i = 0; i < 8; i++) { c.mn[i] = (int16_t)vmin[i]; c.mx[i] = (int16_t)vmax[i]; }
  prefs.begin("xdl2", false);
  prefs.putBytes("c", &c, sizeof(c));
  prefs.end();
}
bool loadCal() {
  bool ok = false;
  prefs.begin("xdl2", false);
  if (prefs.getBytesLength("c") == sizeof(CalData)) {
    CalData c;
    prefs.getBytes("c", &c, sizeof(c));
    if (c.magic == CAL_MAGIC) {
      ok = true;
      for (int i = 0; i < 8; i++) {
        vmin[i] = c.mn[i]; vmax[i] = c.mx[i];
        if (vmax[i] - vmin[i] < (int)P.calmin) ok = false;
      }
    }
  }
  prefs.end();
  return ok;
}

// ===================== PWM (core 2.x va 3.x) =====================
const int PWM_FREQ = 20000, PWM_RES = 8;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmSetupPin(int pin, int ch) { (void)ch; ledcAttach(pin, PWM_FREQ, PWM_RES); }
  void pwmWritePin(int pin, int ch, int duty) { (void)ch; ledcWrite(pin, duty); }
#else
  void pwmSetupPin(int pin, int ch) { ledcSetup(ch, PWM_FREQ, PWM_RES); ledcAttachPin(pin, ch); }
  void pwmWritePin(int pin, int ch, int duty) { (void)pin; ledcWrite(ch, duty); }
#endif
const int CH_A = 0, CH_B = 1;

// ===================== MOTOR =====================
float lastL = 0, lastR = 0;     // lenh cuoi (banh logic trai/phai)

void motorRaw(int in1, int in2, int pwm, int ch, int speed) {
  if (speed > PWM_HARD_MAX) speed = PWM_HARD_MAX;
  if (speed < -PWM_HARD_MAX) speed = -PWM_HARD_MAX;
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  pwmWritePin(pwm, ch, speed < 0 ? -speed : speed);
}

// bu vung chet: lenh khac 0 duoc doi sang [minpwm..255]
float deadComp(float v) {
  if (P.minpwm < 1 || fabsf(v) < 1.0f) return v;
  float a = P.minpwm + fabsf(v) * (1.0f - P.minpwm / 255.0f);
  return v > 0 ? a : -a;
}
// left/right: banh logic TRAI/PHAI, duong = xe tien
void setMotors(float left, float right) {
  lastL = left; lastR = right;
  int l = (int)lroundf(deadComp(left)), r = (int)lroundf(deadComp(right));
  if (P.lrev > 0.5f) l = -l;
  if (P.rrev > 0.5f) r = -r;
  if (P.swap > 0.5f) { int t = l; l = r; r = t; }
  motorRaw(PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_A, l);   // motor A
  motorRaw(PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_B, r);   // motor B
}
// phanh ngan mach TB6612 (IN1 = IN2 = HIGH): dung nhanh hon tha troi rat nhieu
void motorBrake() {
  lastL = lastR = 0;
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  pwmWritePin(PIN_PWMA, CH_A, 255);
  pwmWritePin(PIN_PWMB, CH_B, 255);
}
void motorsEnable(bool on) {
  if (!on) setMotors(0, 0);
  digitalWrite(PIN_STBY, on ? HIGH : LOW);
}

// ===================== MACH DO LINE =====================
uint32_t scanUsLast = 0;

static inline int med3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;
}

// ---- LED IR (chi khi amb = 1) ----
bool irReady = false;
void irSet(bool on) {
  if (!irReady) { pinMode(PIN_IR, OUTPUT); irReady = true; }
  digitalWrite(PIN_IR, (on == (P.irhigh > 0.5f)) ? HIGH : LOW);
}

static inline int readPinFiltered(int pin, int n) {
  if (n <= 1) return analogRead(pin);
  long s = 0; int mn = 4095, mx = 0;
  for (int k = 0; k < n; k++) {
    int a = analogRead(pin);
    s += a; if (a < mn) mn = a; if (a > mx) mx = a;
  }
  if (n >= 4) return (int)((s - mn - mx) / (n - 2));
  return (int)(s / n);
}
void readScan(int *dst) {
  int n = (int)P.filt;
  for (int p = 0; p < 8; p++) dst[p] = readPinFiltered(PIN_LINE[p], n);
}

float rawF[8];
bool rawInit = false;
void readRaw() {
  uint32_t t0 = micros();
  int x[8];
  if (P.amb > 0.5f) {
    int on[8], off[8];
    irSet(true);  delayMicroseconds((uint32_t)P.irus); readScan(on);
    irSet(false); delayMicroseconds((uint32_t)P.irus); readScan(off);
    irSet(true);
    for (int p = 0; p < 8; p++) {
      int d = on[p] - off[p] + 2048;            
      x[p] = d < 0 ? 0 : (d > 4095 ? 4095 : d);
    }
  } else {
    readScan(x);
  }
  float e = P.ema;
  for (int p = 0; p < 8; p++) {
    if (!rawInit || e <= 0.001f) rawF[p] = (float)x[p];
    else rawF[p] = e * rawF[p] + (1.0f - e) * (float)x[p];
    raw[p] = (int)lroundf(rawF[p]);
  }
  rawInit = true;
  scanUsLast = micros() - t0;
}

float bOff[8] = {0};       
int vnorm[8];             
void resetAdapt() { for (int p = 0; p < 8; p++) bOff[p] = 0; }
int nOn = 0, nBlack = 0;
float pos = 3500;         

bool computeLine() {
  long weighted = 0, sum = 0;
  nOn = 0; nBlack = 0;
  bool rev = P.linerev > 0.5f;
  for (int i = 0; i < 8; i++) {          
    int p = rev ? 7 - i : i;
    long range = vmax[p] - vmin[p];
    long v = (range > 0) ? (long)(raw[p] - (long)bOff[p] - vmin[p]) * 1000 / range : 0;
    vnorm[i] = (int)(P.linehigh < 0.5f ? 1000 - v : v);
    if (v < 0) v = 0;
    if (v > 1000) v = 1000;
    if (P.linehigh < 0.5f) v = 1000 - v;

    long nf = (long)P.nfloor;
    if (v <= nf) v = 0; else v = (v - nf) * 1000 / (1000 - nf);
    val[i] = (int)v;
    if (v > 200) nOn++;
    if (v > 700) nBlack++;
    if (v > 50) { weighted += v * (i * 1000L); sum += v; }
  }
  if (nOn == 0 || sum == 0) return false;
  pos = (float)weighted / (float)sum;
  return true;
}

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
// 3 lan do lien tiep deu thay vat can -> dung; 3 lan deu da xa -> chay tiep
bool obstacleNear()  { return P.obscm >= 1 && distMax3() < P.obscm; }
bool obstacleClear() { return P.obscm < 1 || distMin3() > P.obscm + 5; }

// ===================== TRANG THAI =====================
enum State { ST_IDLE, ST_READY, ST_RUN, ST_OBSTACLE };
State state = ST_IDLE;
int watchMode = 0;

// PID / lai
float lastPos = 3500, dFilt = 0, iAcc = 0, baseNow = 0, lastCorr = 0;
uint32_t lastTickUs = 0, nextTick = 0, lostSinceMs = 0;
bool lostActive = false;
bool pidInit = false;      // false = tick dau tien: khong tinh D (tranh giat banh)
int  edgeSide = 0;         // -1/+1: lan cuoi mat NGOAI CUNG trai/phai thay line (de biet goc re ben nao)
uint32_t edgeMs = 0;
int  pivotSide = 0;        // huong quay tim line, KHOA tu luc mat line den luc tim lai o giua

// ---- che do goc 90 do ----
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
  // D4/D5 nhay nhat; D3/D6 can muc tin hieu cao hon.
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

// ===================== HIEU CHINH =====================
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
    // quay ve giua line
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

// ===================== TU BU NEN =====================
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

// ===================== GOC 90 DO =====================
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

// ===================== VONG DIEU KHIEN =====================
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

  // vat can
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

    // ---- toc do nen: giam khi lech nhieu, tang tu tu sau cua ----
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

// ===================== HIEN THI =====================
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
  // thong so?
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

// ===================== NUT BOOT =====================
// 1 = bam ngan, 2 = giu lau
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
