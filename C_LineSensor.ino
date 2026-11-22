
uint32_t scanUsLast = 0;

static inline int med3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;
}


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

