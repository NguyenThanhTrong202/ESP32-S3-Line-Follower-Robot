
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

