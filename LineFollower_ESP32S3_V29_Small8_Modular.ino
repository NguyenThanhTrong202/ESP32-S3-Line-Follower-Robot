/*
 * XE DO LINE ESP32-S3 - V2.9-SMALL8 (= V2.8-B150, toi uu cho cum QTR-8A ngan / line hep)
 * Viet lai dua tren goi mau LineFollower_MPU6050 (khong WiFi, khong MPU, khong ToF).
 * ----------------------------------------------------------------------------
 * Phan cung: ESP32-S3, TB6612FNG, mach do line 8 mat analog (D1..D8), HC-SR04,
 *            2 motor N20, pin 2S 7,4V + mach ha ap 5V cap cho ESP32-S3.
 *
 * SO DO CHAN (giu nguyen day hien tai):
 *   Mach do line : VCC=3V3, D8..D1 = GPIO 4,5,6,7,15,16,17,18 (D1=18 ... D8=4), GND. Chan IR de ho.
 *   TB6612FNG    : PWMA=13 AIN2=12 AIN1=11 STBY=10 BIN1=9 BIN2=46 PWMB=14
 *                  VM = +7,4V pin, VCC logic = 3V3, GND chung
 *   HC-SR04      : TRIG=48 ECHO=47, VCC=5V, GND chung
 *                  (Echo ra 5V: nen qua cau chia ap 1k noi tiep + 2k xuong GND de bao ve GPIO47)
 *   Nut BOOT     : GPIO0 = bat dau / dung
 *
 * NUT BOOT:
 *   Bam ngan lan 1 -> HIEU CHINH (xe tu xoay trai/phai ~4 giay tren line)
 *   Bam ngan lan 2 -> CHAY.   Bam khi dang chay -> DUNG.
 *   Giu nut > 0,8 giay luc dung -> hieu chinh lai.
 *   Hieu chinh duoc luu vao bo nho: bat nguon lai khong can hieu chinh (doi anh sang/san thi hieu chinh lai).
 *
 * CHINH THONG SO KHONG CAN NAP LAI: mo Serial Monitor 115200, chon "Newline", go lenh:
 *   help            xem het lenh
 *   kp 0.09         doi thong so (ten thong so + gia tri). Go "kp" de xem gia tri hien tai
 *   save            luu thong so vao bo nho (tat nguon van con)
 *   testl / testr   quay thu banh TRAI / PHAI (nang xe len cao truoc khi thu)
 *   swap 1 | lrev 1 | rrev 1 | linerev 1   sua huong lap rap ngay, khong sua code
 *   watch 1         bat hien thi truc tiep (vi tri line, banh, vat can). Tat (watch 0) khi chay khong cam USB
 *   dump            in nhat ky 3 giay cuoi cua lan chay vua roi
 *   timing          xem toc do vong dieu khien thuc te
 *
 * MOI TRONG V2.6:
 *   - Che do GOC 90 DO rieng: thay line nam ngang ve 1 phia -> phanh -> di thang qua goc
 *     -> quay tai cho den khi mat giua bat line -> phanh dung quay -> chay tiep.
 *     Thong so: corner tdet tapp tbrk tfwd tspd tappmax
 *   - Giam toc theo TOC DO lech (dslow): line bat dau troi la giam ngay, khong doi lech nhieu.
 *   - Bu vung chet motor (minpwm): banh trong cua khong bi "dung im" khi lenh PWM nho.
 *   - Nhat ky 'dump' co them cot che do: G = dang vao goc, Q = dang quay, M = mat line.
 *
 * MOI TRONG V2.7 (san nhieu, line chi chenh ~100 so voi nen):
 *   - Hieu chinh chap nhan chenh lech nho: 'calmin' (mac dinh 60, V2.6 bat buoc 300).
 *   - Loc manh hon: 'filt' = so mau moi mat (1..16, bo min/max roi lay trung binh) + 'ema' loc theo thoi gian.
 *   - San nhieu 'nfloor': gia tri chuan hoa duoi nguong nay coi nhu NEN (bat nham line vi nhieu).
 *   - Tru anh sang moi truong 'amb 1': doc khi bat LED IR va khi tat, lay hieu so.
 *     Can noi chan IR cua mach do line vao GPIO21 (hien dang de ho). Khong noi thi de amb 0.
 *   - Lenh 'noise': dat xe dung yen, do nhieu tung mat va ty le tin hieu/nhieu (SNR).
 *
 * MOI TRONG V2.8 (theo so do duong dua tren bat hiflex):
 *   - O VUONG DAC / NGA TU / VACH NGANG (ca 2 phia deu thay line): di thang qua (che do B).
 *     Qua roi ma phia truoc trang -> nga ba chu T -> re theo 'jside' (1 phai, -1 trai).
 *   - Nhanh re mot phia ma line thang van tiep tuc: 'jbranch' 0 = di thang, 1 = re vao nhanh.
 *   - Dut line (mui ten xanh, o vuong cam co the "tang hinh" duoi hong ngoai): giu thang 'lostms'.
 *     Neu mat ngoai cung vua thay line (dinh zigzag, goc gat) thi KHONG giu ma quay ngay.
 *   - Tu bu nen 'adapt': bat hiflex bong + anh sang khong deu doc duong -> moi mat tu chinh muc nen khi dang chay.
 *   LUU Y: doi cau truc thong so -> thong so da 'save' truoc day bi bo, ve mac dinh (hieu chinh van giu).
 *
 * MOI TRONG V2.9-SMALL8:
 *   - Phat hien goc bang 1 mat ngoai tin hieu manh, xac nhan lien tiep 3 mau.
 *   - Goc hep: di thang rat ngan roi quay, khong doi 4 mat lien tiep.
 *   - Quay dung khi 2 mau lien tiep thay line giua, co thoi gian quay toi thieu.
 *   - Giam base / maxpwm / toc do quay de test tren cum 8 mat ngan.
 *   - PARAM_MAGIC doi -> cai dat cu khong tu nap (hieu chinh luu van giu).
 *
 * Nap code: Arduino IDE, board "ESP32S3 Dev Module", core esp32 2.x hoac 3.x.
 * Neu Serial Monitor trong: Tools -> USB CDC On Boot -> Enabled. RUT PIN truoc khi cam USB nap code.
 * ----------------------------------------------------------------------------
 */
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


const int PIN_PWMA = 13, PIN_AIN2 = 12, PIN_AIN1 = 11, PIN_STBY = 10;
const int PIN_BIN1 = 9,  PIN_BIN2 = 46, PIN_PWMB = 14;      
const int PIN_LINE[8] = {18, 17, 16, 15, 7, 6, 5, 4};        
const int PIN_TRIG = 48, PIN_ECHO = 47;
const int PIN_BUTTON = 0;
const int PIN_IR = 21;       

const int   PWM_HARD_MAX = 255;     // tran tuyet doi (8 bit). Motor 6V chay pin 7,4-8,4V: 255 = qua ap ~40%, nong + giam tuoi tho. Mac dinh maxpwm van la 191


const uint32_t PARAM_MAGIC = 0x58444C3C;   
struct Params {
  uint32_t magic;
  float kp, ki, kd, dtau;           // PID; dtau = hang so loc D (ms)
  float base, maxpwm, curve, rise, rev;   // toc do nen, PWM toi da, giam toc vao cua, toc do tang toc, banh trong cua quay nguoc
  float search, lostms, losttmo, calspd, obscm;
  float period, filt;               
  float dead;                       
  float cspd, cerr;                 
  float dslow, minpwm;              // giam toc theo toc do lech; bu vung chet motor
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

