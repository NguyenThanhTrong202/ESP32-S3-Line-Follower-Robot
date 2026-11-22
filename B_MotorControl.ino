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
float lastL = 0, lastR = 0;     

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

void setMotors(float left, float right) {
  lastL = left; lastR = right;
  int l = (int)lroundf(deadComp(left)), r = (int)lroundf(deadComp(right));
  if (P.lrev > 0.5f) l = -l;
  if (P.rrev > 0.5f) r = -r;
  if (P.swap > 0.5f) { int t = l; l = r; r = t; }
  motorRaw(PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_A, l);   // motor A
  motorRaw(PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_B, r);   // motor B
}

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

