---------------------------------------------------------------------------------
/* MODULE-2: UNO R4 WiFi Robot Driver
   - PCA9685 on I2C (A4 SDA, A5 SCL)
   - Joysticks: A0..A3
   - TCP client to ESP32 AP (192.168.4.1:3333)
   - Save/Run/Reset controlled by CSV values from glove
*/

#include <WiFiS3.h>               
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// -------- CONFIG --------
const char* WIFI_SSID = "GENOMXZ";
const char* WIFI_PASS = "12345678";
IPAddress ESP32_IP(192,168,4,1);
const uint16_t ESP32_PORT = 3333;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const int PWM_FREQ = 50;
const int SERVOMIN = 102;
const int SERVOMAX = 512;

// channel mapping
const uint8_t CH_BASE=0, CH_SHOULDER=1, CH_ELBOW=2, CH_CLAW=3;

// angle ranges and defaults per spec
const int BASE_MIN=0, BASE_MAX=180;
const int SHOULDER_MIN=120, SHOULDER_MAX=160;
const int ELBOW_MIN=130, ELBOW_MAX=160;
const int CLAW_MIN=0, CLAW_MAX=15;

const int DEFAULT_BASE_ANGLE = 90;
const int DEFAULT_SHOULDER_ANGLE = 135;
const int DEFAULT_ELBOW_ANGLE = 140;
const int DEFAULT_CLAW_ANGLE = 0;

// joystick pins and thresholds
const uint8_t JOY1_VRX_PIN = A0;
const uint8_t JOY1_VRY_PIN = A1;
const uint8_t JOY2_VRX_PIN = A2;
const uint8_t JOY2_VRY_PIN = A3;
const int JOY_ACTIVE_THRESHOLD = 80;
const unsigned long OVERRIDE_IDLE_TIMEOUT_MS = 500; // ms

// LED pins (GPIO) — change to the pins you have wired on UNO R4
const int LED_CONN_PIN = 4;               
const int LED_BORDER_PINS[] = {3,2,5,6};  
const int LED_INNER_PINS[]  = {7,8,9,10}; 
const int LED_COUNT_PINS[]  = {11,12,13}; 
const int NUM_BORDER = sizeof(LED_BORDER_PINS)/sizeof(LED_BORDER_PINS[0]);
const int NUM_INNER  = sizeof(LED_INNER_PINS)/sizeof(LED_INNER_PINS[0]);
const int NUM_COUNT = sizeof(LED_COUNT_PINS)/sizeof(LED_COUNT_PINS[0]);

// -------- STATE --------
WiFiClient client;
bool tcpConnected = false;
int baseAngle = DEFAULT_BASE_ANGLE;
int shoulderAngle = DEFAULT_SHOULDER_ANGLE;
int elbowAngle = DEFAULT_ELBOW_ANGLE;
int clawAngle = DEFAULT_CLAW_ANGLE;

bool overrideMode = false;
unsigned long lastJoystickMoveMs = 0;
unsigned long lastConnBlinkMs = 0;
bool connBlinkState = false;

// TCP RX buffer
char rxBuf[256];
int rxIdx = 0;

// Save/run/reset storage
#define MAX_POSITIONS 30
struct Pose { int b,s,e,c; };
Pose savedPos[MAX_POSITIONS];
int savedCount = 0;
bool runningSequence = false;
bool loopSequence = false;

// sequence playback interpolation parameters
const unsigned long STEP_MS = 40;      
const int INTERP_STEPS = 25;         

// helper: set servo from degree
int degToPulse(int deg) { return map(constrain(deg,0,180), 0, 180, SERVOMIN, SERVOMAX); }

// apply current angles to PCA9685
void applyAllServos() {
  pwm.setPWM(CH_BASE, 0, degToPulse(baseAngle));
  pwm.setPWM(CH_SHOULDER, 0, degToPulse(shoulderAngle));
  pwm.setPWM(CH_ELBOW, 0, degToPulse(elbowAngle));
  pwm.setPWM(CH_CLAW, 0, degToPulse(clawAngle));
}

// show savedCount on LED_COUNT_PINS as binary
void showSavedCountOnLEDs() {
  int v = savedCount;
  for (int i=0;i<NUM_COUNT;i++){
    digitalWrite(LED_COUNT_PINS[i], (v & (1<<i)) ? HIGH : LOW);
  }
}

// override LED border + inner blinking
void overrideLEDPattern(bool innerOn) {
  for (int i=0;i<NUM_BORDER;i++) digitalWrite(LED_BORDER_PINS[i], HIGH);
  for (int i=0;i<NUM_INNER;i++) digitalWrite(LED_INNER_PINS[i], innerOn?HIGH:LOW);
}
void clearOverrideLEDs() {
  for (int i=0;i<NUM_BORDER;i++) digitalWrite(LED_BORDER_PINS[i], LOW);
  for (int i=0;i<NUM_INNER;i++) digitalWrite(LED_INNER_PINS[i], LOW);
}

// blink connect LED while disconnected
void blinkConnLED() {
  unsigned long now = millis();
  if (now - lastConnBlinkMs > 400) {
    lastConnBlinkMs = now;
    connBlinkState = !connBlinkState;
    digitalWrite(LED_CONN_PIN, connBlinkState ? HIGH : LOW);
  }
}

// ensure TCP connection to ESP32 (non-blocking)
void ensureTCP() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    tcpConnected = false;
    client.stop();
    return;
  }
  if (tcpConnected && client && client.connected()) return;
  static unsigned long lastTry = 0;
  if (millis() - lastTry < 1500) return;
  lastTry = millis();
  if (client.connect(ESP32_IP, ESP32_PORT)) {
    tcpConnected = true;
    digitalWrite(LED_CONN_PIN, HIGH);
    Serial.println("TCP connected");
  } else {
    tcpConnected = false;
    client.stop();
    digitalWrite(LED_CONN_PIN, LOW);
  }
}

// parse CSV: base,shoulder,elbow,claw,save,run,reset
void parseAndApplyCSV(const String &line) {
  String s = line;
  s.trim();
  int vals[7] = {0};
  int idx=0, start=0;
  for (int i=0;i<=s.length() && idx<7;i++) {
    if (i==s.length() || s.charAt(i)==',') {
      String tok = s.substring(start, i);
      tok.trim();
      vals[idx++] = tok.toInt();
      start = i+1;
    }
  }
  int baseV = vals[0], shoulderV = vals[1], elbowV = vals[2], clawV = vals[3];
  int saveV = vals[4], runV = vals[5], resetV = vals[6];

  // RESET command: clear saved poses immediately
  if (resetV == 1) {
    savedCount = 0;
    runningSequence = false;
    loopSequence = false;
    showSavedCountOnLEDs();
    Serial.println("RESET -> cleared saved poses");
    return;
  }

  // SAVE command
  if (saveV == 1) {
    if (savedCount < MAX_POSITIONS) {
      savedPos[savedCount++] = { baseAngle, shoulderAngle, elbowAngle, clawAngle };
      showSavedCountOnLEDs();
      Serial.print("Saved pose #"); Serial.println(savedCount);
    } else {
      Serial.println("Saved: memory full");
    }
    return;
  }

  // RUN command: if clawV==1 at same time -> loop
  if (runV == 1) {
    if (savedCount == 0) { Serial.println("Run requested but no saved poses"); return; }
    loopSequence = (clawV == 1);
    runningSequence = true;
    Serial.println(loopSequence ? "RUN loop ON" : "RUN once");
    return;
  }

  // If override not active, apply remote increments per your rules
  if (!overrideMode) {
    bool baseChanged = false;
    bool shoulderChanged = false;

    // Base: +1 / -1 per tick
    if (baseV == 1) { baseAngle = constrain(baseAngle + 1, BASE_MIN, BASE_MAX); baseChanged = true; }
    else if (baseV == 2) { baseAngle = constrain(baseAngle - 1, BASE_MIN, BASE_MAX); baseChanged = true; }

    // Shoulder only if base not changed this cycle
    if (!baseChanged) {
      if (shoulderV == 1) { shoulderAngle = constrain(shoulderAngle + 1, SHOULDER_MIN, SHOULDER_MAX); shoulderChanged = true; }
      else if (shoulderV == 2) { shoulderAngle = constrain(shoulderAngle - 1, SHOULDER_MIN, SHOULDER_MAX); shoulderChanged = true; }
    }

    // Elbow only if shoulder not changed this cycle
    if (!shoulderChanged) {
      if (elbowV == 1) elbowAngle = constrain(elbowAngle + 1, ELBOW_MIN, ELBOW_MAX);
      else if (elbowV == 2) elbowAngle = constrain(elbowAngle - 1, ELBOW_MIN, ELBOW_MAX);
    }

    // Claw direct set
    clawAngle = (clawV == 1) ? CLAW_MIN : CLAW_MAX;
  }
}

// joystick override mapping; called every loop, independent of WiFi
void handleJoysticks() {
  int j1x = analogRead(JOY1_VRX_PIN);
  int j1y = analogRead(JOY1_VRY_PIN);
  int j2x = analogRead(JOY2_VRX_PIN);
  int j2y = analogRead(JOY2_VRY_PIN);
  const int center = 512;
  bool moved = (abs(j1x - center) > JOY_ACTIVE_THRESHOLD) || (abs(j1y - center) > JOY_ACTIVE_THRESHOLD)
            || (abs(j2x - center) > JOY_ACTIVE_THRESHOLD) || (abs(j2y - center) > JOY_ACTIVE_THRESHOLD);

  if (moved) {
    lastJoystickMoveMs = millis();
    if (!overrideMode) {
      overrideMode = true;
      runningSequence = false; // interrupt sequence
      Serial.println("OVERRIDE engaged");
    }
    // map joystick positions to ranges (linear)
    baseAngle = constrain(map(j1x, 0, 1023, BASE_MAX, BASE_MIN), BASE_MIN, BASE_MAX);
    shoulderAngle = constrain(map(j1y, 0, 1023, SHOULDER_MIN, SHOULDER_MAX), SHOULDER_MIN, SHOULDER_MAX);
    elbowAngle = constrain(map(j2x, 0, 1023, ELBOW_MIN, ELBOW_MAX), ELBOW_MIN, ELBOW_MAX);
    clawAngle = constrain(map(j2y, 0, 1023, CLAW_MIN, CLAW_MAX), CLAW_MIN, CLAW_MAX);

    // override LED pattern
    overrideLEDPattern((millis() / 250) % 2 == 0);
  }
}

// smooth interpolation between current angles and target angles
void interpolateToTarget(int targetB, int targetS, int targetE, int targetC, int steps = INTERP_STEPS, unsigned long stepMs = STEP_MS) {
  int startB = baseAngle;
  int startS = shoulderAngle;
  int startE = elbowAngle;
  int startC = clawAngle;
  for (int i=1; i<=steps; ++i) {
    // if override was re-engaged, abort interpolation
    if (overrideMode) return;
    baseAngle = startB + ((targetB - startB) * i) / steps;
    shoulderAngle = startS + ((targetS - startS) * i) / steps;
    elbowAngle = startE + ((targetE - startE) * i) / steps;
    clawAngle = startC + ((targetC - startC) * i) / steps;
    applyAllServos();
    unsigned long t0 = millis();
    while (millis() - t0 < stepMs) {
      // allow joysticks to take over immediately
      handleJoysticks();
      if (overrideMode) return;
      delay(2);
    }
  }
  // finalize
  baseAngle = targetB; shoulderAngle = targetS; elbowAngle = targetE; clawAngle = targetC;
  applyAllServos();
}

// run saved sequence with interpolation
void runSavedSequenceOnceOrLoop() {
  if (savedCount == 0) return;
  int idx = 0;
  while (true) {
    // if overridden by joystick, stop playback
    if (overrideMode) { runningSequence = false; Serial.println("Sequence interrupted by override"); return; }
    // go to pose idx with interpolation
    Pose &p = savedPos[idx];
    interpolateToTarget(p.b, p.s, p.e, p.c);
    idx++;
    if (idx >= savedCount) {
      if (loopSequence) idx = 0;
      else break;
    }
  }
  runningSequence = false;
  Serial.println("Sequence finished");
}

// non-blocking wrapper: step-by-step runSequence with interpolation per pose
void runSequenceNonBlocking() {
  static int seqIdx = 0;
  static bool busy = false;
  static unsigned long lastInterpEnd = 0;

  if (!runningSequence) { seqIdx = 0; busy = false; return; }
  if (overrideMode) { runningSequence = false; seqIdx = 0; busy = false; return; }

  if (!busy) {
    // start interpolation to savedPos[seqIdx]
    if (seqIdx >= savedCount) {
      if (loopSequence) seqIdx = 0;
      else { runningSequence = false; seqIdx = 0; Serial.println("Sequence done"); return; }
    }
    // perform interpolation (blocking small loop but allows joystick check inside)
    Pose &p = savedPos[seqIdx];
    interpolateToTarget(p.b, p.s, p.e, p.c);
    busy = true;
    lastInterpEnd = millis();
  } else {
    // interpolation finished for this pose -> advance after small pause
    if (millis() - lastInterpEnd > 250) {
      seqIdx++;
      busy = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(2);

  // LED pins
  pinMode(LED_CONN_PIN, OUTPUT);
  digitalWrite(LED_CONN_PIN, LOW);
  for (int i=0;i<NUM_BORDER;i++){ pinMode(LED_BORDER_PINS[i], OUTPUT); digitalWrite(LED_BORDER_PINS[i], LOW); }
  for (int i=0;i<NUM_INNER;i++){ pinMode(LED_INNER_PINS[i], OUTPUT); digitalWrite(LED_INNER_PINS[i], LOW); }
  for (int i=0;i<NUM_COUNT;i++){ pinMode(LED_COUNT_PINS[i], OUTPUT); digitalWrite(LED_COUNT_PINS[i], LOW); }

  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // initial hold pose
  baseAngle = DEFAULT_BASE_ANGLE;
  shoulderAngle = DEFAULT_SHOULDER_ANGLE;
  elbowAngle = DEFAULT_ELBOW_ANGLE;
  clawAngle = DEFAULT_CLAW_ANGLE;
  applyAllServos();

  // WiFi try connect non-blocking;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
  ensureTCP();
  // always allow joystick override (even if WiFi down)
  handleJoysticks();

  if (!tcpConnected) {
    blinkConnLED();
    applyAllServos();
    delay(5);
    return;
  }

  // read incoming TCP
  while (client.available()) {
    char c = client.read();
    if (c == '\n' || rxIdx >= (int)sizeof(rxBuf)-2) {
      rxBuf[rxIdx] = '\0';
      String line = String(rxBuf);
      rxIdx = 0;
      if (line.length() > 0) parseAndApplyCSV(line);
    } else {
      rxBuf[rxIdx++] = c;
    }
  }

  // if running sequence, run non-blocking interpolation per-pose
  if (runningSequence) runSequenceNonBlocking();

  // if override timed out, smoothly return to default hold pose
  if (overrideMode && millis() - lastJoystickMoveMs > OVERRIDE_IDLE_TIMEOUT_MS) {
    overrideMode = false;
    Serial.println("Joystick idle -> smooth return to default");
    interpolateToTarget(DEFAULT_BASE_ANGLE, DEFAULT_SHOULDER_ANGLE, DEFAULT_ELBOW_ANGLE, DEFAULT_CLAW_ANGLE);
    clearOverrideLEDs();
  }

  applyAllServos();
  delay(5);
}