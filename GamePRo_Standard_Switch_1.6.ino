// GamePRo Switch Sketch v1.6
// Special thanks to Westini who helped tidy the original sketch.
//
// Changes from v1.5:
//   - Joystick in any direction and at any strength: 'J' + 4 bytes
//     (angle low byte, angle high byte, magnitude 0-100 %, duration in
//     10 ms units; duration 0 = hold, magnitude 0 = centre). Angle 0 = up,
//     90 = right (clockwise). Up/Down/Left/Right are 0/90/180/270 at 100 %.
//     Each axis is scaled to its own calibrated limits, so partial pushes
//     follow the calibration even if a servo's travel is not symmetric.
//   - No more jams when changing a held direction (e.g. hold Up, then hold
//     Right): the stick now glides both servos together from the held
//     position to the new one, instead of snapping one axis home while the
//     other pushes out. A press after a hold waits for the stick to centre.
//   - 'V' returns the firmware version line (no ACK), so the app can tell
//     whether 'J' is supported before sending it.
//   - 'i' (set bearing for 'r' / '9') no longer repeats the previous
//     command. 'r' / '9' keep their behaviour (default bearing 0 = up).
//
// Changes from v1.4:
//   - releaseAll() is now called at the start of EVERY command (D-pad
//     press, NE joystick tap/hold, A/B/X/Y, Soft Reset 1/2, +/Wonder
//     Trade), matching the 3DS/Standard sketch. Previously only the
//     D-pad hold commands and RELEASE_ALL did this, so e.g. holding
//     Right and then pressing B left Right held instead of releasing it.
//   - Every command now sends the ACK byte ('K') when it finishes,
//     matching the 3DS/Standard sketch. Previously only WRITE_CALIBRATION
//     sent ACK, so the Python app's ACK-wait (_send(), 1s timeout) was
//     stalling a full second on every single button command instead of
//     returning as soon as the servo move actually completed.
//
// Changes from v1.3:
//   - Hold commands (8/2/4/6) now call releaseAll() before moving the servo.
//     Previously, sending HOLD_RIGHT then HOLD_UP would leave the right servo
//     held, resulting in both directions being held simultaneously.
//
// Changes from v1.2:
//   - Servo pin assignments stored in EEPROM (fields 25-29 in the CSV).
//     The Python app can now change which Arduino pin each servo is wired
//     to without reflashing firmware.  Pins take effect immediately and
//     survive power cycles.
//   - 'q' now returns 30 comma-separated values (25 calibration + 5 pins).
//   - 'w' accepts 25 (legacy) or 30 values; 30-value writes update pins
//     and reattach servos live.
//   - Old EEPROM data (v1.2) still loads correctly — pin fields default to
//     the hardcoded values below when the EEPROM bytes are out of range.
//
// Note: 'r' is still used for NE_TAP (diagonal joystick tap) — calibration
// read uses 'q' to avoid this conflict.
// Note: myservo_6 (NE joystick, pin 9) is not included in pin assignments
// as it uses computed angles rather than fixed positions.

#include <Servo.h>
#include <EEPROM.h>

#define PI 3.1415926535897932384626433832795

#define READ_LIGHT           'B'
#define READ_LIGHT_ADVANCED  'C'
#define HOLD_UP              '8'
#define HOLD_DOWN            '2'
#define HOLD_LEFT            '4'
#define HOLD_RIGHT           '6'
#define PRESS_UP             'e'
#define PRESS_DOWN           'd'
#define PRESS_LEFT           's'
#define PRESS_RIGHT          'f'
#define BUTTON_A             'a'
#define BUTTON_B             'b'
#define BUTTON_X             'x'
#define BUTTON_Y             'y'
#define RELEASE_ALL          '0'
#define SOFT_RESET_1         'S'
#define SOFT_RESET_2         'Z'
#define WONDER_TRADE         'W'
#define NE_Hold              '9'
#define NE_TAP               'r'
#define STICK                'J'   // + angle(2 bytes LE), magnitude(0-100), duration(x10 ms, 0 = hold)
#define VERSION_QUERY        'V'   // replies FW_VERSION + \n (no ACK)
#define ACK                  'K'   // sent back to Python when a command is complete
#define READ_CALIBRATION     'q'   // sends calibration CSV + \n (no ACK)
#define WRITE_CALIBRATION    'w'   // receives calibration CSV, saves to EEPROM, sends ACK

#define FW_VERSION      "GamePRo Switch 1.6"
#define GLIDE_STEP_DEG  2     // servo degrees per glide step when changing a held direction
#define GLIDE_STEP_MS   6     // ms per glide step
#define STICK_SETTLE_MS 150   // time for the stick to return to centre before a new press

// ── EEPROM calibration ────────────────────────────────────────────────────────
#define EEPROM_MAGIC 0xA7
#define EEPROM_ADDR  0

struct CalibData {
  byte magic;
  int  leftMax,       rightMax,      leftrightOff;
  int  upMax,         downMax,       updownOff;
  int  YMax,          AMax,          AYOff;
  int  BMax,          XMax,          BXOff;
  int  SRMax,         SROff,         WMax;
  int  arrowBackoffU, arrowBackoffD, arrowBackoffL, arrowBackoffR;
  int  arrow_releaseU, arrow_releaseD, arrow_releaseL, arrow_releaseR;
  int  button_release, prePress;
  // v1.3: servo pin assignments (fields 25-29 in CSV)
  // Order: BX(pin2), AY(pin3), SR(pin6), LR(pin7), UD(pin8)
  byte pinBX, pinAY, pinSR, pinLR, pinUD;
};

// ── Servos ────────────────────────────────────────────────────────────────────
Servo myservo_1;   // Left / Right
Servo myservo_2;   // Up / Down
Servo myservo_3;   // A / Y
Servo myservo_4;   // B / X
Servo myservo_5;   // Soft Reset / Home / + (Wonder Trade)
Servo myservo_6;   // NE joystick (diagonal) — pin hardcoded to 9

// ── Calibration values (defaults — overwritten from EEPROM on boot if saved) ──

int leftMax      = 60;
int rightMax     = 120;
int leftrightOff = 90;

int upMax     = 60;
int downMax   = 120;
int updownOff = 90;

int YMax  = 100;
int AMax  = 80;
int AYOff = 90;

int BMax  = 80;
int XMax  = 100;
int BXOff = 90;

int SRMax = 60;
int SROff = 90;
int WMax  = 120;

int arrowBackoffU = 0;
int arrowBackoffD = 0;
int arrowBackoffL = 0;
int arrowBackoffR = 0;

int arrow_releaseU = 100;
int arrow_releaseD = 100;
int arrow_releaseL = 100;
int arrow_releaseR = 100;
int button_release = 120;
int prePress       = 200;

// v1.3: Servo pin assignments (defaults match hardcoded v1.2 pins)
byte pinLR = 7;   // myservo_1 — Left / Right
byte pinUD = 8;   // myservo_2 — Up / Down
byte pinAY = 3;   // myservo_3 — A / Y
byte pinBX = 2;   // myservo_4 — B / X
byte pinSR = 6;   // myservo_5 — Soft Reset / Home / +

double theta  = 0;
int    bearing = 0;
int    letter  = 0;

// Where the two joystick servos were last sent (they are the D-pad servos)
int stickX = 90;
int stickY = 90;


// ── EEPROM helpers ────────────────────────────────────────────────────────────

void loadCalibration() {
  CalibData cal;
  EEPROM.get(EEPROM_ADDR, cal);
  if (cal.magic != EEPROM_MAGIC) return;

  leftMax       = cal.leftMax;       rightMax      = cal.rightMax;
  leftrightOff  = cal.leftrightOff;
  upMax         = cal.upMax;         downMax       = cal.downMax;
  updownOff     = cal.updownOff;
  YMax          = cal.YMax;          AMax          = cal.AMax;
  AYOff         = cal.AYOff;
  BMax          = cal.BMax;          XMax          = cal.XMax;
  BXOff         = cal.BXOff;
  SRMax         = cal.SRMax;         SROff         = cal.SROff;
  WMax          = cal.WMax;
  arrowBackoffU = cal.arrowBackoffU; arrowBackoffD = cal.arrowBackoffD;
  arrowBackoffL = cal.arrowBackoffL; arrowBackoffR = cal.arrowBackoffR;
  arrow_releaseU = cal.arrow_releaseU; arrow_releaseD = cal.arrow_releaseD;
  arrow_releaseL = cal.arrow_releaseL; arrow_releaseR = cal.arrow_releaseR;
  button_release = cal.button_release; prePress     = cal.prePress;

  // Load pin assignments; reject out-of-range bytes (old v1.2 EEPROM will
  // have 0xFF / garbage here — range check keeps defaults in that case).
  if (cal.pinBX >= 2 && cal.pinBX <= 13) pinBX = cal.pinBX;
  if (cal.pinAY >= 2 && cal.pinAY <= 13) pinAY = cal.pinAY;
  if (cal.pinSR >= 2 && cal.pinSR <= 13) pinSR = cal.pinSR;
  if (cal.pinLR >= 2 && cal.pinLR <= 13) pinLR = cal.pinLR;
  if (cal.pinUD >= 2 && cal.pinUD <= 13) pinUD = cal.pinUD;
}

void saveCalibration() {
  CalibData cal;
  cal.magic        = EEPROM_MAGIC;
  cal.leftMax      = leftMax;       cal.rightMax     = rightMax;
  cal.leftrightOff = leftrightOff;
  cal.upMax        = upMax;         cal.downMax      = downMax;
  cal.updownOff    = updownOff;
  cal.YMax         = YMax;          cal.AMax         = AMax;
  cal.AYOff        = AYOff;
  cal.BMax         = BMax;          cal.XMax         = XMax;
  cal.BXOff        = BXOff;
  cal.SRMax        = SRMax;         cal.SROff        = SROff;
  cal.WMax         = WMax;
  cal.arrowBackoffU = arrowBackoffU; cal.arrowBackoffD = arrowBackoffD;
  cal.arrowBackoffL = arrowBackoffL; cal.arrowBackoffR = arrowBackoffR;
  cal.arrow_releaseU = arrow_releaseU; cal.arrow_releaseD = arrow_releaseD;
  cal.arrow_releaseL = arrow_releaseL; cal.arrow_releaseR = arrow_releaseR;
  cal.button_release = button_release; cal.prePress   = prePress;
  cal.pinBX = pinBX; cal.pinAY = pinAY; cal.pinSR = pinSR;
  cal.pinLR = pinLR; cal.pinUD = pinUD;
  EEPROM.put(EEPROM_ADDR, cal);
}


void setup()
{
  Serial.begin(9600);
  loadCalibration();
  myservo_1.attach(pinLR);
  myservo_2.attach(pinUD);
  myservo_3.attach(pinAY);
  myservo_4.attach(pinBX);
  myservo_5.attach(pinSR);
  myservo_6.attach(9);   // NE joystick — pin fixed; not user-configurable

  myservo_1.write(leftrightOff);
  myservo_2.write(updownOff);
  myservo_3.write(AYOff);
  myservo_4.write(BXOff);
  myservo_5.write(SROff);
  stickX = leftrightOff;
  stickY = updownOff;
}


void loop()
{
  if (Serial.available() >= 1)
  {
    char type = Serial.read();

    // 'i' prefix: receive a 2-byte int bearing angle for NE_TAP / NE_Hold
    if (type == 'i') {
      while (Serial.available() < sizeof(int));
      int receivedInt;
      Serial.readBytes((char*)&receivedInt, sizeof(receivedInt));
      bearing = receivedInt;
      letter = 0;   // v1.6: only sets the bearing (v1.5 re-ran the previous command)
    }
    else {
      letter = int(type);
    }

    switch (letter) {

      // ── Read calibration ───────────────────────────────────────────────
      // Sends 30 comma-separated values (25 calibration + 5 pin assignments)
      // terminated by \n.  No ACK byte — the newline is the response terminator.
      case READ_CALIBRATION:
        Serial.print(leftMax);        Serial.print(',');
        Serial.print(rightMax);       Serial.print(',');
        Serial.print(leftrightOff);   Serial.print(',');
        Serial.print(upMax);          Serial.print(',');
        Serial.print(downMax);        Serial.print(',');
        Serial.print(updownOff);      Serial.print(',');
        Serial.print(YMax);           Serial.print(',');
        Serial.print(AMax);           Serial.print(',');
        Serial.print(AYOff);          Serial.print(',');
        Serial.print(BMax);           Serial.print(',');
        Serial.print(XMax);           Serial.print(',');
        Serial.print(BXOff);          Serial.print(',');
        Serial.print(SRMax);          Serial.print(',');
        Serial.print(SROff);          Serial.print(',');
        Serial.print(WMax);           Serial.print(',');
        Serial.print(arrowBackoffU);  Serial.print(',');
        Serial.print(arrowBackoffD);  Serial.print(',');
        Serial.print(arrowBackoffL);  Serial.print(',');
        Serial.print(arrowBackoffR);  Serial.print(',');
        Serial.print(arrow_releaseU); Serial.print(',');
        Serial.print(arrow_releaseD); Serial.print(',');
        Serial.print(arrow_releaseL); Serial.print(',');
        Serial.print(arrow_releaseR); Serial.print(',');
        Serial.print(button_release); Serial.print(',');
        Serial.print(prePress);       Serial.print(',');   // field 24
        Serial.print(pinBX);          Serial.print(',');   // field 25 — X/B  pin
        Serial.print(pinAY);          Serial.print(',');   // field 26 — A/Y  pin
        Serial.print(pinSR);          Serial.print(',');   // field 27 — SR   pin
        Serial.print(pinLR);          Serial.print(',');   // field 28 — LR   pin
        Serial.println(pinUD);                             // field 29 — UD   pin
        break;

      // ── Write calibration ──────────────────────────────────────────────
      // Accepts 25 values (legacy) or 30 values (v1.3 with pins).
      // A 30-value write also reattaches servos to the new pins immediately.
      case WRITE_CALIBRATION: {
        String line = Serial.readStringUntil('\n');
        line.trim();
        char buf[160];
        line.toCharArray(buf, sizeof(buf));
        int vals[30];
        int idx = 0;
        char* token = strtok(buf, ",");
        while (token && idx < 30) {
          vals[idx++] = atoi(token);
          token = strtok(NULL, ",");
        }
        if (idx >= 25) {
          leftMax = vals[0];  rightMax = vals[1];  leftrightOff = vals[2];
          upMax   = vals[3];  downMax  = vals[4];  updownOff    = vals[5];
          YMax    = vals[6];  AMax     = vals[7];  AYOff        = vals[8];
          BMax    = vals[9];  XMax     = vals[10]; BXOff        = vals[11];
          SRMax   = vals[12]; SROff    = vals[13]; WMax         = vals[14];
          arrowBackoffU = vals[15]; arrowBackoffD = vals[16];
          arrowBackoffL = vals[17]; arrowBackoffR = vals[18];
          arrow_releaseU = vals[19]; arrow_releaseD = vals[20];
          arrow_releaseL = vals[21]; arrow_releaseR = vals[22];
          button_release = vals[23]; prePress       = vals[24];

          if (idx == 30) {
            pinBX = constrain(vals[25], 2, 13);
            pinAY = constrain(vals[26], 2, 13);
            pinSR = constrain(vals[27], 2, 13);
            pinLR = constrain(vals[28], 2, 13);
            pinUD = constrain(vals[29], 2, 13);
            // Reattach servos to updated pins and move to resting positions
            myservo_1.detach(); myservo_1.attach(pinLR); myservo_1.write(leftrightOff);
            myservo_2.detach(); myservo_2.attach(pinUD); myservo_2.write(updownOff);
            stickX = leftrightOff; stickY = updownOff;
            myservo_3.detach(); myservo_3.attach(pinAY); myservo_3.write(AYOff);
            myservo_4.detach(); myservo_4.attach(pinBX); myservo_4.write(BXOff);
            myservo_5.detach(); myservo_5.attach(pinSR); myservo_5.write(SROff);
          }
          saveCalibration();
        }
        Serial.write(ACK);
        break;
      }

      // ── D-pad hold ─────────────────────────────────────────────────────
      case HOLD_UP:
        releaseButtons();
        stickGlide(leftrightOff, upMax);
        delay(prePress);
        myservo_2.write(upMax - arrowBackoffU);
        stickY = upMax - arrowBackoffU;
        Serial.write(ACK);
        break;
      case HOLD_DOWN:
        releaseButtons();
        stickGlide(leftrightOff, downMax);
        delay(prePress);
        myservo_2.write(downMax + arrowBackoffD);
        stickY = downMax + arrowBackoffD;
        Serial.write(ACK);
        break;
      case HOLD_LEFT:
        releaseButtons();
        stickGlide(leftMax, updownOff);
        delay(prePress);
        myservo_1.write(leftMax + arrowBackoffL);
        stickX = leftMax + arrowBackoffL;
        Serial.write(ACK);
        break;
      case HOLD_RIGHT:
        releaseButtons();
        stickGlide(rightMax, updownOff);
        delay(prePress);
        myservo_1.write(rightMax - arrowBackoffR);
        stickX = rightMax - arrowBackoffR;
        Serial.write(ACK);
        break;

      // ── D-pad press and release ────────────────────────────────────────
      case PRESS_UP:
        releaseButtons();
        stickCentre(true);
        myservo_2.write(upMax);
        delay(arrow_releaseU);
        myservo_2.write(updownOff);
        Serial.write(ACK);
        break;
      case PRESS_DOWN:
        releaseButtons();
        stickCentre(true);
        myservo_2.write(downMax);
        delay(arrow_releaseD);
        myservo_2.write(updownOff);
        Serial.write(ACK);
        break;
      case PRESS_LEFT:
        releaseButtons();
        stickCentre(true);
        myservo_1.write(leftMax);
        delay(arrow_releaseL);
        myservo_1.write(leftrightOff);
        Serial.write(ACK);
        break;
      case PRESS_RIGHT:
        releaseButtons();
        stickCentre(true);
        myservo_1.write(rightMax);
        delay(arrow_releaseR);
        myservo_1.write(leftrightOff);
        Serial.write(ACK);
        break;

      // ── NE joystick (diagonal) ─────────────────────────────────────────
      case NE_TAP:
        releaseButtons();
        stickCentre(true);
        stickMove(bearing, 100);
        delay(70);
        stickCentre(false);
        Serial.write(ACK);
        break;

      case NE_Hold:
        releaseButtons();
        stickMove(bearing, 100);
        Serial.write(ACK);
        break;

      // ── Joystick: any angle, any strength (v1.6) ─────────────────────────
      case STICK: {
        byte b[4];
        if (Serial.readBytes(b, 4) == 4) {
          int angle = b[0] | (b[1] << 8);
          int mag   = constrain(b[2], 0, 100);
          releaseButtons();
          if (mag == 0) {
            stickCentre(false);
          } else {
            stickMove(angle, mag);        // glides if a direction is already held
            if (b[3] > 0) {               // tap: hold for the duration, then centre
              delay(b[3] * 10);
              stickCentre(false);
            }
          }
        }
        Serial.write(ACK);
        break;
      }

      // ── Version (v1.6) ───────────────────────────────────────────────────
      case VERSION_QUERY:
        Serial.println(FW_VERSION);
        break;

      // ── Face buttons ───────────────────────────────────────────────────
      case BUTTON_A:
        releaseAll();
        myservo_3.write(AMax);
        delay(button_release);
        myservo_3.write(AYOff);
        Serial.write(ACK);
        break;
      case BUTTON_B:
        releaseAll();
        myservo_4.write(BMax);
        delay(button_release);
        myservo_4.write(BXOff);
        Serial.write(ACK);
        break;
      case BUTTON_X:
        releaseAll();
        myservo_4.write(XMax);
        delay(button_release);
        myservo_4.write(BXOff);
        Serial.write(ACK);
        break;
      case BUTTON_Y:
        releaseAll();
        myservo_3.write(YMax);
        delay(button_release);
        myservo_3.write(AYOff);
        Serial.write(ACK);
        break;

      // ── Soft Reset 1 / Home ────────────────────────────────────────────
      case SOFT_RESET_1:
        releaseAll();
        myservo_5.write(SRMax);
        delay(250);
        myservo_5.write(SROff);
        Serial.write(ACK);
        break;

      // ── Soft Reset 2 ───────────────────────────────────────────────────
      case SOFT_RESET_2:
        releaseAll();
        myservo_5.write(SRMax);
        myservo_3.write(AMax);
        myservo_4.write(BMax);
        delay(250);
        myservo_5.write(SROff);
        myservo_3.write(AYOff);
        myservo_4.write(BXOff);
        Serial.write(ACK);
        break;

      // ── Wonder Trade / + ───────────────────────────────────────────────
      case WONDER_TRADE:
        releaseAll();
        myservo_5.write(WMax);
        delay(250);
        myservo_5.write(SROff);
        Serial.write(ACK);
        break;

      // ── Release all ────────────────────────────────────────────────────
      case RELEASE_ALL:
        releaseAll();
        Serial.write(ACK);
        break;
    }
  }
}


void releaseButtons() {
  myservo_3.write(AYOff);
  myservo_4.write(BXOff);
  myservo_5.write(SROff);
}

void releaseAll() {
  releaseButtons();
  stickCentre(false);
}


// ── Joystick helpers (v1.6) ───────────────────────────────────────────────────

bool stickHeld() {
  return stickX != leftrightOff || stickY != updownOff;
}

// Centre the stick. settle = wait for it to get there before the next move.
void stickCentre(bool settle) {
  bool wasHeld = stickHeld();
  myservo_1.write(leftrightOff);
  myservo_2.write(updownOff);
  stickX = leftrightOff;
  stickY = updownOff;
  if (settle && wasHeld) delay(STICK_SETTLE_MS);
}

// One axis: v in -1..1, positive towards posMax; each side scaled to its own limit
int axisPos(float v, int posMax, int negMax, int off) {
  if (v >= 0) return off + (int)round(v * (posMax - off));
  return off + (int)round(-v * (negMax - off));
}

// Move from wherever the stick is to (tx, ty). From a held position both
// servos move together in small steps (a straight line stays inside the
// stick's circle), so one axis never pushes out while the other is still
// holding the stick - the v1.5 jam.
void stickGlide(int tx, int ty) {
  int dx = tx - stickX;
  int dy = ty - stickY;
  int steps = max(abs(dx), abs(dy)) / GLIDE_STEP_DEG;
  if (stickHeld()) {
    for (int i = 1; i < steps; i++) {
      myservo_1.write(stickX + (int)((long)dx * i / steps));
      myservo_2.write(stickY + (int)((long)dy * i / steps));
      delay(GLIDE_STEP_MS);
    }
  }
  myservo_1.write(tx);
  myservo_2.write(ty);
  stickX = tx;
  stickY = ty;
}

// angle: 0 = up, 90 = right (clockwise); mag: 0-100 %
void stickMove(int angle, int mag) {
  float t = angle * PI / 180.0;
  float m = mag / 100.0;
  stickGlide(axisPos(m * sin(t), rightMax, leftMax, leftrightOff),
             axisPos(m * cos(t), upMax, downMax, updownOff));
}
