/*
   DeltaBotFirmware - serial motion controller for the 3d printed DeltaBot.

   Hardware: Arduino Nano + GRBL CNC shield (V3/V4), three 28BYJ-48 steppers
   rewired as bipolar, each turning an M3 x 20 hex bolt in a captive nut so
   that it acts as a screw jack under one corner of the flexure top plate.

   The firmware deliberately knows nothing about the geometry of the machine:
   it speaks in steps and leaves millimetres, tilts and Z to the Python host
   (see ../python/deltabot).

   Protocol: one ASCII command per line, terminated with '\n'.  Every command
   answers with one or more data lines followed by "ok" or "err <reason>".
   When a move finishes the firmware emits an unsolicited "done" line.
   If a limit switch trips during a move, every axis stops dead and the
   firmware emits an unsolicited "limit n" line instead.

     M a b c   move to absolute step positions
     R a b c   move by a relative number of steps
     J n d     jog axis n (0..2) by d steps
     P         report position          -> "pos a b c"
     ?         report full status       -> "status ..."
     V v       set max speed (steps/s)
     A a       set acceleration (steps/s^2)
     E 0|1     disable / enable the drivers
     Z         set the current position as zero
     Z a b c   force the current position to a b c (no motion)
     B lo hi   set the soft limits, in steps (lo == hi disables them); a
               move that brings an axis back towards them is always allowed
     S         stop, decelerating
     X         emergency stop, immediate
     I         raw pin levels           -> "pins <D2..D13> <A0..A5>", 0/1 each
     $         print this command summary

   Limit switches: one per screw, on D9, D10 and D11, using the switch's
   normally closed contact between the pin and GND.  The internal pull-up
   holds the pin HIGH, so it reads LOW normally and HIGH when the switch is
   pressed - or when a wire breaks, which then stops the machine too.  The
   firmware remembers which way the screw was travelling when its switch
   tripped and refuses further moves that way ("err limit n") until it is
   released; moves back off the switch are always allowed.  A switch that is
   already tripped at power up, or pressed by hand while the screw is idle,
   blocks nothing because the direction is unknown.
*/

#include <AccelStepper.h>

#define FIRMWARE_ID "DeltaBot 1.2"

// ---------------------------------------------------------------- pin map
// GRBL shield: EN is active low and shared by all three drivers.
// These step/dir pairs match the original ThreeMotorStepper demo sketch; if
// an axis refuses to move, swap the two numbers for it.
const uint8_t EN_PIN = 8;
const uint8_t STEP_PIN[3] = {7, 6, 5};
const uint8_t DIR_PIN[3]  = {4, 3, 2};

// Flip an entry to true if positive steps drive that corner the wrong way.
// Positive steps should drive every nut down towards its limit switch:
// clockwise seen from the back of the motor, anticlockwise from the shaft
// end.  Motor 1 is wired the other way round to motors 0 and 2.
const bool DIR_INVERT[3] = {true, false, true};

// Limit switches: normally closed contact from pin to GND, internal pull-up.
// Pressed (or a broken wire) reads HIGH.  For normally open switches to GND,
// change LIMIT_ACTIVE to LOW - but then a broken wire goes unnoticed.
const uint8_t LIMIT_PIN[3] = {9, 10, 11};
const uint8_t LIMIT_ACTIVE = HIGH;
const unsigned long LIMIT_RELEASE_MS = 20;   // clear this long to count as released

// Pins the firmware does not use.  They get the internal pull-up too, so the
// I command shows a switch that has been wired to the wrong pin.  D13 carries
// the Nano's LED, which fights the pull-up and may not read cleanly.
const uint8_t SPARE_PIN[] = {12, 13, A0, A1, A2, A3, A4, A5};

// ------------------------------------------------------------- parameters
const float DEFAULT_SPEED = 600.0;    // steps/s   (28BYJ-48 stalls if pushed)
const float DEFAULT_ACCEL = 2000.0;   // steps/s^2

AccelStepper stepper[3] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])
};

float maxSpeed = DEFAULT_SPEED;
float accel    = DEFAULT_ACCEL;
long  softMin  = -400000L;
long  softMax  =  400000L;
bool  enabled  = true;
bool  moving   = false;

bool  limitHit[3]  = {false, false, false};
int8_t limitDir[3] = {0, 0, 0};              // travel direction that tripped it, 0 = unknown
unsigned long limitSeen[3] = {0, 0, 0};      // millis() the switch last read tripped

char  line[64];
uint8_t lineLen = 0;

// --------------------------------------------------------------- helpers
void setEnabled(bool on) {
  enabled = on;
  digitalWrite(EN_PIN, on ? LOW : HIGH);   // active low
}

// A target outside the soft limits is still allowed if it is no further out
// than the axis is now, so an axis beyond them (after homing, say, or sitting
// on a limit switch) can always be brought back, and the other axes moved.
bool withinLimits(long target, long current) {
  if (softMin == softMax) return true;     // limits disabled
  if (target > softMax) return target <= current;
  if (target < softMin) return target >= current;
  return true;
}

// The first axis whose tripped limit switch this target would drive further
// into, or -1 if the move is clear.
int8_t blockedByLimit(long target[3]) {
  for (uint8_t i = 0; i < 3; i++) {
    if (!limitHit[i] || limitDir[i] == 0) continue;
    long d = target[i] - stepper[i].currentPosition();
    if ((d > 0 && limitDir[i] > 0) || (d < 0 && limitDir[i] < 0)) return i;
  }
  return -1;
}

void haltAll() {
  for (uint8_t i = 0; i < 3; i++) stepper[i].setCurrentPosition(stepper[i].currentPosition());
  moving = false;
}

// A switch counts as tripped from its first active reading, so a move is
// stopped at first contact, and as released only once it has read clear for
// LIMIT_RELEASE_MS, so contact bounce while backing off cannot re-trigger it.
void pollLimits() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    if (digitalRead(LIMIT_PIN[i]) == LIMIT_ACTIVE) {
      limitSeen[i] = now;
      if (!limitHit[i]) {
        limitHit[i] = true;
        long d = stepper[i].distanceToGo();
        limitDir[i] = (d > 0) ? 1 : (d < 0) ? -1 : 0;
        if (limitDir[i] != 0) {
          haltAll();
          Serial.print(F("limit "));
          Serial.println(i);
        }
      }
    } else if (limitHit[i] && now - limitSeen[i] >= LIMIT_RELEASE_MS) {
      limitHit[i] = false;
      limitDir[i] = 0;
    }
  }
}

bool rejectIfLimited(long target[3]) {
  int8_t axis = blockedByLimit(target);
  if (axis < 0) return false;
  Serial.print(F("err limit "));
  Serial.println(axis);
  return true;
}

// Scale each axis so that all three finish together: the axis with the
// longest travel runs at the full speed, the others in proportion.  This
// keeps the platform moving along a straight line in joint space while
// still getting AccelStepper's acceleration ramps.
void startMove(long target[3]) {
  long span = 0;
  for (uint8_t i = 0; i < 3; i++) {
    long d = target[i] - stepper[i].currentPosition();
    if (d < 0) d = -d;
    if (d > span) span = d;
  }
  for (uint8_t i = 0; i < 3; i++) {
    long d = target[i] - stepper[i].currentPosition();
    if (d < 0) d = -d;
    float f = (span > 0) ? (float)d / (float)span : 1.0;
    if (f < 0.001) f = 0.001;
    stepper[i].setMaxSpeed(maxSpeed * f);
    stepper[i].setAcceleration(accel * f);
    stepper[i].moveTo(target[i]);
  }
  moving = span > 0;
}

void reportPosition() {
  Serial.print(F("pos "));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(stepper[i].currentPosition());
    Serial.print(i < 2 ? ' ' : '\n');
  }
}

void reportStatus() {
  Serial.print(F("status "));
  Serial.print(moving ? F("moving") : F("idle"));
  Serial.print(F(" pos "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(stepper[i].currentPosition()); Serial.print(' '); }
  Serial.print(F("tgt "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(stepper[i].targetPosition()); Serial.print(' '); }
  Serial.print(F("en ")); Serial.print(enabled ? 1 : 0);
  Serial.print(F(" speed ")); Serial.print(maxSpeed);
  Serial.print(F(" accel ")); Serial.print(accel);
  Serial.print(F(" limits ")); Serial.print(softMin); Serial.print(' '); Serial.print(softMax);
  Serial.print(F(" sw "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(limitHit[i] ? 1 : 0); Serial.print(i < 2 ? ' ' : '\n'); }
}

// D2..D13 then A0..A5 (A6/A7 are analog only).  Outputs read back whatever
// they are driving.
void reportPins() {
  Serial.print(F("pins "));
  for (uint8_t p = 2; p <= 13; p++) Serial.print(digitalRead(p));
  Serial.print(' ');
  for (uint8_t p = A0; p <= A5; p++) Serial.print(digitalRead(p));
  Serial.println();
}

void printHelp() {
  Serial.println(F("M a b c | R a b c | J n d | P | ? | V v | A a | E 0|1"));
  Serial.println(F("Z [a b c] | B lo hi | S | X | I | $"));
}

// Pull up to `count` whitespace separated longs out of `s`.
// Returns the number actually parsed.
uint8_t parseLongs(char *s, long *out, uint8_t count) {
  uint8_t n = 0;
  char *tok = strtok(s, " ,\t");
  while (tok != NULL && n < count) {
    out[n++] = atol(tok);
    tok = strtok(NULL, " ,\t");
  }
  return n;
}

// ---------------------------------------------------------- command entry
void handleLine(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;

  char cmd = toupper(*s);
  char *args = s + 1;
  long v[3];

  switch (cmd) {
    case 'M':
    case 'R': {
      if (parseLongs(args, v, 3) != 3) { Serial.println(F("err need 3 values")); return; }
      long target[3];
      for (uint8_t i = 0; i < 3; i++)
        target[i] = (cmd == 'M') ? v[i] : stepper[i].currentPosition() + v[i];
      for (uint8_t i = 0; i < 3; i++)
        if (!withinLimits(target[i], stepper[i].currentPosition())) {
          Serial.println(F("err soft limit")); return;
        }
      if (rejectIfLimited(target)) return;
      if (!enabled) setEnabled(true);
      startMove(target);
      Serial.println(F("ok"));
      break;
    }
    case 'J': {
      if (parseLongs(args, v, 2) != 2) { Serial.println(F("err need axis and steps")); return; }
      if (v[0] < 0 || v[0] > 2) { Serial.println(F("err axis must be 0..2")); return; }
      long target[3];
      for (uint8_t i = 0; i < 3; i++) target[i] = stepper[i].targetPosition();
      target[v[0]] = stepper[v[0]].currentPosition() + v[1];
      if (!withinLimits(target[v[0]], stepper[v[0]].currentPosition())) {
        Serial.println(F("err soft limit")); return;
      }
      if (rejectIfLimited(target)) return;
      if (!enabled) setEnabled(true);
      startMove(target);
      Serial.println(F("ok"));
      break;
    }
    case 'P': reportPosition(); Serial.println(F("ok")); break;
    case '?': reportStatus();   Serial.println(F("ok")); break;
    case 'V': {
      float f = atof(args);
      if (f <= 0) { Serial.println(F("err speed must be > 0")); return; }
      maxSpeed = f;
      for (uint8_t i = 0; i < 3; i++) stepper[i].setMaxSpeed(maxSpeed);
      Serial.println(F("ok"));
      break;
    }
    case 'A': {
      float f = atof(args);
      if (f <= 0) { Serial.println(F("err accel must be > 0")); return; }
      accel = f;
      for (uint8_t i = 0; i < 3; i++) stepper[i].setAcceleration(accel);
      Serial.println(F("ok"));
      break;
    }
    case 'E': {
      if (parseLongs(args, v, 1) != 1) { Serial.println(F("err need 0 or 1")); return; }
      setEnabled(v[0] != 0);
      Serial.println(F("ok"));
      break;
    }
    case 'Z': {
      uint8_t n = parseLongs(args, v, 3);
      for (uint8_t i = 0; i < 3; i++) {
        long p = (n == 3) ? v[i] : 0L;
        stepper[i].setCurrentPosition(p);   // also clears speed and target
      }
      moving = false;
      Serial.println(F("ok"));
      break;
    }
    case 'B': {
      if (parseLongs(args, v, 2) != 2) { Serial.println(F("err need lo and hi")); return; }
      if (v[1] < v[0]) { Serial.println(F("err hi below lo")); return; }
      softMin = v[0];
      softMax = v[1];
      Serial.println(F("ok"));
      break;
    }
    case 'S':
      for (uint8_t i = 0; i < 3; i++) stepper[i].stop();   // decelerate to a halt
      Serial.println(F("ok"));
      break;
    case 'X':
      haltAll();
      Serial.println(F("ok"));
      break;
    case 'I': reportPins();     Serial.println(F("ok")); break;
    case '$': printHelp(); Serial.println(F("ok")); break;
    default:  Serial.println(F("err unknown command")); break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = '\0';
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;                       // overlong line, drop it
      Serial.println(F("err line too long"));
    }
  }
}

// ------------------------------------------------------------------ main
void setup() {
  pinMode(EN_PIN, OUTPUT);
  setEnabled(true);
  for (uint8_t i = 0; i < 3; i++) pinMode(LIMIT_PIN[i], INPUT_PULLUP);
  for (uint8_t i = 0; i < sizeof(SPARE_PIN); i++) pinMode(SPARE_PIN[i], INPUT_PULLUP);

  for (uint8_t i = 0; i < 3; i++) {
    stepper[i].setPinsInverted(DIR_INVERT[i], false, false);
    stepper[i].setMaxSpeed(maxSpeed);
    stepper[i].setAcceleration(accel);
    stepper[i].setCurrentPosition(0);
  }

  Serial.begin(115200);
  Serial.print(F(FIRMWARE_ID));
  Serial.println(F(" ready"));
}

void loop() {
  pollSerial();
  pollLimits();

  bool busy = false;
  for (uint8_t i = 0; i < 3; i++) {
    stepper[i].run();
    if (stepper[i].distanceToGo() != 0) busy = true;
  }

  if (moving && !busy) {
    moving = false;
    Serial.println(F("done"));
  }
}
