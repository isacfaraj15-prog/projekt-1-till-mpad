/*
* Namn: sopptuna projekt
* Författare: Isac
* Datum: 2025-10-8
* Beskrivning:
*  - Räknar IR-”klick” inom ett tidsfönster och styr motorreläer efter antal klick.
*  - Två ultraljudssensorer triggar varningar på OLED, buzzer och servo när motorerna står still.
*  - LED-lampor visar status (grön = normal, röd = varning).
*  - Servo som öppnar sopptuna locket när ultraljudssensorerna känner av något
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(128, 64, &Wire, -1);
Servo myservo;

// Pinnar
const int PIN_BUZZER      = A2;
const int PIN_RELAY_LEFT  = A1;
const int PIN_RELAY_RIGHT = 8;

const int PIN_LED_GREEN   = 7;
const int PIN_LED_RED     = 6;

const int PIN_TRIG1       = 4;
const int PIN_ECHO1       = 5;

const int PIN_TRIG2       = 2;
const int PIN_ECHO2       = 3;

const int PIN_SERVO       = 11;
const int PIN_IR          = 12;

// Polariteter
const bool LEFT_ACTIVE_LOW  = false;
const bool RIGHT_ACTIVE_LOW = false;
const bool IR_ACTIVE_LOW    = true;

// Avståndsgräns 
const int ALERT_DIST_CM = 5;

// Tider för IR-logik
const unsigned long WINDOW_MS     = 3000;   // hur länge man samlar klick
const unsigned long GAP_APPLY_MS  = 800;    // tid utan klick -> utför
const unsigned long PRESS_LOCK_MS = 250;    // låsning efter klick (anti-dubbelklick)

// Servolägen
const int SERVO_CLOSED = 180;
const int SERVO_OPEN   = 90;

// variabler för tillståndet
bool motorsMoving = false;                  // sant när någon motor är igång
bool alertActive  = false;                  // sant när röd varning är aktiv

// IR-klickräkning
int pressCount = 0;
bool counting = false;
bool lastIR = false;
unsigned long windowStart = 0;
unsigned long lastClickTime = 0;
unsigned long lockUntil = 0;


// Styr ett relä som kan vara active-low eller vanlig active-high

void relayWrite(int pin, bool on, bool activeLow) {
  if (activeLow) {
    if (on) digitalWrite(pin, LOW);
    else    digitalWrite(pin, HIGH);
  } else {
    if (on) digitalWrite(pin, HIGH);
    else    digitalWrite(pin, LOW);
  }
}



// Styr båda motorerna och markerar om motorsystemet är aktivt.

void driveMotors(bool leftOn, bool rightOn) {
  relayWrite(PIN_RELAY_LEFT,  leftOn,  LEFT_ACTIVE_LOW);
  relayWrite(PIN_RELAY_RIGHT, rightOn, RIGHT_ACTIVE_LOW);
  motorsMoving = (leftOn || rightOn);
}


// Använder en tabell för att styra motorerna efter antal IR-klick.

// 1 = båda motorer
// 2 = höger motor
// 3 = vänster motor
// annars = stopp

void applyMotorCommand(int cmd) {
  const bool L[4] = { false, true,  false, true  };
  const bool R[4] = { false, true,  true,  false };

  if (cmd < 0 || cmd > 3) cmd = 0;

  Serial.print("Kommando: ");
  Serial.println(cmd);

  driveMotors(L[cmd], R[cmd]);
}


//Nödstopp: stoppar motorer och nollställer IR-räkning.

void forceStopNow() {
  driveMotors(false, false);
  counting = false;
  pressCount = 0;
  lockUntil = millis() + 1000;
}

// läser av ir sensor och 
bool irSeenNow() {
  int v = digitalRead(PIN_IR);
  return 
    if (IR_ACTIVE_LOW) {
      active = (v == LOW);   // sensorn är aktiv vid LOW
  } else {
      active = (v == HIGH);  // sensorn är aktiv vid HIGH
  }
}



//IR-hantering:
 // den Räknar klick under ett anbtal sekunder och sedan utför handlingen beroende på antal klick

void handleIR() {
  unsigned long now = millis();

  if (now < lockUntil) {
    lastIR = irSeenNow();
    return;
  }


  bool cur = irSeenNow();

  // Nytt klick (flankdetektering)
  if (cur && !lastIR) {
    if (!counting) {
      counting = true;
      windowStart = now;
      pressCount = 0;
    }
    pressCount++;
    lastClickTime = now;
    lockUntil = now + PRESS_LOCK_MS;

    // 4 klick = nödstopp
    if (pressCount >= 4) {
      forceStopNow();
      lastIR = cur;
      return;
    }
  }

  lastIR = cur;

  // Tillämpa om tid gått ut
  if (counting) {
    if ((now - lastClickTime >= GAP_APPLY_MS) || (now - windowStart >= WINDOW_MS)) {
      counting = false;
      applyMotorCommand(pressCount);
      pressCount = 0;
      lockUntil = now + 500;
    }
  }
}


// Läser av ett avstånd i cm från en HC-SR04-sensor.

int readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long dur = pulseIn(echoPin, HIGH, 30000UL);
  if (dur == 0) return 999;

  return (int)(dur * 0.017);
}


//Visar antingen text eller smiley på OLED.

void showOLED(const char* text, bool smiley) {
  display.clearDisplay();
  display.setTextColor(WHITE);

  if (smiley) {
    display.setTextSize(3);
    display.setCursor(40, 20);
    display.print(":)");
  } else {
    display.setTextSize(1);
    display.setCursor(0, 20);
    if (text) display.print(text);
  }
  display.display();
}


 // Aktiverar och stänger av varningsläget.
 // on: true = aktivera varning
 //toneHz: buzzer-ton (0 = tyst)
 //msg: text till OLED
 //smiley: om smiley ska ritas istället

void setAlert(bool on, int toneHz = 0, const char* msg = nullptr, bool smiley = false) {
  alertActive = on;

  if (on) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    myservo.write(SERVO_OPEN);

    if (toneHz > 0) tone(PIN_BUZZER, toneHz);
    if (smiley) showOLED(nullptr, true);
    else if (msg) showOLED(msg, false);
  } else {
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    myservo.write(SERVO_CLOSED);
    noTone(PIN_BUZZER);

    display.clearDisplay();
    display.display();
  }
}



// Setup
void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_TRIG1, OUTPUT); pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_TRIG2, OUTPUT); pinMode(PIN_ECHO2, INPUT);

  pinMode(PIN_RELAY_LEFT, OUTPUT);
  pinMode(PIN_RELAY_RIGHT, OUTPUT);

  pinMode(PIN_IR, INPUT_PULLUP);

  driveMotors(false, false);
  myservo.attach(PIN_SERVO);
  myservo.write(SERVO_CLOSED);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  setAlert(false);
}


// Loop

void loop() {
  // IR-klickstyrning av motorer
  handleIR();

  // Avståndskontroll körs bara när motorerna står still
  if (!motorsMoving) {
    int d1 = readDistanceCm(PIN_TRIG1, PIN_ECHO1);
    int d2 = readDistanceCm(PIN_TRIG2, PIN_ECHO2);

    // Sensor 1: happy smiley + ljud + servo
    if (d1 > 0 && d1 < ALERT_DIST_CM) {
      setAlert(true, 400, nullptr, true);
      delay(3000);
      setAlert(false);

      counting = false;
      pressCount = 0;
    }
    // Sensor 2: soptunnan full
    else if (d2 > 0 && d2 < ALERT_DIST_CM) {
      setAlert(true, 900, "soptunnan ar full", false);
      delay(3000);
      // återställs automatiskt
      counting = false;
      pressCount = 0;
    }
    // Ingen varning till normalt läge
    else {
      if (alertActive) setAlert(false);
    }
  }

  delay(10);
}
