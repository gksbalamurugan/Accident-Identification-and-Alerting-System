
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================================================
// PIN CONFIGURATION
// ==================================================

// ADXL335 accelerometer
#define ACC_X 34
#define ACC_Y 35
#define ACC_Z 32

// NEO-6M GPS
#define GPS_RX 16
#define GPS_TX 17

// SIM900A GSM
#define GSM_RX 26
#define GSM_TX 27

// I2C OLED
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C

// Alert LED
#define ALERT_PIN 2

// Two relay channels
#define RELAY1_PIN 13   // IN1
#define RELAY2_PIN 25  // IN2

// ==================================================
// USER SETTINGS
// ==================================================

const char PHONE_NUMBER[] = "+916369324376";

// Fixed coordinates for SMS
const double FIXED_LATITUDE = 10.727282;
const double FIXED_LONGITUDE = 79.019074;

// OLED assumed SSD1306 128x64
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Most common relay modules are active LOW.
// true: LOW = relay ON, HIGH = relay OFF
// false: HIGH = relay ON, LOW = relay OFF
const bool RELAY_ACTIVE_LOW = true;

// Accident detection settings
const int ACCEL_THRESHOLD = 600;
const byte REQUIRED_HITS = 3;

const unsigned long SAMPLE_INTERVAL = 20;
const unsigned long GPS_MAX_AGE = 5000;

// ==================================================
// OBJECTS
// ==================================================

Adafruit_SSD1306 display(
  SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1
);

TinyGPSPlus gps;

HardwareSerial GPSModule(2);
HardwareSerial SIM900A(1);

// ==================================================
// GLOBAL VARIABLES
// ==================================================

float baseX = 0;
float baseY = 0;
float baseZ = 0;

bool calibrated = false;
bool oledAvailable = false;
bool gpsAvailable = false;

// Once an accident is detected, relay shutdown is latched.
// It will remain OFF until the ESP32 is restarted.
bool accidentLatched = false;

byte accidentHits = 0;

unsigned long lastSample = 0;

double latitude = 0;
double longitude = 0;

// ==================================================
// RELAY CONTROL
// ==================================================

void setRelayState(int pin, bool turnOn) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, turnOn ? LOW : HIGH);
  } else {
    digitalWrite(pin, turnOn ? HIGH : LOW);
  }
}

void turnCircuitOn() {
  setRelayState(RELAY1_PIN, true);
  setRelayState(RELAY2_PIN, true);

  Serial.println("Relays ON: Circuit enabled");
}

void turnCircuitOff() {
  setRelayState(RELAY1_PIN, false);
  setRelayState(RELAY2_PIN, false);

  Serial.println("Relays OFF: Circuit disabled");
}

// ==================================================
// OLED STATUS
// ==================================================

void showStatus(String line1, String line2 = "") {
  Serial.println(line1 + " " + line2);

  if (!oledAvailable) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(line1);

  display.setCursor(0, 18);
  display.println(line2);

  display.display();
}

// ==================================================
// GPS UPDATE
// ==================================================

void updateGPS() {
  while (GPSModule.available()) {
    gps.encode(GPSModule.read());
  }

  gpsAvailable =
    gps.location.isValid() &&
    gps.location.age() < GPS_MAX_AGE;

  if (gpsAvailable) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

// ==================================================
// ACCELEROMETER CALIBRATION
// Keep sensor stationary during startup.
// ==================================================

void calibrateAccelerometer() {
  const int samples = 200;

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  showStatus("Calibrating...", "Keep sensor still");

  for (int i = 0; i < samples; i++) {
    sumX += analogRead(ACC_X);
    sumY += analogRead(ACC_Y);
    sumZ += analogRead(ACC_Z);

    updateGPS();
    delay(10);
  }

  baseX = (float)sumX / samples;
  baseY = (float)sumY / samples;
  baseZ = (float)sumZ / samples;

  calibrated = true;

  Serial.printf("Baseline X: %.1f\n", baseX);
  Serial.printf("Baseline Y: %.1f\n", baseY);
  Serial.printf("Baseline Z: %.1f\n", baseZ);

  showStatus("System Ready", "Circuit ON");
  delay(1000);
}

// ==================================================
// GSM RESPONSE HANDLER
// ==================================================

bool waitForGSM(const char *expected,
                unsigned long timeout) {
  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {
    updateGPS();

    while (SIM900A.available()) {
      char c = SIM900A.read();
      response += c;

      if (response.indexOf(expected) >= 0) {
        Serial.print("GSM response: ");
        Serial.println(response);
        return true;
      }

      if (response.length() > 250) {
        response.remove(0, 125);
      }
    }

    delay(2);
  }

  Serial.print("GSM timeout: ");
  Serial.println(response);

  return false;
}

// ==================================================
// SEND SMS
// ==================================================

bool sendSMS(String message) {
  Serial.println("Preparing SMS...");

  // Check modem communication
  SIM900A.println("AT");

  if (!waitForGSM("OK", 3000)) {
    Serial.println("GSM not responding");
    return false;
  }

  // Enable text mode
  SIM900A.println("AT+CMGF=1");

  if (!waitForGSM("OK", 3000)) {
    Serial.println("SMS text mode failed");
    return false;
  }

  // Set recipient
  SIM900A.print("AT+CMGS=\"");
  SIM900A.print(PHONE_NUMBER);
  SIM900A.println("\"");

  if (!waitForGSM(">", 5000)) {
    Serial.println("SMS prompt not received");
    return false;
  }

  // Send message and Ctrl+Z
  SIM900A.print(message);
  SIM900A.write(26);

  bool success = waitForGSM("+CMGS:", 20000);

  if (success) {
    waitForGSM("OK", 3000);
    Serial.println("SMS submission acknowledged");
  } else {
    Serial.println("SMS submission not confirmed");
  }

  return success;
}

// ==================================================
// ACCIDENT ALERT
// Relays are switched OFF and remain OFF.
// ==================================================

void reportAccident() {
  if (accidentLatched) {
    return;
  }

  // Latch event before performing other operations
  accidentLatched = true;
  accidentHits = 0;

  // Immediately disconnect the controlled circuit
  turnCircuitOff();

  digitalWrite(ALERT_PIN, HIGH);

  showStatus("ACCIDENT DETECTED", "Circuit OFF");

  Serial.println("==============================");
  Serial.println("ACCIDENT DETECTED");
  Serial.println("Relay 1: OFF");
  Serial.println("Relay 2: OFF");
  Serial.println("==============================");

  // Use the specified fixed coordinates in every SMS
  String message = "ACCIDENT ALERT! Circuit switched OFF. ";
  message += "Location: https://maps.google.com/?q=";
  message += String(FIXED_LATITUDE, 6);
  message += ",";
  message += String(FIXED_LONGITUDE, 6);

  Serial.println("SMS message:");
  Serial.println(message);

  showStatus("Sending SMS...", "Circuit OFF");

  bool sent = sendSMS(message);

  digitalWrite(ALERT_PIN, LOW);

  if (sent) {
    showStatus("SMS Submitted", "Circuit remains OFF");
  } else {
    showStatus("SMS FAILED", "Circuit remains OFF");
  }

  Serial.println("Accident handler finished.");
  Serial.println("Relay shutdown remains latched.");
}

// ==================================================
// ACCIDENT DETECTION
// ==================================================

void checkAccident() {
  if (!calibrated || accidentLatched) {
    return;
  }

  if (millis() - lastSample < SAMPLE_INTERVAL) {
    return;
  }

  lastSample = millis();

  int x = analogRead(ACC_X);
  int y = analogRead(ACC_Y);
  int z = analogRead(ACC_Z);

  int dx = abs(x - (int)baseX);
  int dy = abs(y - (int)baseY);
  int dz = abs(z - (int)baseZ);

  bool abnormal =
    dx > ACCEL_THRESHOLD ||
    dy > ACCEL_THRESHOLD ||
    dz > ACCEL_THRESHOLD;

  Serial.printf(
    "X:%d Y:%d Z:%d | dX:%d dY:%d dZ:%d | Hits:%d\n",
    x, y, z, dx, dy, dz, accidentHits
  );

  if (abnormal) {
    if (accidentHits < REQUIRED_HITS) {
      accidentHits++;
    }
  } else {
    accidentHits = 0;
  }

  if (accidentHits >= REQUIRED_HITS) {
    reportAccident();
  }
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);

  // Configure relay outputs and enable circuit by default
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  turnCircuitOn();

  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);

  // ESP32 ADC configuration
  analogReadResolution(12);

  analogSetPinAttenuation(ACC_X, ADC_11db);
  analogSetPinAttenuation(ACC_Y, ADC_11db);
  analogSetPinAttenuation(ACC_Z, ADC_11db);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);

  oledAvailable = display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDR
  );

  if (!oledAvailable) {
    Serial.println("OLED not detected");
  }

  // Start GPS UART
  GPSModule.begin(
    9600, SERIAL_8N1, GPS_RX, GPS_TX
  );

  // Start GSM UART
  SIM900A.begin(
    9600, SERIAL_8N1, GSM_RX, GSM_TX
  );

  showStatus("Accident System", "Starting...");
  delay(1000);

  calibrateAccelerometer();

  Serial.println("System ready");
  Serial.println("Circuit state: ON");
  Serial.println("Waiting for accident...");
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  updateGPS();
  checkAccident();
}
