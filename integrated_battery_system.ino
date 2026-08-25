#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define CELL1 34
#define CELL2 35
#define CELL3 32
#define CELL4 33

#define RED_LED 2
#define GREEN_LED 4
#define YELLOW_LED 5
#define BUZZER 18
#define RELAY 19

const unsigned long SAMPLE_INTERVAL = 500;
const unsigned long SERIAL_INTERVAL = 1000;
const unsigned long LCD_INTERVAL = 200;
const unsigned long SCREEN_INTERVAL = 3000;
const unsigned long FAULT_CONFIRM_TIME = 2000;
const unsigned long RECOVERY_TIME = 5000;
const unsigned long WIFI_RETRY_INTERVAL = 10000;
const unsigned long BLYNK_RETRY_INTERVAL = 10000;
const unsigned long TELEMETRY_RETRY_INTERVAL = 3000;
const unsigned long FROZEN_ADC_TIME = 5000;

const float MIN_CELL_VOLTAGE = 3.00;
const float MAX_CELL_VOLTAGE = 4.20;
const float WEAK_CELL_LIMIT = 3.30;
const float MINOR_IMBALANCE_LIMIT = 2.0;
const float CRITICAL_IMBALANCE_LIMIT = 5.0;
const float RAPID_CHANGE_LIMIT = 0.30;

const int ADC_ZERO_LIMIT = 2;

unsigned long lastSampleTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastLCDTime = 0;
unsigned long lastScreenTime = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastBlynkAttempt = 0;
unsigned long lastTelemetryAttempt = 0;

unsigned long faultStartTime = 0;
unsigned long recoveryStartTime = 0;
unsigned long frozenStartTime = 0;

bool faultTimerRunning = false;
bool recoveryTimerRunning = false;
bool frozenTimerRunning = false;

bool telemetryPending = true;
bool previousWiFiState = false;
bool previousBlynkState = false;

int adcValue[4];
int previousADC[4];

float cellVoltage[4];
float previousCellVoltage[4];

float packVoltage = 0;
float averageVoltage = 0;
float minimumVoltage = 0;
float maximumVoltage = 0;
float imbalancePercent = 0;

int weakestCell = 0;
int strongestCell = 0;

unsigned long faultCounter = 0;
unsigned long lastFaultTime = 0;

enum BatteryHealth {
  HEALTHY,
  MINOR_IMBALANCE,
  CRITICAL_IMBALANCE,
  PACK_FAILURE
};

BatteryHealth batteryHealth = HEALTHY;

enum SystemState {
  NORMAL,
  DEGRADED,
  FAILSAFE,
  SHUTDOWN
};

SystemState systemState = NORMAL;
SystemState previousSystemState = NORMAL;

enum FaultType {
  NO_FAULT,
  WEAK_CELL,
  OVERVOLTAGE,
  SENSOR_FAULT,
  RAPID_FLUCTUATION,
  FROZEN_ADC,
  RELAY_MISMATCH
};

FaultType activeFault = NO_FAULT;
FaultType previousFault = NO_FAULT;

uint8_t currentScreen = 0;

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RELAY, HIGH);

  noTone(BUZZER);

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Battery System");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  readBattery();
  calculateBatteryAnalytics();
  evaluateBatteryHealth();

  for (int i = 0; i < 4; i++) {
    previousADC[i] = adcValue[i];
    previousCellVoltage[i] = cellVoltage[i];
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  Blynk.config(BLYNK_AUTH_TOKEN);

  lastSampleTime = millis();
  lastSerialTime = millis();
  lastLCDTime = millis();
  lastScreenTime = millis();

  Serial.println();
  Serial.println("================================");
  Serial.println("Integrated Battery System");
  Serial.println("System Starting");
  Serial.println("================================");
}

void loop() {
  unsigned long now = millis();

  maintainNetwork();

  Blynk.run();

  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;

    readBattery();
    calculateBatteryAnalytics();
    evaluateBatteryHealth();

    detectSafetyFaults();
    detectFrozenADC();

    updateRuntimeState();
    updateOutputs();
    checkRelayState();

    if (importantDataChanged()) {
      telemetryPending = true;
    }

    savePreviousValues();
  }

  if (now - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = now;
    printSerialData();
  }

  if (now - lastScreenTime >= SCREEN_INTERVAL) {
    lastScreenTime = now;

    if (activeFault == NO_FAULT) {
      currentScreen++;

      if (currentScreen >= 4) {
        currentScreen = 0;
      }
    }
  }

  if (now - lastLCDTime >= LCD_INTERVAL) {
    lastLCDTime = now;
    updateHMI();
  }

  if (telemetryPending &&
      WiFi.status() == WL_CONNECTED &&
      Blynk.connected() &&
      now - lastTelemetryAttempt >= TELEMETRY_RETRY_INTERVAL) {

    lastTelemetryAttempt = now;
    sendTelemetry();
  }
}

void maintainNetwork() {
  unsigned long now = millis();

  bool wifiNow = WiFi.status() == WL_CONNECTED;

  if (!wifiNow) {
    previousWiFiState = false;
    previousBlynkState = false;

    if (now - lastWiFiAttempt >= WIFI_RETRY_INTERVAL) {
      lastWiFiAttempt = now;

      Serial.println("WiFi reconnecting...");

      WiFi.disconnect();
      WiFi.begin(ssid, pass);
    }

    return;
  }

  if (!previousWiFiState) {
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    previousWiFiState = true;
    telemetryPending = true;
  }

  if (!Blynk.connected()) {
    if (now - lastBlynkAttempt >= BLYNK_RETRY_INTERVAL) {
      lastBlynkAttempt = now;

      Serial.println("Blynk connecting...");

      bool connected = Blynk.connect(1000);

      if (connected) {
        Serial.println("Blynk connected");
        telemetryPending = true;
      }
      else {
        Serial.println("Blynk connection failed");
      }
    }
  }
  else {
    if (!previousBlynkState) {
      Serial.println("Blynk device ONLINE");
      previousBlynkState = true;
      telemetryPending = true;
    }
  }
}

void readBattery() {
  adcValue[0] = analogRead(CELL1);
  adcValue[1] = analogRead(CELL2);
  adcValue[2] = analogRead(CELL3);
  adcValue[3] = analogRead(CELL4);

  for (int i = 0; i < 4; i++) {
    cellVoltage[i] = convertADCToCellVoltage(adcValue[i]);
  }
}

float convertADCToCellVoltage(int adc) {
  float adcVoltage = (adc / 4095.0) * 3.3;

  return 3.0 + (adcVoltage / 3.3) * 1.2;
}

void calculateBatteryAnalytics() {
  packVoltage = 0;

  minimumVoltage = cellVoltage[0];
  maximumVoltage = cellVoltage[0];

  weakestCell = 0;
  strongestCell = 0;

  for (int i = 0; i < 4; i++) {
    packVoltage += cellVoltage[i];

    if (cellVoltage[i] < minimumVoltage) {
      minimumVoltage = cellVoltage[i];
      weakestCell = i;
    }

    if (cellVoltage[i] > maximumVoltage) {
      maximumVoltage = cellVoltage[i];
      strongestCell = i;
    }
  }

  averageVoltage = packVoltage / 4.0;

  if (averageVoltage > 0) {
    imbalancePercent =
      ((maximumVoltage - minimumVoltage) /
       averageVoltage) * 100.0;
  }
  else {
    imbalancePercent = 0;
  }
}

void evaluateBatteryHealth() {
  bool invalidReading = false;

  for (int i = 0; i < 4; i++) {
    if (adcValue[i] <= ADC_ZERO_LIMIT) {
      invalidReading = true;
      break;
    }

    if (cellVoltage[i] > MAX_CELL_VOLTAGE + 0.01) {
      invalidReading = true;
      break;
    }
  }

  if (invalidReading) {
    batteryHealth = PACK_FAILURE;
  }
  else if (imbalancePercent >= CRITICAL_IMBALANCE_LIMIT) {
    batteryHealth = CRITICAL_IMBALANCE;
  }
  else if (imbalancePercent >= MINOR_IMBALANCE_LIMIT) {
    batteryHealth = MINOR_IMBALANCE;
  }
  else {
    batteryHealth = HEALTHY;
  }
}

void detectSafetyFaults() {
  FaultType detected = NO_FAULT;

  for (int i = 0; i < 4; i++) {
    if (adcValue[i] <= ADC_ZERO_LIMIT) {
      detected = SENSOR_FAULT;
      break;
    }
  }

  if (detected == NO_FAULT) {
    for (int i = 0; i < 4; i++) {
      if (cellVoltage[i] >= MAX_CELL_VOLTAGE) {
        detected = OVERVOLTAGE;
        break;
      }
    }
  }

  if (detected == NO_FAULT) {
    for (int i = 0; i < 4; i++) {
      if (cellVoltage[i] <= WEAK_CELL_LIMIT) {
        detected = WEAK_CELL;
        break;
      }
    }
  }

  if (detected == NO_FAULT) {
    for (int i = 0; i < 4; i++) {
      float change =
        abs(cellVoltage[i] - previousCellVoltage[i]);

      if (change >= RAPID_CHANGE_LIMIT) {
        detected = RAPID_FLUCTUATION;
        break;
      }
    }
  }

  processFault(detected);
}

void detectFrozenADC() {
  bool frozen = true;

  for (int i = 0; i < 4; i++) {
    if (adcValue[i] != previousADC[i]) {
      frozen = false;
      break;
    }
  }

  bool zeroADC = false;

  for (int i = 0; i < 4; i++) {
    if (adcValue[i] <= ADC_ZERO_LIMIT) {
      zeroADC = true;
      break;
    }
  }

  if (frozen && zeroADC) {
    if (!frozenTimerRunning) {
      frozenTimerRunning = true;
      frozenStartTime = millis();
    }

    if (millis() - frozenStartTime >= FROZEN_ADC_TIME) {
      processFault(FROZEN_ADC);
    }
  }
  else {
    frozenTimerRunning = false;
  }
}

void processFault(FaultType detected) {
  unsigned long now = millis();

  if (detected != NO_FAULT) {
    recoveryTimerRunning = false;

    if (!faultTimerRunning) {
      faultTimerRunning = true;
      faultStartTime = now;
    }

    if (now - faultStartTime >= FAULT_CONFIRM_TIME) {
      if (activeFault != detected) {
        previousFault = activeFault;
        activeFault = detected;

        faultCounter++;
        lastFaultTime = now;

        telemetryPending = true;
      }
    }
  }
  else {
    faultTimerRunning = false;

    if (activeFault != NO_FAULT &&
        activeFault != RELAY_MISMATCH) {

      if (!recoveryTimerRunning) {
        recoveryTimerRunning = true;
        recoveryStartTime = now;
      }

      if (now - recoveryStartTime >= RECOVERY_TIME) {
        previousFault = activeFault;
        activeFault = NO_FAULT;

        recoveryTimerRunning = false;

        telemetryPending = true;
      }
    }
  }
}

void updateRuntimeState() {
  previousSystemState = systemState;

  if (activeFault == SENSOR_FAULT ||
      activeFault == FROZEN_ADC ||
      activeFault == RELAY_MISMATCH) {

    systemState = SHUTDOWN;
  }
  else if (activeFault == WEAK_CELL ||
           activeFault == OVERVOLTAGE ||
           activeFault == RAPID_FLUCTUATION) {

    systemState = FAILSAFE;
  }
  else if (batteryHealth == MINOR_IMBALANCE ||
           batteryHealth == CRITICAL_IMBALANCE) {

    systemState = DEGRADED;
  }
  else if (batteryHealth == PACK_FAILURE) {

    systemState = SHUTDOWN;
  }
  else {

    systemState = NORMAL;
  }

  if (previousSystemState != systemState) {
    telemetryPending = true;
  }
}

void updateOutputs() {
  if (systemState == NORMAL) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);

    digitalWrite(RELAY, HIGH);

    noTone(BUZZER);
  }

  else if (systemState == DEGRADED) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);

    digitalWrite(RELAY, HIGH);

    noTone(BUZZER);
  }

  else if (systemState == FAILSAFE) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    digitalWrite(RELAY, LOW);

    tone(BUZZER, 2000);
  }

  else if (systemState == SHUTDOWN) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    digitalWrite(RELAY, LOW);

    tone(BUZZER, 2500);
  }
}

void checkRelayState() {
  bool expectedRelay =
    (systemState == NORMAL ||
     systemState == DEGRADED);

  bool actualRelay = digitalRead(RELAY);

  if (expectedRelay != actualRelay) {
    if (activeFault != RELAY_MISMATCH) {
      activeFault = RELAY_MISMATCH;

      faultCounter++;
      lastFaultTime = millis();

      telemetryPending = true;
    }
  }
}

bool importantDataChanged() {
  for (int i = 0; i < 4; i++) {
    if (abs(cellVoltage[i] -
            previousCellVoltage[i]) >= 0.03) {
      return true;
    }
  }

  return false;
}

void savePreviousValues() {
  for (int i = 0; i < 4; i++) {
    previousADC[i] = adcValue[i];
    previousCellVoltage[i] = cellVoltage[i];
  }
}

int getRiskLevel() {
  if (systemState == SHUTDOWN) {
    return 3;
  }

  if (systemState == FAILSAFE) {
    return 2;
  }

  if (systemState == DEGRADED) {
    return 1;
  }

  return 0;
}

const char* getHealthText() {
  if (batteryHealth == HEALTHY) {
    return "GOOD";
  }

  if (batteryHealth == MINOR_IMBALANCE) {
    return "MINOR";
  }

  if (batteryHealth == CRITICAL_IMBALANCE) {
    return "CRITICAL";
  }

  return "FAILURE";
}

const char* getStateText() {
  if (systemState == NORMAL) {
    return "NORMAL";
  }

  if (systemState == DEGRADED) {
    return "DEGRADED";
  }

  if (systemState == FAILSAFE) {
    return "FAILSAFE";
  }

  return "SHUTDOWN";
}

const char* getFaultText() {
  if (activeFault == NO_FAULT) {
    return "NO FAULT";
  }

  if (activeFault == WEAK_CELL) {
    return "WEAK CELL";
  }

  if (activeFault == OVERVOLTAGE) {
    return "OVERVOLTAGE";
  }

  if (activeFault == SENSOR_FAULT) {
    return "SENSOR FAULT";
  }

  if (activeFault == RAPID_FLUCTUATION) {
    return "RAPID CHANGE";
  }

  if (activeFault == FROZEN_ADC) {
    return "FROZEN ADC";
  }

  return "RELAY MISMATCH";
}

const char* getRecommendation() {
  if (systemState == NORMAL) {
    return "Battery operating normally";
  }

  if (systemState == DEGRADED) {
    return "Monitor cell imbalance";
  }

  if (systemState == FAILSAFE) {
    if (activeFault == WEAK_CELL) {
      return "Inspect weak cell immediately";
    }

    if (activeFault == OVERVOLTAGE) {
      return "Check cell overvoltage";
    }

    if (activeFault == RAPID_FLUCTUATION) {
      return "Inspect rapid voltage change";
    }

    return "Inspect battery condition";
  }

  if (activeFault == SENSOR_FAULT) {
    return "Check battery sensors";
  }

  if (activeFault == FROZEN_ADC) {
    return "Check ADC inputs";
  }

  if (activeFault == RELAY_MISMATCH) {
    return "Check relay circuit";
  }

  return "Battery isolated service required";
}

void sendTelemetry() {
  if (WiFi.status() != WL_CONNECTED ||
      !Blynk.connected()) {
    return;
  }

  Blynk.virtualWrite(V0, cellVoltage[0]);
  Blynk.virtualWrite(V1, cellVoltage[1]);
  Blynk.virtualWrite(V2, cellVoltage[2]);
  Blynk.virtualWrite(V3, cellVoltage[3]);

  Blynk.virtualWrite(V4, packVoltage);
  Blynk.virtualWrite(V5, averageVoltage);
  Blynk.virtualWrite(V6, imbalancePercent);

  Blynk.virtualWrite(V7, getHealthText());
  Blynk.virtualWrite(V8, getStateText());
  Blynk.virtualWrite(V9, getFaultText());

  Blynk.virtualWrite(V10, faultCounter);
  Blynk.virtualWrite(V11, digitalRead(RELAY));
  Blynk.virtualWrite(V12, getRiskLevel());
  Blynk.virtualWrite(V13, getRecommendation());

  telemetryPending = false;
}

void updateHMI() {
  if (activeFault != NO_FAULT) {
    showFaultScreen();
    return;
  }

  if (currentScreen == 0) {
    showOverviewScreen();
  }
  else if (currentScreen == 1) {
    showCellScreen();
  }
  else if (currentScreen == 2) {
    showAnalyticsScreen();
  }
  else {
    showRuntimeScreen();
  }
}

void clearLCD() {
  lcd.setCursor(0, 0);
  lcd.print("                ");

  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void showOverviewScreen() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("PACK:");
  lcd.print(packVoltage, 2);
  lcd.print("V");

  lcd.setCursor(0, 1);
  lcd.print("AVG:");
  lcd.print(averageVoltage, 2);
  lcd.print(" ");
  lcd.print(getHealthText());
}

void showCellScreen() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("C1:");
  lcd.print(cellVoltage[0], 2);

  lcd.setCursor(9, 0);
  lcd.print("C2:");
  lcd.print(cellVoltage[1], 2);

  lcd.setCursor(0, 1);
  lcd.print("C3:");
  lcd.print(cellVoltage[2], 2);

  lcd.setCursor(9, 1);
  lcd.print("C4:");
  lcd.print(cellVoltage[3], 2);
}

void showAnalyticsScreen() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("IMB:");
  lcd.print(imbalancePercent, 2);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("W:C");
  lcd.print(weakestCell + 1);

  lcd.print(" S:C");
  lcd.print(strongestCell + 1);
}

void showRuntimeScreen() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("STATE:");
  lcd.print(getStateText());

  lcd.setCursor(0, 1);
  lcd.print("FAULTS:");
  lcd.print(faultCounter);
}

void showFaultScreen() {
  clearLCD();

  lcd.setCursor(0, 0);
  lcd.print("FAULT #");
  lcd.print(faultCounter);

  lcd.setCursor(0, 1);
  lcd.print(getFaultText());
}

void printSerialData() {
  Serial.print("C1: ");
  Serial.print(cellVoltage[0], 2);

  Serial.print(" | C2: ");
  Serial.print(cellVoltage[1], 2);

  Serial.print(" | C3: ");
  Serial.print(cellVoltage[2], 2);

  Serial.print(" | C4: ");
  Serial.print(cellVoltage[3], 2);

  Serial.print(" | Pack: ");
  Serial.print(packVoltage, 2);
  Serial.print("V");

  Serial.print(" | Avg: ");
  Serial.print(averageVoltage, 2);
  Serial.print("V");

  Serial.print(" | Imbalance: ");
  Serial.print(imbalancePercent, 2);
  Serial.print("%");

  Serial.print(" | Health: ");
  Serial.print(getHealthText());

  Serial.print(" | State: ");
  Serial.print(getStateText());

  Serial.print(" | Fault: ");
  Serial.print(getFaultText());

  Serial.print(" | Risk: ");
  Serial.print(getRiskLevel());

  Serial.print(" | Relay: ");
  Serial.print(digitalRead(RELAY) ? "ON" : "OFF");

  Serial.print(" | Fault Count: ");
  Serial.print(faultCounter);

  Serial.print(" | WiFi: ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "ON" : "OFF");

  Serial.print(" | Blynk: ");
  Serial.print(Blynk.connected() ? "ONLINE" : "OFFLINE");

  Serial.print(" | Recommendation: ");
  Serial.println(getRecommendation());
}
