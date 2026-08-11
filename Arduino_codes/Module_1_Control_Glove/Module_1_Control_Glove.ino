/*
  MODULE-1: ROBOT CONTROLLING GLOVE (ESP32 DEV MODULE)
  ----------------------------------------------------
  OLED → SDA = D19, SCL = D18  (Wire1)
  MPU6050 → SDA = D22, SCL = D21 (Wire)
  Claw Button → D26
  Save Button → D27
  Run Button → D14
  Reset Button → D12
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

// ----------------------- USER CONFIG -----------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 19
#define OLED_SCL 18

#define MPU_SDA 22
#define MPU_SCL 21

#define CLAW_BTN 26
#define SAVE_BTN 27
#define RUN_BTN 14
#define RESET_BTN 12

// WiFi Credentials
const char* ssid = "GENOMXZ";
const char* password = "12345678";

// -----------------------------------------------------------

// Create objects
MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);

//Port 3333 to match Module-2
WiFiServer server(3333);
WiFiClient client;

// Control Variables
String baseState = "0";
String shoulderState = "0";
String elbowState = "0";
String clawState = "0";
String saveState = "0";
String runState = "0";
String resetState = "0";

// Track last displayed online/offline state to avoid flicker
bool lastOnlineState = false;

// -----------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Button Setup
  pinMode(CLAW_BTN, INPUT_PULLUP);
  pinMode(SAVE_BTN, INPUT_PULLUP);
  pinMode(RUN_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN, INPUT_PULLUP);

  // Initialize I2C for OLED and MPU6050
  Wire1.begin(OLED_SDA, OLED_SCL);   // For OLED
  Wire.begin(MPU_SDA, MPU_SCL);      // For MPU6050

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found!");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Checking MPU6050...");
  display.display();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("MPU6050 not found!");
    display.display();
    while (true);
  }

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("System Ready!");
  display.display();
  delay(2000);

  // Start WiFi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Connected to:");
  display.println(ssid);
  display.print("IP: ");
  display.println(IP);
  display.display();

  //Start server on port 3333
  server.begin();
  Serial.println("WiFi Ready, waiting for Module-2...");
}

// -----------------------------------------------------------

void loop() {
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
    }
  }

  // Current online state
  bool online = (client && client.connected());

  // Update the OLED only when the connection state changes
  if (online != lastOnlineState) {
    lastOnlineState = online;
    displaySystemStatus(online);
    delay(250);
  }

  // If not online, do nothing else this loop 
  if (!online) {
    delay(100);
    return;
  }

  // Connected: run usual operations and update Data view
  readButtons();
  processMPU();
  sendData();
  displayData();

  delay(100);
}

// -----------------------------------------------------------

void readButtons() {
  clawState = digitalRead(CLAW_BTN) == LOW ? "1" : "0";
  saveState = digitalRead(SAVE_BTN) == LOW ? "1" : "0";
  runState = digitalRead(RUN_BTN) == LOW ? "1" : "0";
  resetState = digitalRead(RESET_BTN) == LOW ? "1" : "0";
}

// -----------------------------------------------------------

void processMPU() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ay, &ax, &az);

  // Simplified motion mapping
  if (ax > 8000) baseState = "2";
  else if (ax < -8000) baseState = "1";
  else baseState = "0";

  if (ay > 8000) shoulderState = "1";
  else if (ay < -8000) shoulderState = "2";
  else shoulderState = "0";

  if (az > 8000) elbowState = "1";
  else if (az < -8000) elbowState = "2";
  else elbowState = "0";
}

// -----------------------------------------------------------

void sendData() {
  if (client && client.connected()) {
    String packet = baseState + "," + shoulderState + "," + elbowState + "," +
                    clawState + "," + saveState + "," + runState + "," + resetState + "\n";
    client.print(packet);
  }
}

// -----------------------------------------------------------

void displayData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("== CONTROL DATA ==");

  display.print("Base: ");
  display.println(getAxisSymbol(baseState));

  display.print("Shoulder: ");
  display.println(getAxisSymbol(shoulderState));

  display.print("Elbow: ");
  display.println(getAxisSymbol(elbowState));

  display.print("Claw: ");
  display.println(clawState == "1" ? "Closed" : "Open");

  if (saveState == "1") display.println("Save: Pressed");
  if (runState == "1") display.println("Run: Pressed");
  if (resetState == "1") display.println("Reset: Pressed");

  display.setCursor(0, 55);
  display.print("System: ONLINE");
  display.display();
}

// -----------------------------------------------------------

String getAxisSymbol(String val) {
  if (val == "1") return "++";
  else if (val == "2") return "--";
  else return "--";
}

// -----------------------------------------------------------

void displaySystemStatus(bool online) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("== SYSTEM STATUS ==");

  if (online) {
    display.println("Connected to Module-2");
    display.setCursor(0, 55);
    display.print("System: ONLINE");
  } else {
    display.println("Waiting for Module-2...");
    display.setCursor(0, 55);
    display.print("System: OFFLINE");
  }
  display.display();
}
