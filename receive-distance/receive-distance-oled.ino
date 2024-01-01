#include <Arduino.h>
#include <EEPROM.h>

#include <SPI.h>
#include <LoRa.h>

#include <Wire.h>
#include <U8g2lib.h>

#include "esp_wifi.h"
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>
#include "NTP.h"
#include <WiFiUdp.h>

#include "utilities.h"

#define EEPROM_SIZE 400
#define EEPROM_WIFI_SSID_ADDRESS 0
#define EEPROM_WIFI_SSID_LEN 32
#define EEPROM_WIFI_PASSWORD_ADDRESS 40
#define EEPROM_WIFI_PASSWORD_LEN 64
#define EEPROM_DEPTH_MIN 200
#define EEPROM_DEPTH_MAX 210
#define EEPROM_DEPTH_LEN 5

#define NUM_MEASUREMENTS    500
#define DISPLAY_UPDATE_TIME 60000

U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;
AsyncWebServer server(80);

WiFiUDP ntpUDP;
NTP ntp(ntpUDP);

String readStringFromEEPROM(int address, int maxLength);
void writeStringToEEPROM(int address, const char* str, size_t maxLength);

String wifiSsid;
String wifiPassword;
int depthMin;
int depthMax;

typedef struct {
  time_t time;
  int bootCount;
  float procent;
  int distance;
} Measurement;

typedef struct {
  Measurement data[NUM_MEASUREMENTS];
  int index;
  int size;
} Measurements;

Measurements measurements;

int lastBattery;
int lastLoraFailures;
int lastDistFailures;
int lastPacketRssi;
float lastPacketSnr;

unsigned long lastReceived = 0;
unsigned long lastDisplayUpdate = 0;

void setup()
{

  //-------------------------------------------------------
  // init structures
  measurements.index = 0;
  measurements.size = 0;
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    measurements.data[i] = {0, 0, 0.0, 0};
  }

  //-------------------------------------------------------
  // read settings

  EEPROM.begin(EEPROM_SIZE);
  wifiSsid     = readStringFromEEPROM(EEPROM_WIFI_SSID_ADDRESS, EEPROM_WIFI_SSID_LEN);
  wifiPassword = readStringFromEEPROM(EEPROM_WIFI_PASSWORD_ADDRESS, EEPROM_WIFI_PASSWORD_LEN);
  depthMin     = readStringFromEEPROM(EEPROM_DEPTH_MIN, EEPROM_DEPTH_LEN).toInt();
  depthMax     = readStringFromEEPROM(EEPROM_DEPTH_MAX, EEPROM_DEPTH_LEN).toInt();

  if (depthMin <= depthMax) {
    depthMin = 600;
    depthMax = 0;
  }

  //-------------------------------------------------------
  // init serial

  Serial.begin(74880);
  Serial.println("initBoard");

  //-------------------------------------------------------
  // init display

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LED_OFF);

  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) {
    Serial.println("Started OLED");
    u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    u8g2->begin();
    u8g2->clearBuffer();
    u8g2->setFlipMode(0);
    u8g2->setFontMode(1);  // Transparent
    u8g2->setDrawColor(1);
    u8g2->setFontDirection(0);
    u8g2->firstPage();
    do {
      u8g2->setFont(u8g2_font_inb16_mr);
      u8g2->drawStr(0, 25, "TANK LoRa");
    } while (u8g2->nextPage());
    u8g2->sendBuffer();
  }
  u8g2->setFont(u8g2_font_6x10_mf);

  //-------------------------------------------------------
  // print settings

  Serial.println("==========================");
  Serial.println("SSID:     " + wifiSsid);
  Serial.println("Password: " + wifiPassword);
  Serial.println("Depth:    " + String(depthMin) + " - " + String(depthMax));
  Serial.println("==========================");

  u8g2->drawStr(0, 40, "WIFI: ");
  u8g2->sendBuffer();

  //-------------------------------------------------------
  // Connect to Wi-Fi
  WiFi.useStaticBuffers(true);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin(wifiSsid, wifiPassword);
  
  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime >= 30000) {
      Serial.println("Connection timed out");
      break;
    }
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }


  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    u8g2->drawStr(32, 40, WiFi.localIP().toString().c_str());
    u8g2->sendBuffer();

    ntp.begin();

  } else {
    WiFi.disconnect();
    delay(1000);
    Serial.println("Failed to connect to WiFi");
    Serial.println("Starting AP...");
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.softAP("TankLora");
    IPAddress apIP(192, 168, 100, 1); // Set your desired SoftAP IP address

    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)); // Set subnet mask
    IPAddress ip = WiFi.softAPIP();
    Serial.print("SoftAP IP address: ");
    Serial.println(ip);

    u8g2->drawStr(32, 40, WiFi.softAPIP().toString().c_str());
    u8g2->sendBuffer();
  }

  // When the power is turned on, a delay is required.
  delay(1500);


  //------------------------------------------------------- 
  // LORA

  Serial.print("Starting LoRa... ");

  u8g2->drawStr(0, 50, "LoRa: ");

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
  if (!LoRa.begin(433525E3)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(6);
  LoRa.setOCP(240);
  LoRa.enableCrc();

  Serial.println("done.");


  u8g2->drawStr(32, 50, "done");
  u8g2->sendBuffer();

  //-------------------------------------------------------
  // web server endpoints 

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

    int index = 0;
    if (request->hasParam("index")) {
      index = request->getParam("index")->value().toInt();
    }
    
    DynamicJsonDocument jsonDoc(1024);
    if (measurements.size > 0) {
      Measurement* m = getOldMeasurement(index);
      jsonDoc["bootCount"] = m->bootCount;
      jsonDoc["distance"] = m->distance;
      jsonDoc["procent"] = (int) (m->procent * 100);
      jsonDoc["received"] = m->time;
    }
    
    jsonDoc["battery"] = lastBattery;
    jsonDoc["loraFailures"] = lastLoraFailures;
    jsonDoc["distFailures"] = lastDistFailures;
    jsonDoc["lastRssi"] = lastPacketRssi;
    jsonDoc["lastSnr"] = lastPacketSnr;

    // Serialize the JSON object to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    request->send(200, "application/json", jsonString);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    // Create a JSON object
    DynamicJsonDocument jsonDoc(1024); // Adjust the size as needed
    jsonDoc["mindepth"] = depthMin;
    jsonDoc["maxdepth"] = depthMax;
    jsonDoc["ssid"] = wifiSsid;
    jsonDoc["password"] = "not-shown";

    // Serialize the JSON object to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    request->send(200, "application/json", jsonString);
  });

  server.on("/config/store", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid")) {
      Serial.println("got ssid");
      writeStringToEEPROM(
        EEPROM_WIFI_SSID_ADDRESS,
        request->getParam("ssid")->value().c_str(),
        EEPROM_WIFI_SSID_LEN);
    }
    if (request->hasParam("password")) {
      Serial.println("got password");
      writeStringToEEPROM(
        EEPROM_WIFI_PASSWORD_ADDRESS,
        request->getParam("password")->value().c_str(),
        EEPROM_WIFI_PASSWORD_LEN);
    }
    if (request->hasParam("mindepth")) {
      writeStringToEEPROM(
        EEPROM_DEPTH_MIN,
        request->getParam("mindepth")->value().c_str(),
        EEPROM_DEPTH_LEN);
    }
    if (request->hasParam("maxdepth")) {
      writeStringToEEPROM(
        EEPROM_DEPTH_MAX,
        request->getParam("maxdepth")->value().c_str(),
        EEPROM_DEPTH_LEN);
    }

    request->send(200, "text/plain", "Saved. Reboot now.");
  });

  server.begin();

  u8g2->drawStr(0, 60, "Waiting for data..");
  u8g2->sendBuffer();
}

void loop()
{
  ntp.update();

  // try to parse packet
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    Serial.print("Received packet ------ ");
    Serial.println(packetSize);
  }

  bool receivedPacket = false;

  if (packetSize == 10) {
    // received a packet

    // read packet
    int header = LoRa.read() << 8 | LoRa.read();
    
    if (header == 0xd157) {
      Serial.println("Header ok");
      receivedPacket = true;

      Measurement measurement;
      measurement.time = ntp.epoch();
      measurement.bootCount = LoRa.read() << 8 | LoRa.read();
      measurement.distance = LoRa.read() << 8 | LoRa.read();
      measurement.procent = 1 - min(1.0f, ((float) max(measurement.distance - depthMax, 0)) / (depthMin - depthMax));
      addMeasurement(measurement);

      lastBattery = LoRa.read() << 8 | LoRa.read();
      lastLoraFailures = LoRa.read();
      lastDistFailures = LoRa.read();
 
      lastPacketRssi = LoRa.packetRssi();
      lastPacketSnr = LoRa.packetSnr();

      //    min        max
      //|    v          v
      //|~~~~~~~~~~~~~           < sensor
      //                |<100cm>|
      //              |<-120cm->|
      //     |<-----200cm------>|
    }
  }

  if (measurements.size > 0 && (millis() - lastDisplayUpdate > DISPLAY_UPDATE_TIME || receivedPacket)) {  
    Serial.println("Redrawing");

    char buf[256];
    u8g2->clearBuffer();

    int diagramX = u8g2->getWidth() - 3;
    int diagramY = u8g2->getHeight() - 3;
    int diagramHeight = u8g2->getHeight() - 19 - 5;

    for (int i = 0; i < measurements.size && diagramX - i >= 3; i++) {
      Measurement* m = getOldMeasurement(i);
      int height = (int) (m->procent * diagramHeight);
      int x = diagramX - i;     
      u8g2->drawLine(x, diagramY, x, diagramY - height);
    }

    Measurement current = measurements.data[measurements.index];

    u8g2->drawFrame(0, 19, u8g2->getWidth(), u8g2->getHeight() - 19);
    
    u8g2->setFont(u8g2_font_5x8_mf);
    snprintf(buf, sizeof(buf), "DIST: %i mm (%i%%)", current.distance, (int) round(current.procent * 100));
    u8g2->drawStr(0, 8, buf);
    snprintf(buf, sizeof(buf), "STATUS: %.2fV | %im ago", (float) (lastBattery) / 1000, (ntp.epoch() - current.time) / 60);
    u8g2->drawStr(0, 17, buf);

    u8g2->sendBuffer();

    lastDisplayUpdate = millis();
  }
}

//==============================================================
// EEPROM Handling

String readStringFromEEPROM(int address, int maxLength) {
  Serial.println("read...");
  Serial.println(address);
  String result = "";
  for (int i = 0; i < maxLength; i++) {
    char c = EEPROM.read(address + i);
    if (c == '\0') {
      break; // Null terminator found, end of string
    }
    result += c;
  }
  return result;
}

void writeStringToEEPROM(int address, const char* str, size_t maxLength) {
  Serial.println("Writing...");
  Serial.println(address);
  Serial.println(str);
  int length = min(strlen(str), maxLength);
  for (int i = 0; i < length; i++) {
    EEPROM.write(address + i, str[i]);
  }
  EEPROM.write(address + length, '\0'); // Null-terminate the string
  EEPROM.commit(); // Save changes to EEPROM
}


//==============================================================
// Measurement handling

Measurement* getOldMeasurement(int negativeIndex) {
  if (negativeIndex < measurements.size) {
    int i = measurements.index - negativeIndex;
    if (i < 0) {
      i += NUM_MEASUREMENTS;
    }
    return &measurements.data[i];
  }
}

void addMeasurement(Measurement m) {
  int newIndex = measurements.index + 1;
  if (newIndex >= NUM_MEASUREMENTS) {
    newIndex = 0;
  }
  measurements.index = newIndex;
  measurements.data[newIndex] = m;
  if (measurements.size < NUM_MEASUREMENTS) {
    measurements.size++;
  }
}
