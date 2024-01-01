#include "Arduino.h"
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include "esp_sleep.h"

#define DEBUG false
#define Serial if(DEBUG) Serial

const int NUM_MEASUREMENTS = 5;
const int DEEP_SLEEP_US = 600000000;  // 10 minutes

const int LED_PIN = 15;

const int ULTRASONIC_RX_PIN = 21;
const int ULTRASONIC_TX_PIN = 18;

const int LORA_CS_PIN = 12;    // (12 on s2) LoRa radio chip select
const int LORA_RESET_PIN = 3;  // (3 on s2)  LoRa radio reset
const int LORA_IRQ_PIN = 5;    // (5 on s2)  change for your board; must be a hardware interrupt pin

const int BATTERY_VOLTAGE_PIN = 16;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int loraFailures = 0;
RTC_DATA_ATTR int distFailures = 0;

//SPIClass& spi = new SPIClass();
SoftwareSerial jsnSerial(ULTRASONIC_RX_PIN, ULTRASONIC_TX_PIN);

void setup() {
  Serial.begin(9600);

  Serial.println("Starting up...");
  bootCount++;

  // ultrasonic sensor setup
  Serial.println("Init software serial...");
  jsnSerial.begin(9600);


  Serial.println("Starting LoRa serial...");

  // initialize lora
  LoRa.setPins(LORA_CS_PIN, LORA_RESET_PIN, LORA_IRQ_PIN);
  int loraRes = LoRa.begin(433525E3);
  if (!loraRes) {
    Serial.println("Failed to start lora.");
    loraFailures++;
    deepSleep();
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(6);
  LoRa.setOCP(240);
  LoRa.enableCrc();
}

void deepSleep() {
  Serial.println("deep sleep");
  LoRa.end();
  pinMode(LORA_RESET_PIN, INPUT);

  //gpio_hold_en(GPIO_NUM_10);
  //gpio_hold_en(GPIO_NUM_11);
  //gpio_hold_en(GPIO_NUM_13);
  //gpio_hold_en(GPIO_NUM_12);
  //gpio_hold_en(GPIO_NUM_4);
  //gpio_hold_en(GPIO_NUM_2);
  //gpio_hold_en(GPIO_NUM_18);
  //gpio_hold_en(GPIO_NUM_21);
  //gpio_hold_en((gpio_num_t) LORA_RESET_PIN);
  //gpio_hold_en((gpio_num_t) LORA_CS_PIN);
  
#if DEBUG
  delay(DEEP_SLEEP_US/1000);
  ESP.restart();
#else
  ESP.deepSleep(DEEP_SLEEP_US);
#endif
}

int cmpInt(const void* a, const void* b) {
  return (*(int*)a - *(int*)b);
}


void loop() {
  int distanceMM = measureDistance();
  Serial.print("Distance: ");
  Serial.println(distanceMM);
  int battery = measureBatteryVoltage();
  Serial.print("Voltage: ");
  Serial.println(distanceMM);

  if (LoRa.beginPacket()) {
    LoRa.write(0xd1);
    LoRa.write(0x57);
    LoRa.write(bootCount >> 8);
    LoRa.write(bootCount & 0xFF);
    LoRa.write(distanceMM >> 8);
    LoRa.write(distanceMM & 0xFF);
    LoRa.write(battery >> 8);
    LoRa.write(battery & 0xFF);
    LoRa.write(loraFailures);
    LoRa.write(distFailures);
    if (!LoRa.endPacket()) {
      loraFailures++;
    }

  } else {
    loraFailures++;
  }

  //ultrasonic sensor should be configured with low powered option
  deepSleep();
}

int measureDistance() {
  int measurements[NUM_MEASUREMENTS];

  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    int dist = pingDistance();
    if (dist < 0) {
      distFailures++;
    }
    measurements[i] = dist;
    delay(30);
  }

  qsort(measurements, NUM_MEASUREMENTS, sizeof(int), cmpInt);

  return measurements[NUM_MEASUREMENTS / 2];
}

int pingDistance() {

  // remove anything from the buffer that might interfere
  while (jsnSerial.available()) {
    jsnSerial.read();
  }

  jsnSerial.write(0x01);
  unsigned long start = millis();
  do {
    delay(1);
  } while (jsnSerial.available() < 4 && millis() - start < 100);

  if (jsnSerial.available() < 4) {
    return -1;
  }

  byte startByte = (byte)jsnSerial.read();
  if (startByte == 255) {
    byte buf[3];
    jsnSerial.readBytes(buf, 3);
    int h_data = buf[0];
    int l_data = buf[1];
    byte sum = buf[2];

    int distance = (h_data << 8) + l_data;

    if (((h_data + l_data) & 0xFF) - 1 != sum) {
      return -3;
    } else {
      return distance;
    }
  } else {
    return -2;
  }
}

int measureBatteryVoltage() {
  int measurements[NUM_MEASUREMENTS];

  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    measurements[i] = readBatteryVoltage();
  }

  qsort(measurements, NUM_MEASUREMENTS, sizeof(int), cmpInt);

  return measurements[NUM_MEASUREMENTS / 2];
}

int readBatteryVoltage() {
  return 2 * analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
}
