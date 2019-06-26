#include <Arduino.h>

#include <EEPROM.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EmonLib.h>             // Include Emon Library

#undef  MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 512

#define EEPROM_START 0

EnergyMonitor emon1;                   // Create an instance

boolean setEEPROM = false;
uint32_t memcrc; uint8_t *p_memcrc = (uint8_t*)&memcrc;

struct eeprom_data_t {
  char mqtt_server[40];
  char mqtt_port[6];
  char calibration[4];
} eeprom_data;

static  uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long crc_update(unsigned long crc, byte data) {
  byte tbl_idx;
  tbl_idx = crc ^ (data >> (0 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  tbl_idx = crc ^ (data >> (1 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  return crc;
}

unsigned long crc_byte(byte *b, int len) {
  unsigned long crc = ~0L;
  uint16_t i;

  for (i = 0 ; i < len ; i++)
  {
    crc = crc_update(crc, *b++);
  }
  crc = ~crc;
  return crc;
}

ESP8266WebServer server(80);
WiFiManager wifiManager;

WiFiClient espClient;
PubSubClient client(espClient);

char default_mqtt_server[40] = "";
char default_mqtt_port[6] = "1883";
char default_calibration[4] = "300";

String device_name = "power_meter_" + String(ESP.getChipId());
String availability_topic = "home/sensor/" + device_name + "/availability";
String power_state_topic = "home/sensor/" + device_name + "/state";
String power_config_topic = "home/sensor/" + device_name + "/config";

float calibration;
IPAddress MQTTserver;

void readSettingsESP() {
int i;
  uint32_t datacrc;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  for (i = EEPROM_START; i < EEPROM_START + sizeof(eeprom_data); i++) {
    eeprom_data_tmp[i] = EEPROM.read(i);
  }

  p_memcrc[0] = EEPROM.read(i++);
  p_memcrc[1] = EEPROM.read(i++);
  p_memcrc[2] = EEPROM.read(i++);
  p_memcrc[3] = EEPROM.read(i++);

  datacrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  if (memcrc == datacrc) {
    setEEPROM = true;
    memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));
  } else {
    strncpy(eeprom_data.mqtt_server, default_mqtt_server, sizeof(default_mqtt_server));
    strncpy(eeprom_data.mqtt_port, default_mqtt_port, sizeof(default_mqtt_port));
    strncpy(eeprom_data.calibration, default_calibration, sizeof(default_calibration));
  }
}

void writeSettingsESP() {
  int i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++) {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  memcrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  EEPROM.write(i++, p_memcrc[0]);
  EEPROM.write(i++, p_memcrc[1]);
  EEPROM.write(i++, p_memcrc[2]);
  EEPROM.write(i++, p_memcrc[3]);

  EEPROM.commit();
}

void configModeCallback (WiFiManager *myWiFiManager) {
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {}

void setupWiFi() {
  WiFiManager wifiManager;
  // Debug mode on
  // wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", eeprom_data.mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", eeprom_data.mqtt_port, 6);
  WiFiManagerParameter custom_calibration("calibration", "calibration", eeprom_data.calibration, 4);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_calibration);

  String ssid = "EnergyMeter_" + String(ESP.getChipId());

  if (!wifiManager.autoConnect(ssid.c_str(), "password")) {
    ESP.reset();
    delay(1000);
  }

  strcpy(eeprom_data.mqtt_server, custom_mqtt_server.getValue());
  strcpy(eeprom_data.mqtt_port, custom_mqtt_port.getValue());
  strcpy(eeprom_data.calibration, custom_calibration.getValue());

  WiFi.enableAP(0);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    Serial.print(" Current state = ");
    Serial.println(client.state());
    
    // Attempt to connect
    if (client.connect(device_name.c_str())) {
      Serial.println("connected");
      
      String config = "{\"state_topic\": \"" + power_state_topic + "\", \"device_class\": \"power\", \"name\": \"" + device_name + "\", \"unit_of_measurement\": \"W\"}";
      
      bool published = client.publish(power_config_topic.c_str(), config.c_str());
      if (published) {
        Serial.println(config);
      }
      String payload = "online";
      published = client.publish(availability_topic.c_str(), payload.c_str());
      if (published) {
        Serial.println(payload);
      } else {
        Serial.println("Can't publish");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  readSettingsESP();
  setupWiFi();
  writeSettingsESP();

  int port = atoi(eeprom_data.mqtt_port);
  client.setServer(eeprom_data.mqtt_server, port);
  client.setCallback(mqttCallback);

  Serial.begin(115200);
  
  emon1.current(A0, 26);             // Current: input pin, calibration.
}

int samples = 0;
bool lockSending = true;
int publishInterval = 5 * 1000;
int lastSend = 0;
double samplesAcc;
int samplesCount = 0;

void loop() {
  double Irms = emon1.calcIrms(1480);
  double consumption = Irms * 234.0;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (lockSending) {
    samples += 1;
    if (samples > 5) {
      lockSending = false;
    }

    return;
  }

  if (millis() - lastSend < publishInterval) {
    samplesAcc += consumption;
    samplesCount += 1;
  } else {
    lastSend = millis();

    Serial.print(samplesAcc);	
    Serial.print(" ");
    Serial.println(samplesCount);
    
    String payload;
    if (samplesCount > 0) {
       payload = String(samplesAcc / samplesCount);
    } else {
      payload = "0";
    }

    bool published = client.publish(power_state_topic.c_str(), payload.c_str());
    Serial.print("published: ");
    Serial.println(published);

    if (published) {
      samplesAcc = 0;
      samplesCount = 0;
    }
  }
}
