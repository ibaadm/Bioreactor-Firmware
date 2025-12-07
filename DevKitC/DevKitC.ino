#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_eap_client.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "cert.h"
#include "secrets.h"

const int RX_PIN = 32;
const int TX_PIN = 33;

const char* WIFI_SSID = "eduroam";

const char* MQTT_BROKER = "b127666df1f5484f8d4b824052b92e6f.s1.eu.hivemq.cloud";
const int   MQTT_PORT   = 8883;
const char* MQTT_USER   = "ENGF0001";
const char* MQTT_TOPIC_TELEMETRY  = "bioreactor/telemetry";
const char* MQTT_TOPIC_CONTROL = "bioreactor/control/#";

WiFiClientSecure wifi_client;
unsigned long last_wifi_connection_attempt = 0;
const int WIFI_CONNECTION_DELAY = 10000;
bool detected_wifi_connection = false;

PubSubClient mqttClient(wifi_client);
unsigned long last_mqtt_connection_attempt = 0;
const int MQTT_CONNECTION_DELAY = 5000;
unsigned long last_publish = 0;

void setConnectionDetails() {
  wifi_client.setCACert(root_ca);

  WiFi.mode(WIFI_STA);
  esp_eap_client_set_identity((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_username((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_password((uint8_t*)SECRET_WIFI_PASS, strlen(SECRET_WIFI_PASS));
  esp_wifi_sta_enterprise_enable();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void attemptWiFiConnection() {
  if (millis() - last_wifi_connection_attempt >= WIFI_CONNECTION_DELAY) {
    last_wifi_connection_attempt = millis();
    Serial.printf("Connecting to WiFi (%s)...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID);
  }
}

void attemptMQTTConnection() {
  if (millis() - last_mqtt_connection_attempt >= MQTT_CONNECTION_DELAY) {
    last_mqtt_connection_attempt = millis();
    Serial.println("Connecting to HiveMQ...");
    if (mqttClient.connect("ESP32Client", MQTT_USER, SECRET_MQTT_PASS)) {
      Serial.println("Connected to HiveMQ");
      mqttClient.subscribe(MQTT_TOPIC_CONTROL);
      Serial.println("Subscribed to project/setpoints");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  setConnectionDetails();
  
  attemptWiFiConnection();
  attemptMQTTConnection();
}

bool maintainConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (detected_wifi_connection) {
      Serial.println("WiFi disconnected");
      detected_wifi_connection = false;
    }
    attemptWiFiConnection();
    return false;
  }
  if (!detected_wifi_connection) {
    Serial.printf("Connected to WiFi (%s)\n", WIFI_SSID);
    detected_wifi_connection = true;
  }

  if (!mqttClient.connected()) {
    attemptMQTTConnection();
    return false;
  }

  return true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  
  Serial2.write(payload, length);
  Serial2.println();
  Serial.println("Received control JSON message and forwarded it to Nano");
}

void readNanoData() {
  if (Serial2.available() > 0) {
    Serial.println("Success");
    String incomingJson = Serial2.readStringUntil('\n');

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, incomingJson);

    if (error) {
      Serial.print(F("Nano Data Error: "));
      Serial.println(error.f_str());
      return;
    }

    publishTelemetry(doc["ph"], doc["temperature"], doc["rpm"]);
  }
}

void publishTelemetry(double ph, double temperature, double rpm) {

  char msg[128];
  snprintf(msg, sizeof(msg), 
            "{\"ph\": %.2f, \"temperature\": %.1f, \"rpm\": %d}", 
            ph, temperature, rpm);

  if (mqttClient.publish(MQTT_TOPIC_TELEMETRY, msg)) {
    Serial.print("Published: ");
    Serial.println(msg);
  } else {
    Serial.println("Publish failed!");
  }
}

void loop() {

  readNanoData();

  if (maintainConnection()) {
    mqttClient.loop();
  }
}