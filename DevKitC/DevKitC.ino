#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_eap_client.h>
#include <PubSubClient.h>
#include "cert.h"
#include "secrets.h"

const char* WIFI_SSID = "eduroam";

const char* MQTT_BROKER = "b127666df1f5484f8d4b824052b92e6f.s1.eu.hivemq.cloud";
const int   MQTT_PORT   = 8883;
const char* MQTT_USER   = "ENGF0001";
const char* MQTT_TOPIC  = "project/telemetry";

WiFiClientSecure wifi_client;
unsigned long last_wifi_connection_attempt = 0;
const int WIFI_CONNECTION_DELAY = 10000;
bool detected_wifi_connection = false;

PubSubClient mqttClient(wifi_client);
unsigned long last_mqtt_connection_attempt = 0;
const int MQTT_CONNECTION_DELAY = 5000;
unsigned long last_publish = 0;

float ph = 5.0;
float temperature = 30.0;
int rpm = 1000;
unsigned long last_sensor_update = 0;

void setConnectionDetails() {
  wifi_client.setCACert(root_ca);

  WiFi.mode(WIFI_STA);
  esp_eap_client_set_identity((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_username((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_password((uint8_t*)SECRET_WIFI_PASS, strlen(SECRET_WIFI_PASS));
  esp_wifi_sta_enterprise_enable();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
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
      // TODO: subscribe to a command topic here
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  randomSeed(esp_random()); // temporary, see readSensorData()

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

void readSensorData() { // temporarily generate random values
  if (millis() - last_sensor_update >= 1000) {
    last_sensor_update = millis();
    ph += random(-20, 21) / 100.0;
    ph = min(max(ph, 3.0f), 7.0f);
    temperature += random(-5, 6) / 10.0;
    temperature = min(max(temperature, 25.0f), 35.0f);
    rpm += random(-10, 11);
    rpm = min(max(rpm, 500), 1500);
  }
}

void publishTelemetry() {
  if (millis() - last_publish >= 1000) {

    char msg[128];
    snprintf(msg, sizeof(msg), 
             "{\"ph\": %.2f, \"temperature\": %.1f, \"rpm\": %d}", 
             ph, temperature, rpm);

    if (mqttClient.publish(MQTT_TOPIC, msg)) {
      Serial.print("Published: ");
      Serial.println(msg);
    } else {
      Serial.println("Publish failed!");
    }

    last_publish = millis();
  }
}

void loop() {

  readSensorData();

  if (maintainConnection()) {
    mqttClient.loop();
    publishTelemetry();
  }
}