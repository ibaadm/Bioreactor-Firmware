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
const char* MQTT_TOPIC  = "project/counter";

WiFiClientSecure wifi_client;
unsigned long last_wifi_connection_attempt = 0;
const int WIFI_CONNECTION_DELAY = 10000;
bool detected_wifi_connection = false;

PubSubClient mqttClient(wifi_client);
unsigned long last_mqtt_connection_attempt = 0;
const int MQTT_CONNECTION_DELAY = 5000;
unsigned long last_publish = 0;

int counter = 0;
unsigned long last_counter_update = 0;

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
      // subscribe to a command topic here
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

void updateCounter() {
  if (millis() - last_counter_update >= 2000) {
    counter++;
    last_counter_update = millis();
  }
}

void publishTelemetry() {
  if (millis() - last_publish >= 1000) {

    char msg[50];
    snprintf(msg, sizeof(msg), "{\"counter\": %d}", counter);

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

  updateCounter();

  if (maintainConnection()) {
    mqttClient.loop();
    publishTelemetry();
  }
}