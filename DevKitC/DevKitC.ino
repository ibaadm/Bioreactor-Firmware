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

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
unsigned long lastPublish = 0;

int counter = 0;
unsigned long lastCounterUpdate = 0;

/* notes
handle wifi (and mqtt?) disconnecting midway through
*/

void connectWiFiEduroam() {
  Serial.println("Connecting to Eduroam...");

  WiFi.mode(WIFI_STA);
  esp_eap_client_set_identity((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_username((uint8_t*)SECRET_WIFI_USER, strlen(SECRET_WIFI_USER));
  esp_eap_client_set_password((uint8_t*)SECRET_WIFI_PASS, strlen(SECRET_WIFI_PASS));
  esp_wifi_sta_enterprise_enable();

  WiFi.begin(WIFI_SSID);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Eduroam!");
  }
}

void connectMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  while (!mqttClient.connected()) {
    Serial.print("Connecting to HiveMQ...");
    if (mqttClient.connect("ESP32Client", MQTT_USER, SECRET_MQTT_PASS)) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  wifiClient.setCACert(root_ca);
  connectWiFiEduroam();
  connectMQTT();
}

void updateCounter() {
  if (millis() - lastCounterUpdate >= 2000) {
    counter++;
    lastCounterUpdate = millis();
  }
}

void publishTelemetry() {
  if (millis() - lastPublish >= 1000) {

    char msg[20];
    snprintf(msg, sizeof(msg), "{\"counter\": %d}", counter);

    if (mqttClient.publish(MQTT_TOPIC, msg)) {
      Serial.print("Published: ");
      Serial.println(msg);
    } else {
      Serial.println("Publish failed!");
    }

    lastPublish = millis();
  }
}

void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  updateCounter();
  publishTelemetry();
}