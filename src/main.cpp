#define TINY_GSM_MODEM_SIM7000   // Esto debe ir PRIMERO
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "GPRS.h"

#define Hard2502
#ifdef Hard2502
#define MODEM_RX 26
#define MODEM_TX 27
#else
#define MODEM_RX 16
#define MODEM_TX 17
#endif
#define MODEM_BAUD 115200

#define PWKEY_PIN 32

TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

const char apn[] = "igprs.claro.com.ar";
const char broker[] = "serverqst.com";
const int port = 1883;
const char topic[] = "esp32/test";

unsigned long lastSend = 0;
const unsigned long interval = 1000;

void connectMQTT() {
  if (mqtt.connected()) return;

  Serial.print("Conectando al broker MQTT...");
  if (mqtt.connect("esp32client")) {
    Serial.println("Conectado!");
  } else {
    Serial.println("Fallo conexión MQTT");
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(PWKEY_PIN, OUTPUT);
  digitalWrite(PWKEY_PIN, HIGH);

  Serial2.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  Serial.println("Inicializando módem...");
  modem.restart();

  bool modemOk = modem.testAT();
  Serial.printf("Resultado de testAT(): %s\n", modemOk ? "OK" : "FALLÓ");

  initGPRS(modem);

  mqtt.setServer(broker, port);

  // 🔧 Activar GPS
  Serial.println("Encendiendo GPS...");
  modem.enableGPS();
}

void loop() {
  checkNetwork();

  if (netState == NET_CONNECTED) {
    connectMQTT();
    mqtt.loop();

    if (millis() - lastSend > interval) {
      lastSend = millis();

      // 🔍 Obtener ubicación GPS
      float lat, lon;
      if (modem.getGPS(&lat, &lon)) {
        String payload = "Lat: " + String(lat, 6) + ", Lon: " + String(lon, 6);
        Serial.print("Publicando: ");
        Serial.println(payload);

        if (mqtt.publish(topic, payload.c_str())) {
          Serial.println("Publicado OK");
        } else {
          Serial.println("Fallo al publicar");
        }
      } else {
        Serial.println("GPS no disponible aún.");
      }
    }
  }
}
