#include "GPRS.h"

static TinyGsm* modemPtr = nullptr;
NetworkState netState = NET_DISCONNECTED;
unsigned long lastCheck = 0;
const unsigned long checkInterval = 1000;

void initGPRS(TinyGsm& modemRef) {
  modemPtr = &modemRef;
}

void checkNetwork() {
  if (!modemPtr) return;

  unsigned long now = millis();
  if (now - lastCheck < checkInterval) return;
  lastCheck = now;

  switch (netState) {
    case NET_DISCONNECTED:
      Serial.println("Esperando red...");
      if (modemPtr->isNetworkConnected()) {
        netState = NET_REGISTERED;
      }
      break;

    case NET_REGISTERED:
      Serial.println("Intentando conectar GPRS...");
      modemPtr->gprsConnect("igprs.claro.com.ar", "", "");
      netState = NET_GPRS_CONNECTING;
      break;

    case NET_GPRS_CONNECTING:
      if (modemPtr->isGprsConnected()) {
        Serial.println("GPRS conectado");
        netState = NET_CONNECTED;
      }
      break;

    case NET_CONNECTED:
      if (!modemPtr->isGprsConnected()) {
        Serial.println("GPRS perdido, reconectando...");
        netState = NET_DISCONNECTED;
      }
      break;

    default:
      break;
  }
}
