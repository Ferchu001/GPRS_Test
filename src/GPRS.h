#ifndef GPRS_H
#define GPRS_H

#ifndef TINY_GSM_MODEM_SIM7000
#define TINY_GSM_MODEM_SIM7000
#endif
#include <TinyGsmClient.h>


enum NetworkState {
  NET_DISCONNECTED,
  NET_WAITING,
  NET_REGISTERED,
  NET_GPRS_CONNECTING,
  NET_CONNECTED
};

extern NetworkState netState;
extern unsigned long lastCheck;
extern const unsigned long checkInterval;

void initGPRS(TinyGsm& modemRef);
void checkNetwork();

#endif
