#define TINY_GSM_MODEM_SIM7000   // Esto debe ir PRIMERO
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "GPRS.h"
#define HARDWARE_2503 //Esta version es el modelo nuevo, se configuran los pines para este hardware
#define SERIAL_CAUDALIMETRO

#ifdef HARDWARE_2503
    #define PWR_KEY 32
    #define LED_BUILTIN  2 //Definido en pins_arduino.h

    /*   Definicion de PINES IO */
    #define PIN_WIFI_LED        18   // LED_1
    #define PIN_GPRS_LED        17   //LED _3
    #define PIN_GPS_LED         5     //LED_2
    #define PIN_SD_LED          16   //LED_4
    #define PIN_PIN_V_IN24      4  //No utilizada

    #define PIN_SERIAL_AT_RX    26
    #define PIN_SERIAL_AT_TX    27
    #define PIN_SERIAL_Q_RX     22         // Flow-meter receiving pin
    #define PIN_SERIAL_Q_TX     23         // Flow-meter transmitting pin

#else
    #define PWR_KEY 32
    #define LED_BUILTIN 2 //Definido en pins_arduino.h


    /*   Definicion de PINES IO */
    #define PIN_WIFI_LED        22
    #define PIN_GPRS_LED        21
    #define PIN_GPS_LED         25
    #define PIN_SD_LED          33
    #define PIN_PIN_V_IN24      4

    #define PIN_SERIAL_AT_RX    16
    #define PIN_SERIAL_AT_TX    17
    #define PIN_SERIAL_Q_RX     26         // Flow-meter receiving pin
    #define PIN_SERIAL_Q_TX     27         // Flow-meter transmitting pin

#endif
#define MODEM_BAUD 115200

#define AT_DEBUG 1

#define GPRS_PWRKEY_TIME 1200
#define MODEM_ON_MAX_RETRIES 6

#ifdef SERIAL_DEBUG
  #define DEBUG_PRINTLN(x)    Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTF(fmt, ...)    Serial.printf((fmt), ##__VA_ARGS__)
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif

#if AT_DEBUG
#include <StreamDebugger.h>
StreamDebugger debugger(Serial2, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(Serial2);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

const char apn[] = "igprs.claro.com.ar";
const char broker[] = "serverqst.com";
const int port = 1883;
const char topic[] = "esp32/test";

unsigned long lastSend = 0;
const unsigned long interval = 1000;

unsigned long gpsStartTime = 0;
bool gpsReady = false;

#define GPRS_PWRKEY_OFF_TIME 1200
#define GPRS_PWRKEY_ON_TIME 72

void activate(void)
{
  modem.enableGPS();
  delay(500);  // Retardo para asegurar la activación
  modem.sendAT("+CGNSPWR=1");
  delay(500);  // Retardo para asegurar la activación
  modem.sendAT("+CGNSSEQ=\"RMC\"");
  gpsStartTime = millis();  // Marca de tiempo para esperar fix

}

static void Sim7000_OnOff()
{
    DEBUG_PRINTLN("***GPRS OFF***");
   // portENTER_CRITICAL(&gprs_mux);
    digitalWrite(PWR_KEY, HIGH);
    vTaskDelay(GPRS_PWRKEY_OFF_TIME / portTICK_PERIOD_MS); // turn Off
    digitalWrite(PWR_KEY, LOW);
    vTaskDelay(5000 / portTICK_PERIOD_MS); // turn Off
    activate();
//    portEXIT_CRITICAL(&gprs_mux);
}

void connectMQTT() {
  if (mqtt.connected()) return;

  Serial.print("Conectando al broker MQTT...");
  if (mqtt.connect("esp32client")) {
    Serial.println("Conectado!");
  } else {
    Serial.println("Fallo conexión MQTT");
  }
}
static bool Sim7000_PowerOn(void)
{
    bool success = false;
    uint8_t tries = 0;
    DEBUG_PRINTLN("***Sim7000_PowerOn***");

    if (!modem.testAT(2000)) 
      {
        do {//Incertidumbre si mandamos un power on o off
            //Hacemos 3 intentos revisando la conexion serie con testAT
            //Si esto ultimo tiene exito determinamos haber encendido el modem
            //Un flanco en bajo de 1200ms enciende o apaga el modem
            //Pongo PWRKEY en BAJO sobre el SIM7000
            //https://en.simcom.com/service-194.html
            //https://en.simcom.com/product/SIM7000G.html
                        
            digitalWrite(PWR_KEY, HIGH);
            //Espero el tiempo indicado
            vTaskDelay(GPRS_PWRKEY_TIME / portTICK_PERIOD_MS); // turn Off
            //Pongo PWRKEY en ALTO sobre el SIM7000
            digitalWrite(PWR_KEY, LOW); //LOW level on normal execution

            vTaskDelay(2000 / portTICK_PERIOD_MS);

            success = modem.testAT(5000);//Timeout 10 segundos para intentos test AT
           Serial.print("Reintento:*** ");
            Serial.print(tries);
            Serial.println(" ***");

            tries++;
        } while (!success && tries < MODEM_ON_MAX_RETRIES);
      }
    else 
      {
      success = true;
      }
    if (success) 
      {
      if (!modem.restart()) 
        {
          success = false;
          DEBUG_PRINTLN("***Sim7000 FALLA modem.restart***");
        }
      if (!modem.enableGPS()) 
        {
          success = false;
          DEBUG_PRINTLN("***Sim7000 FALLA modem.enableGPS***");
        } 
      else
          DEBUG_PRINTLN("***Sim7000 ENCENDIDO***");        
      }
    return success;
    //Toda la secuencia se repite MODEM_ON_MAX_RETRIES veces
}

int k;
// Timer - /*********************Interrupcion para ver la vida del ESP con el LED_BLUITIN ***************  */
hw_timer_t *timer = NULL;
volatile bool ledState = false;
void IRAM_ATTR onTimer() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
}
/*********************************************************** */

void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(PWR_KEY, OUTPUT);

  digitalWrite(PWR_KEY, LOW);
pinMode(LED_BUILTIN, OUTPUT);
   /*********************Interrupcion para ver la vida del ESP con el LED_BLUITIN ***************  */
    digitalWrite(LED_BUILTIN, HIGH);
  // Configura el timer 0, con prescaler de 80 (1 tick = 1 µs)
  timer = timerBegin(0, 80, true);
  
  // Asocia la interrupción
  timerAttachInterrupt(timer, &onTimer, true);

  // 500000 µs = 500 ms
  timerAlarmWrite(timer, 500000, true);  // Repetir cada 500 ms

  // Inicia la alarma
  timerAlarmEnable(timer);
  /************************************************************* */

  Serial2.begin(MODEM_BAUD, SERIAL_8N1, PIN_SERIAL_AT_RX, PIN_SERIAL_AT_TX);
  //delay(3000);
  Sim7000_PowerOn();
  Serial.println("-----------Saliendo del power ON ---------");
  Serial.println("Inicializando módem...");
  modem.restart();

  bool modemOk = modem.testAT();
  Serial.printf("Resultado de testAT(): %s\n", modemOk ? "OK" : "FALLÓ");

  initGPRS(modem);

  mqtt.setServer(broker, port);

  // 🔧 Activar GPS
  Serial.println("Encendiendo GPS...");

  activate();
}

void loop() {
  // Leer comando desde Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // Quita espacios en blanco o saltos de línea

    if (cmd == "ON_OFF") {
      Serial.println("Recibido comando ON_OFF");
      Sim7000_OnOff();
    }
  }
  checkNetwork();

  if (netState == NET_CONNECTED) {
    connectMQTT();
    mqtt.loop();

    // Verificar si ya pasó tiempo suficiente para obtener fix (10 segundos)
    if (!gpsReady && millis() - gpsStartTime > 10000) {
      gpsReady = true;
      Serial.println("GPS debería estar listo. Consultando estado...");
    }
 if (!modem.testAT(2000)) {
            if (!Sim7000_PowerOn()) {
                DEBUG_PRINTLN("Nucleo0, FALLA COMUNICACION GPRS!!!");
            }
        }        
    if (millis() - lastSend > interval) {
      lastSend = millis();

      if (gpsReady) {
        // Verificar el estado de GNSS
        modem.sendAT("+CGNSPWR?");
        String pwr = modem.stream.readStringUntil('\n');
        Serial.println("Estado CGNSPWR: " + pwr);

        // 🔍 Mostrar info del GPS
        modem.sendAT("+CGNSINF");
        delay(100); // Esperar la respuesta
        while (modem.stream.available()) {
          Serial.write(modem.stream.read());  // Mostrar la respuesta cruda
        }

        // 🔍 Obtener ubicación GPS
        float lat, lon;
        if (modem.getGPS(&lat, &lon)) {
          String payload = "Lat: " + String(lat, 6) + ", Lon: " + String(lon, 6);
          Serial.print("Publicando: ");
          Serial.println(payload);

          if (mqtt.publish(topic, payload.c_str())) {
            Serial.println("Publicado OK");
            Serial.println(k);
          } else {
            Serial.println("Fallo al publicar");
          }
        } else {
          Serial.println("GPS no disponible aun o sin fix.");
        }
      } else {
        Serial.println("Esperando a que el GPS se estabilice...");
      }
    }
  }
}
