// incluimos librerias
#include <ZMPT101B.h>
#include "ACS712.h"
#include <ESP32Time.h>

//parte de wifi para debug quitar cuando sirva 
#include "WiFi.h"
#include <NetworkUdp.h>
const char *ssid = "SANTY0W0 8645";
const char *password = "123456789";
const char *udpAddress = "192.168.137.160";
const char *udpAddress1 = "192.168.137.117";
const int udpPort = 8000;
NetworkUDP udp;
//fin debug    

// Se define la sensibilidad para el sensor de corriente
#define SENSITIVITY 500.0f

// Se definen los pines donde están conectados los relés
#define Rele1 3
#define Rele2 4 
#define Rele3 5
#define Rele4 6

#define PIN_ACS1    8
#define PIN_ACS2    9
#define PIN_ACS3    10
#define PIN_ACS4    11
#define PIN_VOLTAGE 1  // Usar GP1

ZMPT101B voltageSensor(PIN_VOLTAGE, 60.0);
ACS712 ACS1(PIN_ACS1, 3.32, 4096, 66);
ACS712 ACS2(PIN_ACS2, 3.32, 4096, 66);
ACS712 ACS3(PIN_ACS3, 3.32, 4096, 66);
ACS712 ACS4(PIN_ACS4, 3.32, 4096, 66);


// Se declara un reloj en tiempo real
ESP32Time rtc(0);

// Se definen los umbrales de protección
const float voltajeMax = 139.7;   // Limite maximo de voltaje
const float voltajeMin = 100.0;   // Límite inferior de voltaje (inseguro)
const float corrienteMax = 2000;   // Corte por sobrecorriente
const float corrienteMin = 30;  // Carga completa si baja de este valor (prueba 30-100)

// Se definen las variables para la temporización
unsigned long lastMeasurementTime = 0;             // Almacena el tiempo de la última medición
const unsigned long measurementInterval = 5000;  // Intervalo de medición en milisegundos (1 minutos)

// Prototipos de funciones
float leerCorriente(ACS712& sensor);
float getRmsVoltage(int loopCount);
void apagarTodo();
void verificarCarga(float corriente, int relePin);


void setup() {
  Serial.begin(115200);
 
//debug
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
  Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
  udp.begin(WiFi.localIP(), udpPort);
  //debug y mas debug
  // Configuración de los pines de salida para los relés


  pinMode(Rele1, OUTPUT);
  pinMode(Rele2, OUTPUT);
  pinMode(Rele3, OUTPUT);
  pinMode(Rele4, OUTPUT);

  // Inicialización de los relés (todos encendidos inicialmente)
  digitalWrite(Rele1, LOW);
  digitalWrite(Rele2, LOW);
  digitalWrite(Rele3, LOW);
  digitalWrite(Rele4, LOW);

  // Configuración de los sensores de corriente para calibrar y obtener lecturas correctas
  ACS1.autoMidPoint(60,30);
  ACS2.autoMidPoint(60,30);
  ACS3.autoMidPoint(60,30);
  ACS4.autoMidPoint(60,30);

  // Ajustar la sensibilidad del sensor de voltaje ZMPT101B
  voltageSensor.setSensitivity(SENSITIVITY);

  // Configura el reloj en tiempo real  con una hora inicial predeterminada
  rtc.setTime(0, 0, 0, 1, 1, 2025);
  /*borrrar
  // Se asegura de que los relés estén apagados al inicio del sistema
  digitalWrite(Rele1, HIGH);
  digitalWrite(Rele2, HIGH);
  digitalWrite(Rele3, HIGH);
  digitalWrite(Rele4, HIGH);*/
}

void loop() {
  float V = 0;
  float mA1 = 0;
  float mA2 = 0;
  float mA3 = 0;
  float mA4 = 0;
  float P1 = 0;
  float P2 = 0;
  float P3 = 0;
  float P4 = 0;

  // Verificar si han pasado 10 minutos desde la última medición
  if (millis() - lastMeasurementTime >= measurementInterval) {
    
  Serial.println("🟢 Loop activo");
    // Actualizar el tiempo de la última medición
    lastMeasurementTime = millis();

    // Se mide el voltaje de la red eléctrica utilizando el sensor ZMPT101B
    V = getRmsVoltage(10);  // Promediar 10 mediciones para obtener un valor de voltaje confiable

    // Medir la corriente de cada uno de los dispositivos conectados (4 dispositivos)
    mA1 = leerCorriente(ACS1);  // Medir corriente del dispositivo 1
    mA2 = leerCorriente(ACS2);  // Medir corriente del dispositivo 2
    mA3 = leerCorriente(ACS3);  // Medir corriente del dispositivo 3
    mA4 = leerCorriente(ACS4);  // Medir corriente del dispositivo 4

    P1 = V * (mA1 / 1000.0);
    P2 = V * (mA2 / 1000.0);
    P3 = V * (mA3 / 1000.0);
    P4 = V * (mA4 / 1000.0);

    //debug web corriente
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Prueba");
    udp.printf("\nvolaje: %.2f", V);
    
    udp.printf("\n1 Corriente: %.2f,  Wats: %.2f", mA1, P1);
    udp.printf("\n2 Corriente: %.2f,  Wats: %.2f", mA2, P2);
    udp.printf("\n3 Corriente: %.2f,  Wats: %.2f", mA3, P3);
    udp.printf("\n4 Corriente: %.2f,  Wats: %.2f", mA4, P4);
    udp.endPacket();
    //fin

    //debug web corriente
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Prueba");
    udp.printf("\nvolaje: %.2f", V);
    
    udp.printf("\n1 Corriente: %.2f,  Wats: %.2f", mA1, P1);
    udp.printf("\n2 Corriente: %.2f,  Wats: %.2f", mA2, P2);
    udp.printf("\n3 Corriente: %.2f,  Wats: %.2f", mA3, P3);
    udp.printf("\n4 Corriente: %.2f,  Wats: %.2f", mA4, P4);
    udp.endPacket();
    //fin

    if (V > voltajeMax || V < voltajeMin) {
      apagarTodo();  // protección por voltaje
    } else {
      // Evaluar cada carga
      verificarCarga(mA1, Rele1);
      verificarCarga(mA2, Rele2);
      verificarCarga(mA3, Rele3);
      verificarCarga(mA4, Rele4);
    }

    // Esperar 1 segundo antes de realizar otra medición (para dar tiempo a estabilizar el sistema)
    delay(1000);
  }
}

// Función para leer la corriente a través del sensor ACS712
float leerCorriente(ACS712& sensor) {
  long suma = 0;       // Variable para almacenar la suma de las lecturas
  int muestras = 100;  // Número de muestras a tomar para promediar

  // Tomar 100 lecturas de corriente
  for (int i = 0; i < muestras; i++) {
    suma += sensor.mA_AC();  // Añadir cada lectura de corriente
    delay(10);
  }

  // Calcular el promedio de las lecturas
  float mA = suma / float(muestras);
  return mA;  // Devolver el valor promedio de la corriente
}

// Función para medir el voltaje RMS (Raíz Cuadrada Media) de la red eléctrica
float getRmsVoltage(int loopCount) {
  long suma = 0;  // Variable para almacenar la suma de las lecturas

  // Se toman múltiples lecturas del sensor de voltaje y promediarlas
  for (int i = 0; i < loopCount; i++) {
    suma += voltageSensor.getRmsVoltage();  // Medir el voltaje y sumarlo
  }

  // Calcular el promedio de las mediciones
  float promedio = suma / loopCount;
  return promedio;  // Devolver el voltaje promedio
}

// Función para apagar todos los relés y desconectar todos los dispositivos
void apagarTodo() {
  digitalWrite(Rele1, HIGH);  // Apagar el relé del dispositivo 1
  digitalWrite(Rele2, HIGH);  // Apagar el relé del dispositivo 2
  digitalWrite(Rele3, HIGH);  // Apagar el relé del dispositivo 3
  digitalWrite(Rele4, HIGH);  // Apagar el relé del dispositivo 4

  //debug web corriente
    udp.beginPacket(udpAddress, udpPort);
    udp.printf("Se ha apagado por voltaje peligroso");
    udp.endPacket();
  //debug web corriente
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Se ha apagado por voltaje peligroso");
    udp.endPacket();
}

void verificarCarga(float corriente, int relePin) {
  if (corriente >= corrienteMax) {
    // Protección por sobrecorriente
    digitalWrite(relePin, HIGH);  // Apaga el relé
  } else if (corriente < corrienteMin) {
    // Dispositivo completamente cargado (consume muy poco)
    digitalWrite(relePin, HIGH);  // Apaga el relé
    
    udp.beginPacket(udpAddress, udpPort);
    udp.printf("✅ Rele en pin %d APAGADO por carga completa (%.2f mA)", relePin, corriente);
    udp.endPacket();
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("✅ Rele en pin %d APAGADO por carga completa (%.2f mA)", relePin, corriente);
    udp.endPacket();
  } else {
    // Está cargando correctamente
    digitalWrite(relePin, LOW);  // Mantener carga activa
  }
}
