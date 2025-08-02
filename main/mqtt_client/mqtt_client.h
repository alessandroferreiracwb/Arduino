#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "../globals.h" // Para PubSubClient e variáveis globais
#include <WiFi.h> // Para WiFiClient e verificação de conexão WiFi

// Protótipos das funções do cliente MQTT
void setupMqttClient();
void mqttCallback(char* topic, byte* payload, unsigned int length); // Corrigido o nome do protótipo
void reconnectMqtt();
void publishSensorData(float temp, float hum);
void publishBufferedData();

#endif // MQTT_CLIENT_H