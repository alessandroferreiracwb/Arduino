#include "mqtt_client/mqtt_client.h"
#include "screens/screens.h" // Para drawMainScreen()
#include <string.h> // Para strncpy, strlen

// As instâncias WiFiClient e PubSubClient são definidas em globals.cpp

// Callback para mensagens MQTT recebidas
void mqttCallback(char* topic, byte* payload, unsigned int length) { // Corrigido o nome da função
  Serial.print("Mensagem MQTT recebida no topico: [");
  Serial.print(topic);
  Serial.print("] Payload: ");
  String message = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  Serial.println();

  // Exemplo: Se você quiser fazer algo com a mensagem recebida, faça aqui.
  // if (String(topic) == "seu/topico/comando") {
  //   // Lógica para o comando
  // }
}

// Tenta reconectar ao servidor MQTT de forma completamente não bloqueante
void reconnectMqtt() {
  if (!mqttClient.connected()) {
    static unsigned long lastAttemptTime = 0; // Armazena o tempo da última tentativa
    const long retryInterval = 5000; // Intervalo entre as tentativas de reconexão (5 segundos)

    // Tenta reconectar apenas se o tempo desde a última tentativa excedeu o intervalo,
    // ou se for a primeira tentativa (lastAttemptTime == 0).
    if (millis() - lastAttemptTime > retryInterval) {
      lastAttemptTime = millis(); // Atualiza o tempo da tentativa

      Serial.print("Tentando conexão MQTT com: ");
      Serial.print(mqttServerPath);
      Serial.print("...");
      currentMqttStatus = APP_MQTT_CONNECTING; // Define o status para "Conectando..." na UI

      // Lógica de conexão adaptada para Mosquitto sem autenticação (ou com, se configurado)
      bool connected;
      if (strlen(storedMqttUsername) > 0 && strlen(storedMqttPassword) > 0) {
        // Tenta conectar com usuário e senha
        connected = mqttClient.connect(mqttClientId, storedMqttUsername, storedMqttPassword);
      } else {
        // Tenta conectar sem usuário e senha (padrão para Mosquitto local)
        connected = mqttClient.connect(mqttClientId);
      }

      if (connected) {
        Serial.println("conectado!");
        currentMqttStatus = APP_MQTT_CONNECTED; // Define o status para "Conectado" na UI
        // Subscreve a tópicos aqui, se necessário
        // mqttClient.subscribe("seu/topico/controle");
        // Serial.println("Subscrito ao topico: seu/topico/controle");
        // publishBufferedData(); // Publica dados pendentes após a reconexão (chamado pelo loop principal)
      } else {
        Serial.print("falhou, rc=");
        Serial.print(mqttClient.state());
        Serial.println(". Proxima tentativa em 5 segundos.");
        currentMqttStatus = APP_MQTT_DISCONNECTED; // Define o status para "Desconectado" na UI
        // Não há delay() aqui. O loop() principal continuará executando a IHM.
      }
    }
  }
}

// Configura o cliente MQTT e tenta a primeira conexão
void setupMqttClient() {
  if (strlen(mqttServerPath) > 0) {
    if (!mqttClientConfigured) { // Só configura uma vez
        Serial.print("Configurando cliente MQTT para o servidor: ");
        Serial.println(mqttServerPath);
        mqttClient.setServer(mqttServerPath, 1883); // Porta MQTT padrão é 1883
        mqttClient.setCallback(mqttCallback); // Define a função de callback para mensagens recebidas
        mqttClientConfigured = true; // Marca como configurado
    }
    reconnectMqtt(); // Tenta conectar (agora não bloqueante)
  } else {
    Serial.println("Caminho do servidor MQTT vazio. Nao sera possivel configurar o cliente.");
    currentMqttStatus = APP_MQTT_DISCONNECTED; // Garante que o status na UI seja desconectado
  }
}

// Publica os dados atuais do sensor
void publishSensorData(float temp, float hum) {
  if (mqttClient.connected()) {
    char jsonBuffer[100]; // Buffer para o payload JSON
    char currentTimestamp[20];
    getFormattedDateTime(currentTimestamp, sizeof(currentTimestamp));

    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"temp\":%.1f, \"hum\":%.0f, \"time\":\"%s\"}", temp, hum, currentTimestamp);
    
    // Altere "parmalog/sensor/data" para o seu tópico desejado
    if (mqttClient.publish("parmalog/sensor/data", jsonBuffer)) {
      Serial.print("Publicado [parmalog/sensor/data]: ");
      Serial.println(jsonBuffer);
    } else {
      Serial.print("Falha ao publicar. Estado MQTT: ");
      Serial.println(mqttClient.state());
    }
  } else {
    // Se MQTT desconectado, armazena no buffer
    Serial.println("Cliente MQTT desconectado. Armazenando dados no buffer.");
    char currentTimestamp[20];
    getFormattedDateTime(currentTimestamp, sizeof(currentTimestamp));

    // Armazena no índice atual e avança (buffer circular)
    bufferedReadings[currentBufferIndex].temperature = temp;
    bufferedReadings[currentBufferIndex].humidity = hum;
    strncpy(bufferedReadings[currentBufferIndex].timestamp, currentTimestamp, sizeof(bufferedReadings[currentBufferIndex].timestamp) - 1);
    bufferedReadings[currentBufferIndex].timestamp[sizeof(bufferedReadings[currentBufferIndex].timestamp) - 1] = '\0'; // Garante null-termination

    currentBufferIndex++;
    if (currentBufferIndex >= MAX_BUFFERED_READINGS) {
      currentBufferIndex = 0; // Volta para o início (buffer circular)
      bufferFull = true; // Marca que o buffer já foi preenchido uma vez
    }
  }
}

// Publica os dados armazenados no buffer
void publishBufferedData() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT nao conectado, nao foi possivel publicar dados do buffer.");
    return;
  }

  Serial.println("Publicando dados pendentes do buffer...");
  int startIdx = 0;
  int numToPublish = 0;

  if (bufferFull) {
    startIdx = currentBufferIndex; // Começa do mais antigo (onde o índice circular parou)
    numToPublish = MAX_BUFFERED_READINGS;
  } else {
    startIdx = 0; // Começa do início
    numToPublish = currentBufferIndex; // Publica até o índice atual
  }

  for (int i = 0; i < numToPublish; ++i) {
    int actualIndex = (startIdx + i) % MAX_BUFFERED_READINGS;
    
    char jsonBuffer[100];
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"temp\":%.1f, \"hum\":%.0f, \"time\":\"%s\"}", 
             bufferedReadings[actualIndex].temperature, 
             bufferedReadings[actualIndex].humidity, 
             bufferedReadings[actualIndex].timestamp);

    if (mqttClient.publish("parmalog/sensor/buffered_data", jsonBuffer)) { // Tópico para dados em buffer
      Serial.print("Publicado [parmalog/sensor/buffered_data - ");
      Serial.print(actualIndex);
      Serial.print("]: ");
      Serial.println(jsonBuffer);
    } else {
      Serial.print("Falha ao publicar dado buffered (idx ");
      Serial.print(actualIndex);
      Serial.print("). Estado MQTT: ");
      Serial.println(mqttClient.state());
      // Se a publicação falhar, paramos, para não perder dados ao reiniciar
      break; 
    }
  }
  // Após tentar publicar tudo, se o MQTT ainda estiver conectado, limpa o buffer
  if (mqttClient.connected()) {
    currentBufferIndex = 0;
    bufferFull = false;
    Serial.println("Buffer de dados esvaziado.");
  }
}
