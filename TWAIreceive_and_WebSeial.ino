/*
 * IP de acesso: 192.168.4.1/webserial
No navegador, envie os comandos:
CAN 125 → configura a 125 kbps
CAN 250 → configura a 250 kbps
CAN 500 → configura a 500 kbps
CAN 1000 → configura a 1 Mbps

ESPAsyncWebServer 
WebSerialLite.h Versão 2.3
WebSerial.h Versão 2.1.1
*/

#include "driver/twai.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>           // Versão 1.1.4
#include <ESPAsyncWebServer.h>  // Versão 3.1 - Instalação only
#include <WebSerialLite.h>      // Versão 2.3
//#include <WebSerial.h>        // Removido 

int led=0;
// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5
#define RXD2 16
#define TXD2 17

AsyncWebServer server(80);

const char* ssid = "AP_ESP"; // Your WiFi AP SSID 
const char* password = "admin1234"; // Your WiFi Password


/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  String d = "";
  for(int i = 0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerial.println("Recebido: " + d);

  if(d == "20") digitalWrite(2, 1);
  else if(d == "21") digitalWrite(2, 0);
  else if (d.startsWith("CAN ")) {
    int bitrate = d.substring(4).toInt();
    startCAN(bitrate);
  } else {
    WebSerial.println("Comando desconhecido.");
  }
}


// Intervall:
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

bool startCAN(int bitrate_kbps) {
  if (driver_installed) {
    twai_stop();
    twai_driver_uninstall();
    driver_installed = false;
  }

  twai_timing_config_t t_config;
  switch (bitrate_kbps) {
    case 125:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      WebSerial.println("Velocidade CAN inválida!");
      return false;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    WebSerial.println("Falha ao instalar o driver CAN");
    return false;
  }

  if (twai_start() != ESP_OK) {
    WebSerial.println("Falha ao iniciar o driver CAN");
    return false;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  twai_reconfigure_alerts(alerts_to_enable, NULL);

  driver_installed = true;
  WebSerial.printf("CAN iniciada a %d kbps\n", bitrate_kbps);
  return true;
}


void setup() {
  // Inicializa a Serial2 com os pinos definidos
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Iniciado. Aguardando dados na Serial2...");
  
  pinMode(2, OUTPUT);
  // Start Serial:
  Serial.begin(9600);

  WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    //WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.onMessage(recvMsg);
    server.begin();

  // Initialize configuration structures using macro initializers
  startCAN(250); // CAN padrão inicial: 250 kbps


  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

static void handle_rx_message(twai_message_t& message) {
  String output = "";
  output += "ID: ";
  output += String(message.identifier, HEX);
  output.toUpperCase();  // Converte para maiúsculas
  output += " B";

  if (!(message.rtr)) {
    for (int i = 0; i < message.data_length_code; i++) {
      char buf[6];
      sprintf(buf, " : %02X", message.data[i]);
      output += String(buf);
    }
  }
  //WebSerial.println(output);  // Envia tudo de uma vez com \n incluso
  delay(500);
}


void loop() { 
  // Verifica se há dados disponíveis na Serial2
  while (Serial2.available()) {
    char c = Serial2.read();  // Lê o caractere da UART2
    Serial.print(c);          // Imprime na Serial principal
  }
  
   
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  } else {
    WebSerial.println("Sem dados ou Velocidade incompativel"); 
  }
}
/*Fim*/
