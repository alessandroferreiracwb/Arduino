/* ESP32 TWAI receive example.
  Receive messages and sends them over serial.

  Connect a CAN bus transceiver to the RX/TX pins.
  For example: SN65HVD230

  TWAI_MODE_LISTEN_ONLY is used so that the TWAI controller will not influence the bus.

  The API gives other possible speeds and alerts:
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html

  Example output from a can bus message:
  -> Message received
  -> Message is in Standard Format
  -> ID: 604
  -> Byte: 0 = 00, 1 = 0f, 2 = 13, 3 = 02, 4 = 00, 5 = 00, 6 = 08, 7 = 00

  Example output with alerts:
  -> Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.
  -> Bus error count: 171
  -> Alert: The RX queue is full causing a received frame to be lost.
  -> RX buffered: 4  RX missed: 46 RX overrun 0

  created 05-11-2022 by Stephan Martin (designer2k2)
*/

#include "driver/twai.h"

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>

AsyncWebServer server(80);

const char* ssid = "AP_ESP"; // Your WiFi AP SSID 
const char* password = "admin1234"; // Your WiFi Password

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervall:
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

void setup() {
  // Start Serial:
  Serial.begin(9600);

  WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.onMessage(recvMsg);
    server.begin();

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

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

  WebSerial.println(output);  // Envia tudo de uma vez com \n incluso
  delay(500);
}


void loop() {
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
  }
    //delay(1000);
    /****************************************************************/
    // we suggest you to use `print + "\n"` instead of `println`
    // because the println will send "\n" separately, which means
    // it will cost a sending buffer just for storage "\n". (there
    // only 8 buffer space in ESPAsyncWebServer by default)
    //WebSerial.print(F("IP address: "));
    // if not use toString the ip will be sent in 7 part,
    // which means it will cost 7 sending buffer.
    //WebSerial.println(WiFi.localIP().toString());
    //WebSerial.printf("Millis=%lu\n", millis());
    //WebSerial.printf("Free heap=[%u]\n", ESP.getFreeHeap());
    /************************************************************/
  
}
