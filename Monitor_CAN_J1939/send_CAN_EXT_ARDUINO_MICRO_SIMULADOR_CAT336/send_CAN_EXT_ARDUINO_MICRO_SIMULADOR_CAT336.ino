// Envia Can com Modulo MC2515
// Simula o funcionamento de um equipamento com valores incrementados

#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"

// Define o pino CS (Chip Select) para o pino 10
const int SPI_CS_PIN = 10;

// Cria uma instância da classe mcp2515_can
mcp2515_can CAN(SPI_CS_PIN); 

// Variáveis para os dados que você quer enviar, incluindo os contadores
unsigned char rpm_counter = 0;
bool rpm_direction = true; // true = contando para cima, false = contando para baixo

unsigned char dial_counter = 100;
bool dial_direction = false;

unsigned char temp_counter = 50;
bool temp_direction = true;

unsigned char comb_counter = 200;
bool comb_direction = false;

// Variáveis para os dados que não serão contadores
unsigned char bat = 0x0C;     
unsigned char lebre = 0x01;   
unsigned char x = 0x0A;       
unsigned char y = 0x0B;       
unsigned char hyd = 0x01;     

// Função para atualizar um contador (incrementa/decrementa)
void updateCounter(unsigned char &counter, bool &direction) {
  if (direction) {
    if (counter < 255) {
      counter++;
    } else {
      direction = false; // Chegou ao limite, inverte a direção
      counter--;
    }
  } else {
    if (counter > 0) {
      counter--;
    } else {
      direction = true; // Chegou a zero, inverte a direção
      counter++;
    }
  }
}

void setup() {
    Serial.begin(115200);

    // Inicie o barramento CAN com a velocidade de 500kbps e oscilador de 8MHz
    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) { 
        Serial.println("Falha na inicialização do CAN. Tentando novamente...");
        delay(100);
    }
    Serial.println("CAN inicializado com sucesso a 500kbps!");
    
    // Define o modo de operação normal
    CAN.setMode(0);
}

void loop() {
    // Atualiza os contadores a cada loop
    updateCounter(rpm_counter, rpm_direction);
    updateCounter(dial_counter, dial_direction);
    updateCounter(temp_counter, temp_direction);
    updateCounter(comb_counter, comb_direction);

    unsigned char data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 1. Electronic Engine Controller 1 - RPM (ID Estendido)
    // ID: 0x0CF00400, Data[4] = rpm_counter
    data[4] = rpm_counter;
    CAN.sendMsgBuf(0x0CF00400, 1, 8, data);
    delay(10);
    
    // 2. Dial (ID Estendido)
    // ID: 0x18FED9F7, Data[4] = dial_counter
    data[4] = dial_counter;
    CAN.sendMsgBuf(0x18FED9F7, 1, 8, data);
    delay(10);
    
    // 3. Engine Temperature 1 (ID Estendido)
    // ID: 0x18FEEE00, Data[0] = temp_counter
    data[0] = temp_counter;
    CAN.sendMsgBuf(0x18FEEE00, 1, 8, data);
    delay(10);
    
    // 4. Fuel Economy (Liquid) (ID Estendido)
    // ID: 0x18FEF200, Data[0] = comb_counter, Data[1] = 0xFF
    data[0] = comb_counter;
    data[1] = 0xFF;
    CAN.sendMsgBuf(0x18FEF200, 1, 8, data);
    delay(10);
    
    // 5. Vehicle Electrical Power 1 (ID Estendido)
    // ID: 0x18FEF700, Data[4] = bat, Data[5] = 0x2
    data[4] = bat;
    data[5] = 0x02;
    CAN.sendMsgBuf(0x18FEF700, 1, 8, data);
    delay(10);
    
    // 6. Dash Combustivel (ID Estendido)
    // ID: 0x18FEFC2E, Data[1] = comb_counter
    data[1] = comb_counter;
    CAN.sendMsgBuf(0x18FEFC2E, 1, 8, data);
    delay(10);

    // 7. Lebre/Tartaruga (ID Estendido)
    // ID: 0x18A7FFF7, Data[2] = lebre
    data[2] = lebre;
    CAN.sendMsgBuf(0x18A7FFF7, 1, 8, data);
    delay(10);
    
    // 8. Vehicle Fluids (ID Estendido)
    // ID: 0x18FE682E, Data[0] = comb_counter
    data[0] = comb_counter;
    CAN.sendMsgBuf(0x18FE682E, 1, 8, data);
    delay(10);

    // 9. Inclinometro (ID Padrão)
    // ID: 0x50, Data[0]=0x01, Data[1]=0x05, Data[2]=0x02, Data[3]=x
    data[0] = 0x01;
    data[1] = 0x05;
    data[2] = 0x02;
    data[3] = x;
    CAN.sendMsgBuf(0x50, 0, 8, data);
    delay(10);
    
    // 10. Inclinômetro Lohr (ID Estendido)
    // ID: 0x0CF013A2, Data[0]=x, Data[1]=y
    data[0] = x;
    data[1] = y;
    CAN.sendMsgBuf(0x0CF013A2, 1, 8, data);
    delay(10);
    
    // 11. Bloqueio Hyd mp (ID Estendido)
    // ID: 0x18EF002E, Data[0]=0x95, Data[1]=0x00, Data[3]=0x01&hyd
    data[0] = 0x95;
    data[1] = 0x00;
    data[3] = 0x01 & hyd;
    CAN.sendMsgBuf(0x18EF002E, 1, 8, data);
    delay(10);
    
    // 12. Slope Sensor Information 2 (ID Estendido)
    // ID: 0x0CF029E2, Data[0,1,2]=0xAA
    data[0] = 0xAA;
    data[1] = 0xAA;
    data[2] = 0xAA;
    CAN.sendMsgBuf(0x0CF029E2, 1, 8, data);
    delay(10);
    
    // 13. Slope Sensor Information 2 (ID Estendido) - (Outro frame para o mesmo ID)
    // ID: 0x0CF029E2, Data[7]=0xAA
    data[7] = 0xAA;
    CAN.sendMsgBuf(0x0CF029E2, 1, 8, data);
    delay(10);

    // Limpa o array de dados para a próxima iteração
    memset(data, 0, sizeof(data));

    // Espera um tempo maior antes de enviar o próximo conjunto de frames
    delay(500); 
}