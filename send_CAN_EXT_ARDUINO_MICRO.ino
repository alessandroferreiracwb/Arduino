// demo: CAN-BUS Shield, send data
// Compativel co Arduino Micro
// CAN.MCP_CAN::sendMsgBuf(0x1FEE70A0, 1,  8, stmp); (1= Extended ) (0 = Standart)
//-------------------------------------/\-------------------------------
//--------------------------------------|-------------------------------
#include <SPI.h>

#define CAN_2515

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;
//const int CAN_INT_PIN = 2;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif
 
                           

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    

    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    CAN.setMode(0); // Controle de msg ACK

    //configure pin 2 as an input and enable the internal pull-up resistor
    pinMode(9, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
}


//B1 = stmp0[0];
unsigned char stmp0[8] = { 0x00, 0x11,  0x11,  0x11, 0x11,  0x11,  0x11,  0x11};
unsigned char stmp1[8] = { 0x01, 0x12,  0x12,  0x12, 0x12,  0x12,  0x12,  0x12}; //Temperatura 2
unsigned char stmp2[8] = { 0x02, 0x13,  0x13,  0x13, 0x13,  0x13,  0x13,  0x13}; //Rotação do Motor
unsigned char stmp3[8] = { 0x03, 0x14,  0x14,  0x14, 0xAF,  0xFF,  0xFF,  0xFF}; //LEBRE/LESMA
unsigned char stmp4[8] = { 0x04, 0x15,  0x31,  0x20, 0x20,  0x20,  0x20,  0x20};
unsigned char stmp5[8] = { 0x05, 0x16,  0x36,  0x30, 0xAF,  0x5,  0x6,  0x7};
unsigned char stmp6[8] = { 0x06, 0x17,  0x37,  0x30, 0xAF,  0x5,  0x6,  0x7};
unsigned char stmp7[8] = { 0x07, 0x18,  0x38,  0x30, 0xAF,  0x5,  0x6,  0x7};
unsigned char stmp8[8] = { 0x08, 0x19,  0x38,  0x30, 0xAF,  0x5,  0x6,  0x7};
unsigned char stmp9[8] = { 0x09, 0x20,  0x38,  0x30, 0xAF,  0x5,  0x6,  0x7};

void loop() {
    //read the pushbutton value into a variable
    int Bt1 = digitalRead(9);
    int Bt2 = digitalRead(8);
   

    //SERIAL_PORT_MONITOR.println(Bt2);
    // send data:  id = 0x70, standard frame, data len = 8, stmp: data buf
    CAN.MCP_CAN::sendMsgBuf(0x0CC00B14, 1, 8, stmp0);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC10B14, 1, 8, stmp1);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC20B14, 1, 8, stmp2);
    delay(10);    
    CAN.MCP_CAN::sendMsgBuf(0x0CC30B14, 1, 8, stmp3);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC40B14, 1, 8, stmp4);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC50B14, 1, 8, stmp5);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC60B14, 1, 8, stmp6);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC70B14, 1, 8, stmp7);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC80B14, 1, 8, stmp8);
    delay(10);
    CAN.MCP_CAN::sendMsgBuf(0x0CC90B14, 1, 8, stmp9);
    delay(10);
    
    
    
    delay(100);                       // send data once per second
    CAN.setMode(0); // Controle de msg ACK
}

/*********************************************************************************************************
    END FILE
*********************************************************************************************************/
