/*
 * demo: CAN-BUS Shield, receive all frames and print all fields id/type/data
 * to receive frame fastly, a poll in loop() is required.
 *
 * Copyright (C) 2020 Seeed Technology Co.,Ltd.
 */
#include <SPI.h>
#define SERIAL_PORT_MONITOR Serial

#define CAN_2515

// Set SPI CS Pin according to your hardware
const int SPI_CS_PIN = 10;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while (!SERIAL_PORT_MONITOR) {}

    #if MAX_DATA_SIZE > 8
    /*
     * To compatible with MCP2515 API,
     * default mode is CAN_CLASSIC_MODE
     * Now set to CANFD mode.
     */
    CAN.setMode(CAN_NORMAL_MODE);
    #endif

    while (CAN_OK != CAN.begin(CAN_250KBPS, MCP_8MHz)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
        delay(100);
    }
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
}

uint32_t id;
uint16_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte cdata[MAX_DATA_SIZE] = {0};


void loop() {
    // check if data coming
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }

    char prbuf[32 + MAX_DATA_SIZE * 3];
    int i, n;

    unsigned long t = millis();
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) |
           (CAN.isRemoteRequest() << 1);
    /*
     * MCP2515(or this driver) could not handle properly
     * the data carried by remote frame
     */

    //n = sprintf(prbuf, "%04lu.%03d ", t / 1000, int(t % 1000));
    /* Displayed type:
     *
     * 0x00: standard data frame
     * 0x02: extended data frame
     * 0x30: standard remote frame
     * 0x32: extended remote frame
     */
    
     
    //if ( id == (214371092)){
        static const byte type2[] = {0x00, 0x02, 0x30, 0x32};
        n += sprintf(prbuf + n, "RX: [%08lX](%02X) ", (unsigned long)id, type2[type]);
        // n += sprintf(prbuf, "RX: [%08lX](%02X) ", id, type);
    //}
   

    for (i = 0; i < len; i++) {
        n += sprintf(prbuf + n, "%02X ", cdata[i]);
    }
    SERIAL_PORT_MONITOR.println(prbuf);
    CAN.setMode(0); // Controle de msg ACK
}
// END FILE
