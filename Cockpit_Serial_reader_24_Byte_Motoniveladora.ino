//#include <Arduino_BuiltIn.h>
#include <SPI.h>
#include <HID.h>
#include <Joystick.h>

#define COMMUNICATION_TIMEOUT 1000

typedef struct cockpitNevil
{
  /*byte 0: init of frame, default value is 0xF0*/

  /*byte 1*/
  uint8_t specialMessage;
  /*byte 2*/
  uint8_t addressCode;

  /*byte 3*/ /*JOYS ESQUERDO (Y)*/
  bool leftJoystickFowardFlag;
  bool leftJoystickBackwardFlag;
  uint8_t leftJoystickYPoint;

  /*byte 4*/ /*JOYS ESQUERDO (X)*/
  bool leftJoystickRightFlag;
  bool leftJoystickLeftFlag;
  uint8_t leftJoystickXPoint;

  /*byte 5*/ /*JOYS ESQUERDO ANALOGICO (1)*/
  bool leftAnalogic1UpFlag;
  bool leftAnalogic1DwFlag;
  uint8_t leftAnalogic1Point;

  /*byte 6*/ /*JOYS ESQUERDO ANALOGICO (2)*/
  bool leftAnalogic2UpFlag;
  bool leftAnalogic2DwFlag;
  uint8_t leftAnalogic2Point;

  /*byte 7*/ /*JOYS ESQUERDO ANALOGICO (3)*/
  bool leftAnalogic3UpFlag;
  bool leftAnalogic3DwFlag;
  uint8_t leftAnalogic3Point;
/*------------------------------------------*/
  /*byte 8*/ /*JOYS DIREITO (Y)*/
  bool rightJoystickFowardFlag;
  bool rightJoystickBackwardFlag;
  uint8_t rightJoystickYPoint;

  /*byte 9*/ /*JOYS DIREITO (X)*/
  bool rightJoystickRightFlag;
  bool rightJoystickLeftFlag;
  uint8_t rightJoystickXPoint;

  /*byte 10*/ /*JOYS DIREITO ANALOGICO (1)*/
  bool rightAnalogic1UpFlag;
  bool rightAnalogic1DwFlag;
  uint8_t rightAnalogic1Point;

  /*byte 11*/ /*JOYS DIREITO ANALOGICO (2)*/
  bool rightAnalogic2UpFlag;
  bool rightAnalogic2DwFlag;
  uint8_t rightAnalogic2Point;

  /*byte 12*/ /*JOYS DIREITO ANALOGICO (3)*/
  bool rightAnalogic3UpFlag;
  bool rightAnalogic3DwFlag;
  uint8_t rightAnalogic3Point;

  /*byte 13*/ /*PEDAL ESQUERDO*/
  bool leftPedalFowardFlag;
  bool leftPedalBackwardFlag;
  uint8_t leftPedalYPoint;

  /*byte 14*/
  bool rightPedalFowardFlag;
  bool rightPedalBackwardFlag;
  uint8_t rightPedalYPoint;

  /*byte 15*/
  bool AddPedalFowardFlag; 
  bool AddPedalBackwardFlag;
  uint8_t AddPedalYPoint;

  /*byte 16 - 17*/
  uint8_t rightJoystickButtons[6];
  uint8_t leftJoystickButtons[6];

  /*byte 18 - 20*/
  uint8_t keypadButtonStatus[23];

  /*byte 21: end of frame, default value is 0xE7*/

  /*byte 22 and byte 23: CRC16 Modbus*/
} cockpitNevil_t;

void cockpitNevilPaser(uint8_t *data, cockpitNevil_t *_cockpit)
{
  _cockpit->specialMessage = data[1]; /* Cod. Endereço1 */

  _cockpit->addressCode = data[2];    /* Cod. Endereço2 */
/****************************Joystick Esquerdo******************************/
  _cockpit->leftJoystickFowardFlag    =  data[3] & 0b00000001;
  _cockpit->leftJoystickBackwardFlag  = (data[3] & 0b00000010) >> 1;
  _cockpit->leftJoystickYPoint        = (data[3] & 0b01111100) >> 2;

  _cockpit->leftJoystickRightFlag     =  data[4] & 0b00000001;
  _cockpit->leftJoystickLeftFlag      = (data[4] & 0b00000010) >> 1;
  _cockpit->leftJoystickXPoint        = (data[4] & 0b01111100) >> 2;

  _cockpit->leftAnalogic1UpFlag       =  data[5] & 0b00000001;
  _cockpit->leftAnalogic1DwFlag       = (data[5] & 0b00000010) >> 1;
  _cockpit->leftAnalogic1Point        = (data[5] & 0b01111100) >> 2;

  _cockpit->leftAnalogic2UpFlag       =  data[6] & 0b00000001;
  _cockpit->leftAnalogic2DwFlag       = (data[6] & 0b00000010) >> 1;
  _cockpit->leftAnalogic2Point        = (data[6] & 0b01111100) >> 2;

  _cockpit->leftAnalogic3UpFlag       =  data[7] & 0b00000001;
  _cockpit->leftAnalogic3DwFlag       = (data[7] & 0b00000010) >> 1;
  _cockpit->leftAnalogic3Point        = (data[7] & 0b01111100) >> 2;
  
/****************************Joystick Direito**************************/
  _cockpit->rightJoystickFowardFlag   =  data[8] & 0b00000001;
  _cockpit->rightJoystickBackwardFlag = (data[8] & 0b00000010) >> 1;
  _cockpit->rightJoystickYPoint       = (data[8] & 0b01111100) >> 2;

  _cockpit->rightJoystickRightFlag    =  data[9] & 0b00000001;
  _cockpit->rightJoystickLeftFlag     = (data[9] & 0b00000010) >> 1;
  _cockpit->rightJoystickXPoint       = (data[9] & 0b01111100) >> 2;

  _cockpit->rightAnalogic1UpFlag      =  data[10] & 0b00000001;
  _cockpit->rightAnalogic1DwFlag      = (data[10] & 0b00000010) >> 1;
  _cockpit->rightAnalogic1Point       = (data[10] & 0b01111100) >> 2;

  _cockpit->rightAnalogic2UpFlag      =  data[11] & 0b00000001;
  _cockpit->rightAnalogic2DwFlag      = (data[11] & 0b00000010) >> 1;
  _cockpit->rightAnalogic2Point       = (data[11] & 0b01111100) >> 2;

  _cockpit->rightAnalogic3UpFlag      =  data[12] & 0b00000001;
  _cockpit->rightAnalogic3DwFlag      = (data[12] & 0b00000010) >> 1;
  _cockpit->rightAnalogic3Point       = (data[12] & 0b01111100) >> 2;

  /*****************************Pedais*********************************/
  _cockpit->leftPedalFowardFlag       =  data[13] & 0b00000001;
  _cockpit->leftPedalBackwardFlag     = (data[13] & 0b00000010) >> 1;
  _cockpit->leftPedalYPoint           = (data[13] & 0b01111100) >> 2;

  _cockpit->rightPedalFowardFlag      =  data[14] & 0b00000001;
  _cockpit->rightPedalBackwardFlag    = (data[14] & 0b00000010) >> 1;
  _cockpit->rightPedalYPoint          = (data[14] & 0b01111100) >> 2;

  _cockpit->AddPedalFowardFlag        =  data[15] & 0b00000001;
  _cockpit->AddPedalBackwardFlag      = (data[15] & 0b00000010) >> 1;
  _cockpit->AddPedalYPoint            = (data[15] & 0b01111100) >> 2;  

  /*------------------------ 12 Botões Joystick------------------------*/
  _cockpit->rightJoystickButtons[0]   = (data[16] & 0b00000001);
  _cockpit->rightJoystickButtons[1]   = (data[16] & 0b00000010) >> 1;
  _cockpit->rightJoystickButtons[2]   = (data[16] & 0b00000100) >> 2;
  _cockpit->rightJoystickButtons[3]   = (data[16] & 0b00001000) >> 3;
  _cockpit->rightJoystickButtons[4]   = (data[16] & 0b00010000) >> 4;
  _cockpit->rightJoystickButtons[5]   = (data[16] & 0b00100000) >> 5;

  _cockpit->leftJoystickButtons[0]    = (data[17] & 0b00000001); 
  _cockpit->leftJoystickButtons[1]    = (data[17] & 0b00000010) >> 1;
  _cockpit->leftJoystickButtons[2]    = (data[17] & 0b00000100) >> 2;
  _cockpit->leftJoystickButtons[3]    = (data[17] & 0b00001000) >> 3;
  _cockpit->leftJoystickButtons[4]    = (data[17] & 0b00010000) >> 4;
  _cockpit->leftJoystickButtons[5]    = (data[17] & 0b00100000) >> 5;
  
  /**************************** 23 Botoes Teclado ************************/
  _cockpit->keypadButtonStatus[0]     = (data[18] & 0b00000001);
  _cockpit->keypadButtonStatus[1]     = (data[18] & 0b00000010) >> 1;
  _cockpit->keypadButtonStatus[2]     = (data[18] & 0b00000100) >> 2;
  _cockpit->keypadButtonStatus[3]     = (data[18] & 0b00001000) >> 3;
  _cockpit->keypadButtonStatus[4]     = (data[18] & 0b00010000) >> 4;
  _cockpit->keypadButtonStatus[5]     = (data[18] & 0b00100000) >> 5;
  _cockpit->keypadButtonStatus[6]     = (data[18] & 0b01000000) >> 6;
  _cockpit->keypadButtonStatus[7]     = (data[18] & 0b10000000) >> 7;

  _cockpit->keypadButtonStatus[8]     = (data[19] & 0b00000001);
  _cockpit->keypadButtonStatus[9]     = (data[19] & 0b00000010) >> 1;
  _cockpit->keypadButtonStatus[10]    = (data[19] & 0b00000100) >> 2;
  _cockpit->keypadButtonStatus[11]    = (data[19] & 0b00001000) >> 3;
  _cockpit->keypadButtonStatus[12]    = (data[19] & 0b00010000) >> 4;
  _cockpit->keypadButtonStatus[13]    = (data[19] & 0b00100000) >> 5;
  _cockpit->keypadButtonStatus[14]    = (data[19] & 0b01000000) >> 6;
  _cockpit->keypadButtonStatus[15]    = (data[19] & 0b10000000) >> 7;

  _cockpit->keypadButtonStatus[16]    = (data[20] & 0b00000001);
  _cockpit->keypadButtonStatus[17]    = (data[20] & 0b00000010) >> 1;
  _cockpit->keypadButtonStatus[18]    = (data[20] & 0b00000100) >> 2;
  _cockpit->keypadButtonStatus[19]    = (data[20] & 0b00001000) >> 3;
  _cockpit->keypadButtonStatus[20]    = (data[20] & 0b00010000) >> 4;
  _cockpit->keypadButtonStatus[21]    = (data[20] & 0b00100000) >> 5;
  _cockpit->keypadButtonStatus[22]    = (data[20] & 0b01000000) >> 6;
  _cockpit->keypadButtonStatus[23]    = (data[20] & 0b10000000) >> 7;
}

uint16_t crc16(unsigned char* data_p, int len)
{
  uint16_t _crc = 0xFFFF;
  char bit2 = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    _crc ^= data_p[i];

    for (bit2 = 0; bit2 < 8; bit2++)
    {
      if (_crc & 0x0001)
      {
        _crc >>= 1;
        _crc ^= 0xA001;
      }
      else
      {
        _crc >>= 1;
      }
    }
  }

  return _crc;
}

Joystick_ joystickUSB(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
        /*BOTÕES*/    12, 0,                  // Button Count, but no Hat Switch Count
  /*JOY ESQUERDO*/    true, true,             // X, Y           - 
    /*ANALOGICOS*/    true, true, true,       // Z, Rx, Ry      - 
        /*PEDAIS*/    true, true, true,       // No rudder or throttle
                      false, false, false);   // No accelerator, brake, or steering

Joystick_ keypadUSB(JOYSTICK_DEFAULT_REPORT_ID + 1, JOYSTICK_TYPE_GAMEPAD,
        /*BOTÕES*/  24, 0,                    // Button Count, but no Hat Switch Count
   /*JOY DIREITO*/  true, true,               // X, but no Y and Z Axis
    /*ANALOGICOS*/  true, true, true,         // No Rx, Ry, or Rz
                    false, false, false,       // No rudder or throttle
                    false, false, false);     // No accelerator, brake, or steering

cockpitNevil_t cockpit;

uint8_t bufferSerial[24];

bool timeouted_communication = false;

unsigned long lastSerialReceiveTime = 0;

void setup()
{
  SerialUSB.begin(115200);
  Serial1.begin(9600);
  
  // Configuração dos ranges para joystickUSB
  joystickUSB.setXAxisRange(-1000, 1000); // Joy Esquerdo (X)
  joystickUSB.setYAxisRange(-1000, 1000); // Joy Esquerdo (Y)
  joystickUSB.setZAxisRange(-1000, 1000); // Joy Esquerdo A1
  joystickUSB.setRxAxisRange(-1000, 1000); // Joy Esquerdo A2
  joystickUSB.setRyAxisRange(-1000, 1000); // Joy Esquerdo A3
  
  // Pedais: Ranges de 0 a 1000 para serem unipolares (como estavam funcionando)
  joystickUSB.setRzAxisRange(0, 1000);    // Pedal Esquerdo (Payload[13]) agora em RzAxis
  joystickUSB.setRudderRange(0, 1000);    // Pedal Direito (Payload[14]) agora em Rudder
  joystickUSB.setThrottleRange(0, 1000);  // Pedal Adicional (Payload[15]) agora em Throttle
  joystickUSB.begin(false);

  // Configuração dos ranges para keypadUSB
  keypadUSB.setXAxisRange(-1000, 1000); // Joy Direito (X)
  keypadUSB.setYAxisRange(-1000, 1000); // Joy Direito (Y)
  keypadUSB.setZAxisRange(-1000, 1000); // Joy Direito A1
  keypadUSB.setRxAxisRange(-1000, 1000); // Joy Direito A2
  keypadUSB.setRyAxisRange(-1000, 1000); // Joy Direito A3
  keypadUSB.begin(false);

  Serial1.setTimeout(4);
}

void loop()
{
  if (Serial1.readBytes(bufferSerial, 24) == 24)
  {
    if ((bufferSerial[0] == 0xF1) && (bufferSerial[21] == 0xE7))
    {
      uint16_t crcReceived = (uint16_t)(((uint16_t)bufferSerial[22] << 8) | (uint16_t)bufferSerial[23]);
      uint16_t crcCalculated = crc16(bufferSerial, 22);

      if (crcCalculated == crcReceived)
      {
        cockpitNevilPaser(bufferSerial, &cockpit);

        /* right joystick USB */
        joystickUSB.setXAxis(cockpit.rightJoystickXPoint * (cockpit.rightJoystickLeftFlag ? -50 : 50));
        joystickUSB.setYAxis(cockpit.rightJoystickYPoint * (cockpit.rightJoystickBackwardFlag ? -50 : 50));
        joystickUSB.setZAxis(cockpit.rightAnalogic1Point * (cockpit.rightAnalogic1UpFlag ? -50 : 50));
        joystickUSB.setRxAxis(cockpit.rightAnalogic2Point * (cockpit.rightAnalogic2UpFlag ? -50 : 50));
        joystickUSB.setRyAxis(cockpit.rightAnalogic3Point * (cockpit.rightAnalogic3UpFlag ? -50 : 50));

        
               
        /* Pedais joystick USB */
        joystickUSB.setRzAxis(cockpit.rightPedalYPoint * (cockpit.rightPedalBackwardFlag ? -50 : 50));   /* right pedal USB */        
        joystickUSB.setRudder(cockpit.leftPedalYPoint * (cockpit.leftPedalBackwardFlag ? -50 : 50));    /* left pedal USB */ 
        joystickUSB.setThrottle(cockpit.AddPedalYPoint * (cockpit.AddPedalBackwardFlag ? -50 : 50));    /* add pedal USB */ 

        /* left keypad USB */
        keypadUSB.setXAxis(cockpit.leftJoystickXPoint * (cockpit.leftJoystickLeftFlag ? -50 : 50));
        keypadUSB.setYAxis(cockpit.leftJoystickYPoint * (cockpit.leftJoystickBackwardFlag ? -50 : 50));        
        keypadUSB.setZAxis(cockpit.leftAnalogic1Point * (cockpit.leftAnalogic1UpFlag ? -50 : 50));
        keypadUSB.setRxAxis(cockpit.leftAnalogic2Point * (cockpit.leftAnalogic2UpFlag ? -50 : 50));
        keypadUSB.setRyAxis(cockpit.leftAnalogic3Point * (cockpit.leftAnalogic3UpFlag ? -50 : 50));   
        
        /* 12 botões joystick USB */
        joystickUSB.setButton(0, cockpit.rightJoystickButtons[0]);
        joystickUSB.setButton(1, cockpit.rightJoystickButtons[1]);
        joystickUSB.setButton(2, cockpit.rightJoystickButtons[2]);
        joystickUSB.setButton(3, cockpit.rightJoystickButtons[3]);
        joystickUSB.setButton(4, cockpit.rightJoystickButtons[4]);
        joystickUSB.setButton(5, cockpit.rightJoystickButtons[5]);        
        
        joystickUSB.setButton(6, cockpit.leftJoystickButtons[0]);
        joystickUSB.setButton(7, cockpit.leftJoystickButtons[1]);
        joystickUSB.setButton(8, cockpit.leftJoystickButtons[2]);
        joystickUSB.setButton(9, cockpit.leftJoystickButtons[3]);
        joystickUSB.setButton(10, cockpit.leftJoystickButtons[4]);
        joystickUSB.setButton(11, cockpit.leftJoystickButtons[5]);

        /* keyboard USB */
        keypadUSB.setButton(0, cockpit.keypadButtonStatus[0]);
        keypadUSB.setButton(1, cockpit.keypadButtonStatus[1]);
        keypadUSB.setButton(2, cockpit.keypadButtonStatus[2]);
        keypadUSB.setButton(3, cockpit.keypadButtonStatus[3]);
        keypadUSB.setButton(4, cockpit.keypadButtonStatus[4]);
        keypadUSB.setButton(5, cockpit.keypadButtonStatus[5]);
        keypadUSB.setButton(6, cockpit.keypadButtonStatus[6]);
        keypadUSB.setButton(7, cockpit.keypadButtonStatus[7]);
        keypadUSB.setButton(8, cockpit.keypadButtonStatus[8]);
        keypadUSB.setButton(9, cockpit.keypadButtonStatus[9]);
        keypadUSB.setButton(10, cockpit.keypadButtonStatus[10]);
        keypadUSB.setButton(11, cockpit.keypadButtonStatus[11]);
        keypadUSB.setButton(12, cockpit.keypadButtonStatus[12]);
        keypadUSB.setButton(13, cockpit.keypadButtonStatus[13]);
        keypadUSB.setButton(14, cockpit.keypadButtonStatus[14]);
        keypadUSB.setButton(15, cockpit.keypadButtonStatus[15]);
        keypadUSB.setButton(16, cockpit.keypadButtonStatus[16]);
        keypadUSB.setButton(17, cockpit.keypadButtonStatus[17]);
        keypadUSB.setButton(18, cockpit.keypadButtonStatus[18]);
        keypadUSB.setButton(19, cockpit.keypadButtonStatus[19]);
        keypadUSB.setButton(20, cockpit.keypadButtonStatus[20]);
        keypadUSB.setButton(21, cockpit.keypadButtonStatus[21]);
        keypadUSB.setButton(22, cockpit.keypadButtonStatus[22]);
        keypadUSB.setButton(23, cockpit.keypadButtonStatus[23]);
        

        /* send USB states*/
        joystickUSB.sendState();
        keypadUSB.sendState();

        lastSerialReceiveTime = millis();
        timeouted_communication = false;

        for(uint8_t i = 0; i < 24; i++)
        {
          SerialUSB.print(bufferSerial[i],BIN);

          SerialUSB.print(' ');
        }

        SerialUSB.println();
          
      }
    }
  }
  else if ((millis() - lastSerialReceiveTime > COMMUNICATION_TIMEOUT) && !timeouted_communication)
  {
    timeouted_communication = true;
    SerialUSB.println('T');

    /* right joystick USB */
    joystickUSB.setXAxis(0);
    joystickUSB.setYAxis(0);

    for (uint8_t i = 0; i < 6; i++)
      joystickUSB.setButton(i, 0);

    /* left joystick USB */
    joystickUSB.setRxAxis(0);
    joystickUSB.setRyAxis(0);

    for (uint8_t i = 0; i < 13; i++)
      joystickUSB.setButton(i, 0);

    /* right pedal USB */
    joystickUSB.setZAxis(0);

    /* left pedal USB */
    joystickUSB.setRzAxis(0);

    /* keyboard USB */
    for (uint8_t buttonIndex = 0; buttonIndex < 23; buttonIndex++)
    {
      keypadUSB.setButton(buttonIndex, 0);
    }

    /* send USB states*/
    joystickUSB.sendState();
    keypadUSB.sendState();
  }
}
