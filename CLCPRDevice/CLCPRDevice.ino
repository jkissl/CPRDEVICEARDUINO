/*
  Name:		CLCPRDevice.ino
  Created:	3/30/2016 6:50:13 PM
  Author:	Mitchell Baldwin & Joseph Kissling 
  Quick check. Comments comming soon
*/
#include <PacketSerial.h>
#include <EEPROM.h>

#define LEDPIN 13
#define PACKET_SIZE 30
uint8_t inPacket[PACKET_SIZE];
uint8_t outPacket[PACKET_SIZE];

// Software Verson info
byte MajorFVN_Verson = 1;
byte MinorFVN_Verson = 21;
byte FVNPkt[2];

// Hardware Verson info
byte MajorHVN_Verson = 1;
byte MinorHVN_Verson = 5;
byte HVNPkt[2];
int MAJORrev_EEPROM_addr = 0;
int MINORrev_EEPROM_addr = 1;

// CPR Phase Times
uint16_t CPRPT[4];
byte CPRPTData[8];

PacketSerial spUSB;

void setup()
{
  pinMode(LEDPIN, OUTPUT);
  FVNPkt[0] = MajorFVN_Verson;
  FVNPkt[1] = MinorFVN_Verson;

  for (int i = 0; i < PACKET_SIZE; ++i)
  {
    inPacket[i] = 0x00;
    outPacket[i] = 0x00;
  }
  spUSB.setPacketHandler(&OnUSBPacket);
  spUSB.begin(115200);
}

void loop()
{
  spUSB.update();
  delay(10);
}

void OnUSBPacket(const uint8_t* buffer, size_t size)
{
  if (buffer[0] == 0x10)				// Test case
  {
    ToggleUserLED();
    outPacket[0] = 255;
    outPacket[1] = 255;
    outPacket[2] = 255;
    outPacket[3] = 255;
    outPacket[4] = 255;
    outPacket[5] = 255;
    outPacket[6] = 255;
    outPacket[7] = 255;
    SendPacket();


  }
  if (buffer[0] == 0xA3)        //testparm
  {
    ToggleUserLED();
    SendFVN();
  }

  // Read Hardware verson
  if (buffer[0] == 0xA4)
  {
    outPacket[1] = EEPROM.read(MAJORrev_EEPROM_addr);
    outPacket[2] = EEPROM.read(MINORrev_EEPROM_addr);
    SendPacket();
  }

  // Set hardware verson
  if (buffer[0] == 0xA5)
  {
    // MajorHVN_Verson = buffer[1];
    // MinorHVN_Verson = buffer[2];
    EEPROM.write(MAJORrev_EEPROM_addr, MajorHVN_Verson);
    EEPROM.write(MINORrev_EEPROM_addr, MinorHVN_Verson);
  }
if (buffer[0] == 0xA5)
{
  // CPR Phase times
}

  // State Flag reader
  for (int j = 0; j < 4; ++j)
  {
    if (buffer[0] & _BV(j))
    {
      switch (j) {
        case 0 :
          // COMP
          break;
        case 1 :
          // VENT
          break;
        case 2 :
          // SYNC
          break;
        case 3 :
          // MEAS
          break;
        default:
          break;
      }
    }
  }
}

void ToggleUserLED()
{
  if (digitalRead(LEDPIN) == HIGH)
  {
    digitalWrite(LEDPIN, LOW);
  }
  else
  {
    digitalWrite(LEDPIN, HIGH);
  }
}

void SendPacket()
{
  int checksum = 0;
  for (int i = 0; i < PACKET_SIZE - 1; ++i)
  {
    checksum += outPacket[i];
  }
  outPacket[PACKET_SIZE - 1] = checksum;
  spUSB.send(outPacket, PACKET_SIZE);
}

void SendFVN()
{
  outPacket[0] = FVNPkt[0];
  outPacket[1] = FVNPkt[1];
  SendPacket();
}

