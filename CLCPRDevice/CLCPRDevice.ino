/*
  Name:		CLCPRDevice.ino
  Created:	3/30/2016 6:50:13 PM
  Author:	Mitchell Baldwin & Joseph Kissling
*/

// Libraries for functions
#include <PacketSerial.h>
#include <EEPROM.h>

#define LEDPIN 13
#define PACKET_SIZE 30
uint8_t inPacket[PACKET_SIZE];
uint8_t outPacket[PACKET_SIZE];

// Software Verson info
byte MajorFVN_Verson = 1;
byte MinorFVN_Verson = 21;

// Software Versone packet
byte FVNPkt[2];

// Hardware Verson info
byte MajorHVN_Verson = 1;
byte MinorHVN_Verson = 5;

// Hardware packet
byte HVNPkt[2];

// EEPROM addresses
int MAJORrev_EEPROM_addr = 0;
int MINORrev_EEPROM_addr = 1;

// CPR Phase Times
uint16_t CPRPT[4];
byte CPRPTData[8];

PacketSerial spUSB;

void setup()
{
  // Set the pin mode
  pinMode(LEDPIN, OUTPUT);

  // Build the Firmware packet
  FVNPkt[0] = MajorFVN_Verson;
  FVNPkt[1] = MinorFVN_Verson;

  // Build the major communication packets
  for (int i = 0; i < PACKET_SIZE; ++i)
  {
    inPacket[i] = 0x00;
    outPacket[i] = 0x00;
  }

  // Turn on the packe communication system
  spUSB.setPacketHandler(&OnUSBPacket);
  spUSB.begin(115200);
}

void loop()
{
  spUSB.update();
  delay(10);
}

// Read and respond to the packets
void OnUSBPacket(const uint8_t* buffer, size_t size)
{
  // Test case for returing the stated of the device
  if (buffer[0] == 0x10)
  {
    ToggleUserLED();
    outPacket[0] = 255;
    SendPacket();
  }

  // send the firmware version
  if (buffer[0] == 0xA3)
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

  // Set hardware verson, writes the values to EEPROM
  if (buffer[0] == 0xA5)
  {
    // MajorHVN_Verson = buffer[1];
    // MinorHVN_Verson = buffer[2];
    EEPROM.write(MAJORrev_EEPROM_addr, MajorHVN_Verson);
    EEPROM.write(MINORrev_EEPROM_addr, MinorHVN_Verson);
  }

  // Set CPR Phase times
  if (buffer[0] == 0x20)
  {
    // CPR Phase times
  }

  // State Flag reader for indicating & controlling the state of the subsystems
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

// Switches toggles the LED, used as an heartbeat to verifiy commmunication
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

// Send the data packet to the computer.
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

// Send the firmware verson packet
void SendFVN()
{
  outPacket[0] = FVNPkt[0];
  outPacket[1] = FVNPkt[1];
  SendPacket();
}

