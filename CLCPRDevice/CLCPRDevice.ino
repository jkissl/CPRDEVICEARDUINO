/*
  Name:		CLCPRDevice.ino
  Created:	3/30/2016 6:50:13 PM
  Author:	Mitchell Baldwin & Joseph Kissling
*/

// Libraries for functions
#include <PacketSerial.h>
#include <EEPROM.h>

#define Position_Output 12
#define LEDPIN 13
#define Position_Input A0
#define PACKET_SIZE 30

uint8_t inPacket[PACKET_SIZE];
uint8_t outPacket[PACKET_SIZE];

// Create the Packet that contains the position of the cylinder in CM
byte PositionPacket[2];
byte *Pos_MSB = &PositionPacket[1];
byte *Pos_LSB = &PositionPacket[2];

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
const uint8_t MAJORrev_EEPROM_addr = 0;
const uint8_t MINORrev_EEPROM_addr = 1;

// CPR Phase Times
uint16_t CPRPT[4];
byte CPRPTData[8];

PacketSerial spUSB;

void setup()
{ 
  pinMode(LEDPIN, OUTPUT);

  // Set the power pin 
  pinMode(Position_Output, OUTPUT);

  
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
  //Serial.begin(115200);
}

void loop()
{
  spUSB.update();
  Get_Cylinder_Pos();
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

/*
=============================
Get_Cylinder_Pos()
Updates the values of the position of the cylinder in CM
Calculated over the full range of motion of the cylinder 9.5 in or 
24.1 CM. Using measured analog range values of 865 for 0 cm and 
393 for 24.1 cm (9.5)in. 
=============================
*/
void Get_Cylinder_Pos()
{
  float Position; 
  float temp;
  Position = analogRead(Position_Input);
  Position = (Position*(-.05)) + 44.16; // Convert to CM
  *Pos_MSB = floor(Position);
  temp = (Position-floor(Position))*100;
  *Pos_LSB = temp;
}

