//****************************************************
// BMS Master-Slave
// Slave Module
//
// v2, 18.05.2023
// + Arduino Pro Mini 
// + CAN Shield
//
//**************************************************** 

#include <mcp_can.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/*  Arduino pins    CAN Shield pins
         2          INT
        13          SCK
        12          S0
        11          SI
        10          CS
        Vcc         Vcc
        GND         GND
*/
 
#define CAN_Int_Pin 2   // pins for the CAN module
#define CAN_CS_Pin 10

#define Temp_Pin 8      // pin for Temp Sensor
#define Volt_Pin A2     // input pin for voltage
#define Bal_Pin 9       // output pin for balancing resistor

MCP_CAN CAN(CAN_CS_Pin); 

#define MasterID 0x100      // ID Master

#define SlaveID  0x101      // ID Slave -> IT MUST BE MODIFIED FOR EACH SLAVE MODULE!!!

uint8_t ID = (uint8_t)(SlaveID - 0x100);

// Transmitted CAN Frame, from Slave to Master
// {Balancing State, Voltage High Byte, Voltage Low Byte, Temperature, 0x00, 0x00, 0x00, 0x00}
byte TxBuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Received CAN Frame, from Master to Slave
byte RxBuf[8];
long unsigned int RxID;

byte SlaveAlohaMsg[8] = {0x00, 0x00, 0x53, 0x6C, 0x61, 0x76, 0x65, 0x00};    // Slave word

unsigned long int i = 0; 

static uint32_t Current_Time; 
static int Temp_Timeout = 100;      // timeout for reading temperature
static uint32_t Temp_LastTime; 
static int Volt_Timeout = 100;      // timeout for reading voltage
static uint32_t Volt_LastTime; 
static int CAN_Timeout = 1000;      // timeout for transmiting CAN frame
static uint32_t CAN_LastTime; 

int Temp_NoSamples = CAN_Timeout / Temp_Timeout;
int Volt_NoSamples = CAN_Timeout / Volt_Timeout;

int Cell_Bal = 0;   // Balancing State
int Cell_Volt;      // Cell Voltage
int Cell_Temp;      // Cell Temperature

byte SamplesNo = 100;

OneWire oneWire(Temp_Pin);                  // Temp Sensor initialization
DallasTemperature Temp_Sensor(&oneWire);


void setup () 
{
  // Initialize PINs
  pinMode(Volt_Pin, INPUT);
  pinMode(CAN_Int_Pin, INPUT);
  
  pinMode(Bal_Pin, OUTPUT); digitalWrite(Bal_Pin, LOW);

  // Initialize CAN bus at 500 kbps, 8 MHz
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ))
  {
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);

  Current_Time = millis();
  Temp_LastTime = Current_Time; 
  Volt_LastTime = Current_Time;
  CAN_LastTime = Current_Time;
 
}

void loop () 
{
  Current_Time = millis();

  // Received CAN frame from the Master
  // Verify Balancing Status from Master
  if (digitalRead(CAN_Int_Pin) == LOW) {
    CAN.readMsgBuf(&RxID, 8, RxBuf);
    if ( ( RxID == MasterID ) && ( RxBuf[0] == ID ) ) {
      Cell_Bal = RxBuf[1];
    } 
  }

  // Reading balancing state
  TxBuf[0] = Cell_Bal;

  // Reading voltage
  if (Current_Time - Volt_LastTime > Temp_Timeout) { 
    Cell_Volt = (int)(analogRead(Volt_Pin) * 5.0 * 1000.0 / 1023.0);
    TxBuf[1] = (Cell_Volt >> 8 ) & 0xFF;
    TxBuf[2] = Cell_Volt & 0xFF;
    Volt_LastTime = Current_Time;
  }

  // Reading temperature
  if (Current_Time - Temp_LastTime > Temp_Timeout) { 
    Temp_Sensor.requestTemperatures();
    TxBuf[3] = Temp_Sensor.getTempCByIndex(0);
    Temp_LastTime = Current_Time;
  }  

  // Transmission of data
  if (Current_Time - CAN_LastTime > CAN_Timeout) { 
    CAN.sendMsgBuf(SlaveID, 0, 8, TxBuf);
    CAN_LastTime = Current_Time;
  }

  // Activate/Deactivate balancing based on the state from the master
  if ( Cell_Bal == 0x01 ) {
    digitalWrite(Bal_Pin, HIGH);
  } else if ( Cell_Bal == 0x00 ) {
    digitalWrite(Bal_Pin, LOW);
  }
}

 
