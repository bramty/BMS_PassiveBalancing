#define USE_TIMER_1     true

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1, Timer2
#else          
  #define USE_TIMER_3     true
  #warning Using Timer1, Timer3
#endif

#include "TimerInterrupt.h"

int Cell_Bal[4];                // array for balancing states for all cells
int Cell_Volt[4];               // array for cells voltages
int Cell_Temp[4];               // array for cells temperatures

int j;
float y;

// long T_serial = 1000;
#define TIMER1_INTERVAL_MS    1000
long prev;

void TimerHandler(void)
{
  sendInfoSerial(Cell_Bal, Cell_Volt, Cell_Temp);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  for(int i=0; i<4; i++){
    Cell_Bal[i] = 0;
    Cell_Volt[i] = 0;
    Cell_Temp[i] = 0;
  }

  ITimer1.init();

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler)) {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  } else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  // prev = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  int pot = analogRead(A0);
  y = map(pot, 0, 1023, 0, 10);

  Cell_Bal[0] = 0;
  Cell_Volt[0] = 3450;
  Cell_Temp[0] = 28;

  Cell_Bal[1] = 0;
  Cell_Volt[1] = 3450;
  Cell_Temp[1] = 28;

  Cell_Bal[2] = 1;
  Cell_Volt[2] = 3250;
  Cell_Temp[2] = 27;

  Cell_Bal[3] = 0;
  Cell_Volt[3] = 3450;
  Cell_Temp[3] = 28;

  // for(int i=0; i<4; i++){
  //   Cell_Bal[i] = (int) (i+1)*0.25*y;
  //   Cell_Volt[i] = (int) (i+1)*0.4*y;
  //   Cell_Temp[i] = (int) (i+1)*0.75*y;
  // }

  // if (millis() - prev > T_serial){
  //   sendInfoSerial(Cell_Bal, Cell_Volt, Cell_Temp);
  // }
  
  //delay(1000);
}

void sendInfoSerial(int balState[4], int voltages[4], int temp[4]){

  Serial.println("BMS");

  Serial.print("Bal:"); 
  for (j=0; j<=3; j++) 
  {
    Serial.print(balState[j]);
    if (j != 3){
      Serial.print(",");
    }
  }
  Serial.println();
    
  Serial.print("Vol:"); 
  for (j=0; j<=3; j++) 
  {
    Serial.print(voltages[j]);
    if (j != 3){
      Serial.print(",");
    }
  }
  Serial.println();    

  Serial.print("Tmp:"); 
  for (j=0; j<=3; j++) 
  {
    Serial.print(temp[j]);
    if (j != 3){
      Serial.print(",");
    }
  }
  Serial.println();    
}