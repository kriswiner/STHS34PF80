/* 1/28/23 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer   
 *  
 *  The STHS34PF80 is a three-channel UV light sensor with separate photodiodes sensitive to UVA, UVB, and UVC
 *  light radiation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include <RTC.h>
#include "STHS34PF80.h"

// Ladybug pin assignments
#define myLed       13 // red

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   //  8 characters HH:MM:SS

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

bool SerialDebug = true;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
 
volatile bool alarmFlag = false;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, Temperature;

//STHS34PF80 definitions
#define STHS34PF80_intPin   8    // interrupt pin definitions 

// STHS34PF80 configuration
bool ONESHOT = false;       // continuous (false) or one-shot mode (true)
// Select use of embedded functions (true) or just use as a Amb/Obj temperature sensor; 
// ONESHOT mode not compatible with embedded function use!
bool FUNCTIONS = true;      // temperatures only (false) or presence, motion, Tshock flags (true)?

ODR odr = STHS34PF80_ODR_1;               // select 0.25, 0.5, 1, 2, 4, 8, 15 and 30 Hz
AVGT avgt = STHS34PF80_AVGT_8;            // default STHS34PF80_AVGT_8
AVGTMOS avgtmos = STHS34PF80_AVGTMOS_128; // default STHS34PF80_AVGTMOS_128

// configure detection thresholds  
// These numbers are arbitrary (not temperatures) depend somewhat on odr and
// lpfs, etc but can be adjusted to favor presence or motion or both.
uint16_t presence_ths = 500;    // default 200, maximum 32768
uint16_t motion_ths = 500;      // default 200, maximum 32768
uint16_t tamb_shock_ths = 2000; // default  10, maximum 32768
// configure hysteresis  
uint8_t presence_hyst = 32;     // default is 32, maximum is 256
uint8_t motion_hyst = 32;       // default is 32, maximum is 256
uint8_t tamb_shock_hyst = 2;    // default is  2, maximum is 256

//set low pass filters
uint8_t lpf_P = STHS34PF80_ODR_200, lpf_M = STHS34PF80_ODR_200, lpf_PM = STHS34PF80_ODR_09, lpf_Tshock = STHS34PF80_ODR_50; // default

// set normal or wide gain mode
uint8_t gain = NORMAL_MODE;  // NORMAL_MODE or WIDE_MODE (no embedded functionality)

uint16_t algoConfig[5] = {0, 0, 0, 0, 0};
int16_t AmbTemp = 0, ObjTemp = 0, Presence = 0, Motion = 0, AmbShock = 0, ObjSense = 0;
volatile bool STHS34PF80_DataReady_flag = false;
uint8_t status = 0, funcstatus = 0;

STHS34PF80 STHS34PF80(&i2c_0); // instantiate STHS34PF80 class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  Serial.blockOnOverrun(false);  
  delay(4000);
  Serial.println("Serial enabled!");

  // Test the rgb led, active LOW
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);   // toggle red led on
  delay(1000);                 // wait 1 second

  pinMode(STHS34PF80_intPin, INPUT);  // define STHS34PF80 data ready interrupt

  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect STHS34PF80 at 0x14 and BME280 at 0x77
  delay(1000);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();
  
  // Internal STM32L4 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  Serial.println(" "); 

  // Read the STHS34PF80 Chip ID register, this is a good test of communication
  Serial.println("STHS34PF80 accelerometer...");
  byte STHS34PF80_ID = STHS34PF80.getChipID();  // Read CHIP_ID register for STHS34PF80
  Serial.print("STHS34PF80 "); Serial.print("I AM "); Serial.print(STHS34PF80_ID, HEX); Serial.print(" I should be "); Serial.println(0xD3, HEX);
  Serial.println(" ");
  delay(1000); 

  if(STHS34PF80_ID == 0xD3) // check if STHS34PF80 has acknowledged
   {
   Serial.println("STHS34PF80 is online..."); Serial.println(" ");

   STHS34PF80.reset();     // reset all registers to default, sensor enters power down after reset, so next line is redundant
   STHS34PF80.powerDown(); // power down to configure embedded algorithms or wait for one-shot command
   
   // retreive ObjTemp sensitivity
   ObjSense = (STHS34PF80.readSenseData() / 16) + 2048;
   Serial.print("Object Sense Data (LSB/oC) = "); Serial.println(ObjSense);
   
   STHS34PF80.setLowpassFilters(lpf_P, lpf_M, lpf_PM, lpf_Tshock);
   STHS34PF80.config(avgt, avgtmos, gain, FUNCTIONS);

   STHS34PF80.configAlgo(presence_ths, motion_ths, tamb_shock_ths, presence_hyst, motion_hyst, tamb_shock_hyst);
   STHS34PF80.readAlgoConfig(algoConfig); // check that algorithm is configured properly
   Serial.print("Presence threshold = ");   Serial.print(algoConfig[0]);               Serial.print(", should be "); Serial.println(presence_ths);
   Serial.print("Motion threshold = ");     Serial.print(algoConfig[1]);               Serial.print(", should be "); Serial.println(motion_ths);
   Serial.print("Tamb Shock threshold = "); Serial.print(algoConfig[2]);               Serial.print(", should be "); Serial.println(tamb_shock_ths);
   Serial.print("Presence hysteresis = ");  Serial.print(algoConfig[3] >> 8);          Serial.print(", should be "); Serial.println(presence_hyst);
   Serial.print("Motion hysteresis = ");    Serial.print(algoConfig[3] & 0x00FF);      Serial.print(", should be "); Serial.println(motion_hyst);
   Serial.print("TambShock hysteresis = "); Serial.print(algoConfig[4] >> 8);          Serial.print(", should be "); Serial.println(tamb_shock_hyst);
   Serial.print("Algo Config = 0x0");       Serial.print(algoConfig[4] & 0x000F, HEX); Serial.println(", should be 0x08 for pulsed mode");  
   Serial.println(" ");
   STHS34PF80.resetAlgo();
  
   if(!ONESHOT) {
        STHS34PF80.powerUp(odr); // power up and run continuously at odr
        Serial.println("Starting continuous mode!");
   }
   
  }
  else 
  {
   if(STHS34PF80_ID != 0xD3) Serial.println(" STHS34PF80 not functioning!");
   while(1){}; // wait here forever for a power cycle
  }
  
  /* Set the RTC time */
  SetDefaultRTC();
  
  // set alarm to update the RTC periodically
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(STHS34PF80_intPin, myinthandler, RISING);  // attach data ready INT pin output of STHS34PF80

  STHS34PF80.getFuncStatus(); // reset DR bit of STATUS to 0 before entering main loop
  
}/* end of setup */


void loop()
{
  /* STHS34PF80 data ready detect*/
  if(STHS34PF80_DataReady_flag)
  {
   STHS34PF80_DataReady_flag = false;    // clear the data ready flag  

   status = STHS34PF80.getDataReadyStatus();
//   Serial.print("status = 0x"); Serial.println(status, HEX);
   
   if(FUNCTIONS) {
    funcstatus = STHS34PF80.getFuncStatus();
//    Serial.print("funcstatus = 0x"); Serial.println(funcstatus, HEX);
    if(funcstatus & 0x04) Serial.println("Presence detected !");
    if(funcstatus & 0x02) Serial.println("Motion detected !");
    if(funcstatus & 0x01) Serial.println("T Shock detected !");
   }
   
   if(status & 0x04) {  // when data ready

    Presence = STHS34PF80.readPresence();  
    Motion =   STHS34PF80.readMotion();  
    AmbTemp =  STHS34PF80.readAmbTemp();
    ObjTemp =  STHS34PF80.readObjTemp();
    AmbShock = STHS34PF80.readAmbShock();

   Serial.println("Raw counts");
   Serial.print("STHS34PF80 ObjTemp = ");  Serial.println(ObjTemp);  
   Serial.print("STHS34PF80 AmbTemp = ");  Serial.println(AmbTemp);  
   Serial.print("STHS34PF80 Presence = "); Serial.println(Presence);  
   Serial.print("STHS34PF80 Motion = ");   Serial.println(Motion);  
   Serial.print("STHS34PF80 AmbShock = ");  Serial.println(AmbShock);  
   Serial.println(" ");
   
   Serial.print("STHS34PF80 Ambient Temperature = "); Serial.print(float(AmbTemp) / 100.0f, 2); Serial.println(" C");
   Serial.print("STHS34PF80 Object Temperature = "); Serial.print(float(ObjTemp) / float(ObjSense), 2); Serial.println(" C");
   Serial.println(" ");
   }

  } /* end of STHS34PF80 data ready interrupt handling */

 
  /*RTC*/
  if (alarmFlag) { // update RTC output at the alarm
      alarmFlag = false;

    if(ONESHOT) STHS34PF80.oneShot();  // take one temperature measurement (do one conversion), interrupt when data ready
           
    VDDA = STM32.getVREF();
    Temperature = STM32.getTemperature();
    if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
      Serial.println(" ");
    }

  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
  
    digitalWrite(myLed, HIGH); delay(1);  digitalWrite(myLed, LOW); // toggle red led on
 } // end of RTC alarm section
    
//    STM32.stop();       // Enter STOP mode and wait for an interrupt
    STM32.sleep();        // Enter SLEEP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */
void myinthandler()
{
  STHS34PF80_DataReady_flag = true; 
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()                                                                                 // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
