/* PeopleCounter contains:
   CMWX1ZZABZ (STM32L082 and SX1276)
   LIS2DW12 accelerometer and BME280 environmental sensor 

   Same basic design as Cricket asset tracker but in a more convenient form factor. Using
   Grasshopper board variant since Cricket does not make all GPIO available to the application program.

   This skt=etch simply shows how to get the data from the sensors and send some of it via LoRaWAN to
   the TTN server.

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction means no serial through the USB.  

   This example code is in the public domain.
*/
#include "STM32L0.h"
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "RTC.h"
#include "STHS34PF80.h"
#include "LIS2DW12.h"
#include "BME280.h"
#include "CayenneLPP.h"

//#define Serial Serial2  // use Serial port on connector if no USB
bool SerialDebug = true;

// Production Cricket 1
const char *appEui = "70B3D57ED000964D";
const char *appKey = "60170539954A1E2C8B519658F0CAFB52";
const char *devEui = "70B3D57ED004F377";

CayenneLPP myLPP(64);

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

TimerMillis LoRaTimer;                // instantiate LoraWAN timer

// Internal MCU and battery voltage monitor definitions
// People Counter pin assignments
#define myLed       10 // blue led 
#define VBAT_en      2
#define USER_BTN     2
#define VBAT_sense  A1

float VDDA, VBAT, VBUS, STM32L0Temp;
uint32_t UID[3] = {0, 0, 0}; 
char buffer[32];


//STHS34PF80 definitions
#define STHS34PF80_1_intPin   4    // bottom IR sensor interrupt pin  
#define STHS34PF80_2_intPin   5    // top IR sensor interrupt pin  
#define IR_select            A0    // LOW for IR sensor 1 (bottom), HIGH for IR sensor 2 (top)

// STHS34PF80 configuration
bool ONESHOT = false;       // continuous (false) or one-shot mode (true)
// Select use of embedded functions (true) or just use as a Amb/Obj temperature sensor; 
// ONESHOT mode not compatible with embedded function use!
bool FUNCTIONS = true;      // temperatures only (false) or presence, motion, Tshock flags (true)?

IRODR irodr = STHS34PF80_ODR_1;           // select 0.25, 0.5, 1, 2, 4, 8, 15 and 30 Hz
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
volatile bool STHS34PF80_1_flag = false, STHS34PF80_2_flag = false;
uint8_t irstatus1 = 0, funcstatus1 = 0, irstatus2 = 0, funcstatus2 = 0;

STHS34PF80 STHS34PF80(&i2c_0); // instantiate STHS34PF80 class


//LIS2DW12 definitions
#define LIS2DW12_intPin1   3    // interrupt1 pin definitions, wake interrupt pin
#define LIS2DW12_intPin2   A4   // interrupt2 pin definitions, sleep interrupt pin

// Specify sensor parameters //
LPMODE   lpMode = LIS2DW12_LP_MODE_1;      // choices are low power modes 1, 2, 3, or 4
MODE     mode   = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
ODR      odr    = LIS2DW12_ODR_12_5_1_6HZ; //  1.6 Hz in lpMode, max is 200 Hz in LpMode
FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
BW_FILT  bw     = LIS2DW12_BW_FILT_ODR2;   // choices are ODR divided by 2, 4, 10, or 20
FIFOMODE fifoMode = BYPASS;                // capture 32 samples of data before wakeup event, about 2 secs at 25 Hz
bool lowNoise = false;                     // low noise or lowest power

float aRes = 0;         // Sensor data scale in mg/LSB
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // 8-bit signed temperature output
uint8_t rawTempCount;   // raw temperature output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // holds accel bias offsets
float stress[3];        // holds results of the self test
uint8_t status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;

// Logic flags to keep track of device states
bool LIS2DW12_wake_flag = false;
bool LIS2DW12_sleep_flag = false;
bool InMotion = false;

LIS2DW12 LIS2DW12(&i2c_0); // instantiate LIS2DW12 class


// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_01, Hosr = H_OSR_01, Tosr = T_OSR_01, Mode = Sleep, IIRFilter = full, SBy = t_1000ms;     // set pressure amd temperature output data rate

int32_t rawPress, rawTemp, rawHumidity, compTemp;   // pressure, humidity, and temperature raw count output for BME280
uint32_t compHumidity, compPress;                   // variables to hold compensated BME280 humidity and pressure values
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280(&i2c_0); // instantiate BME280 class


uint8_t Hour = 12, Minute = 0, Second = 0, Year = 23, Month = 1, Day = 1;
uint8_t year, month, day, hours, minutes, seconds;
uint16_t milliseconds, Millisec;
volatile bool alarmFlag = true;
uint16_t count = 0, tempBytes[4], pressBytes[4];
float Temperature;


void setup()
{
  if(SerialDebug) {
    Serial.begin(115200);
    Serial.println("Serial enabled!");
    delay(5000);
  }
  
  STM32L0.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); }

  LoRaWAN.getDevEui(buffer, 18);
  if(SerialDebug) {Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); }
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(VBAT_en, OUTPUT);
  digitalWrite(VBAT_en, LOW);

  pinMode(VBAT_sense, INPUT);
  analogReadResolution(12);

  pinMode (IR_select, OUTPUT);
  digitalWrite(IR_select, LOW); // select bottom IR sensor by default

  pinMode(STHS34PF80_1_intPin, INPUT);  // define bottom STHS34PF80 data ready interrupt
  pinMode(STHS34PF80_2_intPin, INPUT);  // define top STHS34PF80 data ready interrupt

  pinMode(LIS2DW12_intPin1, INPUT);  // define LIS2DW12 wake and sleep interrupt pins as L082 inputs
  pinMode(LIS2DW12_intPin2, INPUT);
  
  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L0
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(100);

  digitalWrite(IR_select, LOW);  // select bottom IR sensor 
  i2c_0.I2Cscan();               // should detect all I2C devices on the bus
  delay(100);
  digitalWrite(IR_select, HIGH); // select top IR sensor  
  i2c_0.I2Cscan();               // should detect all I2C devices on the bus
  delay(1000);
  

  // Read the bottom STHS34PF80 Chip ID register, this is a good test of communication
  digitalWrite(IR_select, LOW);  // select bottom IR sensor 
  Serial.println("STHS34PF80 IR sensorr...");
  byte STHS34PF80_1_ID = STHS34PF80.getChipID();  // Read CHIP_ID register for STHS34PF80
  if(SerialDebug) {
    Serial.print("STHS34PF80 1: "); Serial.print("I AM "); Serial.print(STHS34PF80_1_ID, HEX); Serial.print(" I should be "); Serial.println(0xD3, HEX);
    Serial.println(" ");
   }
   
  // Read the top STHS34PF80 Chip ID register, this is a good test of communication
  digitalWrite(IR_select, HIGH);  // select top IR sensor 
  Serial.println("STHS34PF80 IR sensor...");
  byte STHS34PF80_2_ID = STHS34PF80.getChipID();  // Read CHIP_ID register for STHS34PF80
  if(SerialDebug) {
    Serial.print("STHS34PF80 2: "); Serial.print("I AM "); Serial.print(STHS34PF80_2_ID, HEX); Serial.print(" I should be "); Serial.println(0xD3, HEX);
    Serial.println(" ");
   }
   
  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  byte LIS2DW12_ChipID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  if(SerialDebug) {
    Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
    Serial.println(" ");
  }

  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte BME280_ChipID = BME280.getChipID();  // Read WHO_AM_I register for BME280
  if(SerialDebug) {
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(BME280_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  }
  
  delay(1000);  
  
  if(LIS2DW12_ChipID == 0x44 && BME280_ChipID == 0x60 && STHS34PF80_1_ID == 0xD3 && STHS34PF80_2_ID == 0xD3) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   if(SerialDebug) {Serial.println("LIS2DW12 and BME280 and both STHS34PF80 are online..."); Serial.println(" "); }

   digitalWrite(IR_select, LOW);  // select bottom IR sensor 
   if(SerialDebug)  Serial.print("Configure IR sensor 1");
   STHS34PF80.reset();     // reset all registers to default, sensor enters power down after reset, so next line is redundant
   STHS34PF80.powerDown(); // power down to configure embedded algorithms or wait for one-shot command
   
   // retreive ObjTemp sensitivity
   ObjSense = (STHS34PF80.readSenseData() / 16) + 2048;
      if(SerialDebug) {Serial.print("Object Sense Data (LSB/oC) = "); Serial.println(ObjSense);}
   
   STHS34PF80.setLowpassFilters(lpf_P, lpf_M, lpf_PM, lpf_Tshock);
   STHS34PF80.config(avgt, avgtmos, gain, FUNCTIONS);

   STHS34PF80.configAlgo(presence_ths, motion_ths, tamb_shock_ths, presence_hyst, motion_hyst, tamb_shock_hyst);
   STHS34PF80.readAlgoConfig(algoConfig); // check that algorithm is configured properly
      if(SerialDebug) {
        Serial.print("Presence threshold = ");   Serial.print(algoConfig[0]);               Serial.print(", should be "); Serial.println(presence_ths);
        Serial.print("Motion threshold = ");     Serial.print(algoConfig[1]);               Serial.print(", should be "); Serial.println(motion_ths);
        Serial.print("Tamb Shock threshold = "); Serial.print(algoConfig[2]);               Serial.print(", should be "); Serial.println(tamb_shock_ths);
        Serial.print("Presence hysteresis = ");  Serial.print(algoConfig[3] >> 8);          Serial.print(", should be "); Serial.println(presence_hyst);
        Serial.print("Motion hysteresis = ");    Serial.print(algoConfig[3] & 0x00FF);      Serial.print(", should be "); Serial.println(motion_hyst);
        Serial.print("TambShock hysteresis = "); Serial.print(algoConfig[4] >> 8);          Serial.print(", should be "); Serial.println(tamb_shock_hyst);
        Serial.print("Algo Config = 0x0");       Serial.print(algoConfig[4] & 0x000F, HEX); Serial.println(", should be 0x08 for pulsed mode");  
        Serial.println(" ");
      }
   STHS34PF80.resetAlgo();
   STHS34PF80.powerUp(irodr); // power up and run continuously at odr
      if(SerialDebug) {
        Serial.println("Starting IR sensor 1 continuous mode!");  
        Serial.println(" ");
      }


   digitalWrite(IR_select, HIGH);  // select top IR sensor 
   if(SerialDebug) Serial.print("Configure IR sensor 2");
   STHS34PF80.reset();     // reset all registers to default, sensor enters power down after reset, so next line is redundant
   STHS34PF80.powerDown(); // power down to configure embedded algorithms or wait for one-shot command
   
   // retreive ObjTemp sensitivity
   ObjSense = (STHS34PF80.readSenseData() / 16) + 2048;
   if(SerialDebug) {Serial.print("Object Sense Data (LSB/oC) = "); Serial.println(ObjSense);}
   
   STHS34PF80.setLowpassFilters(lpf_P, lpf_M, lpf_PM, lpf_Tshock);
   STHS34PF80.config(avgt, avgtmos, gain, FUNCTIONS);

   STHS34PF80.configAlgo(presence_ths, motion_ths, tamb_shock_ths, presence_hyst, motion_hyst, tamb_shock_hyst);
   STHS34PF80.readAlgoConfig(algoConfig); // check that algorithm is configured properly
      if(SerialDebug) {
        Serial.print("Presence threshold = ");   Serial.print(algoConfig[0]);               Serial.print(", should be "); Serial.println(presence_ths);
        Serial.print("Motion threshold = ");     Serial.print(algoConfig[1]);               Serial.print(", should be "); Serial.println(motion_ths);
        Serial.print("Tamb Shock threshold = "); Serial.print(algoConfig[2]);               Serial.print(", should be "); Serial.println(tamb_shock_ths);
        Serial.print("Presence hysteresis = ");  Serial.print(algoConfig[3] >> 8);          Serial.print(", should be "); Serial.println(presence_hyst);
        Serial.print("Motion hysteresis = ");    Serial.print(algoConfig[3] & 0x00FF);      Serial.print(", should be "); Serial.println(motion_hyst);
        Serial.print("TambShock hysteresis = "); Serial.print(algoConfig[4] >> 8);          Serial.print(", should be "); Serial.println(tamb_shock_hyst);
        Serial.print("Algo Config = 0x0");       Serial.print(algoConfig[4] & 0x000F, HEX); Serial.println(", should be 0x08 for pulsed mode");  
        Serial.println(" ");
      }
   STHS34PF80.resetAlgo();
   STHS34PF80.powerUp(irodr); // power up and run continuously at odr
      if(SerialDebug) {
        Serial.println("Starting IR sensor 2 continuous mode!");  
        Serial.println(" ");
      }
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);      

   LIS2DW12.selfTest(stress);                                       // perform sensor self test
   if(SerialDebug) {
    Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
    Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   }
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);                                                     

   aRes = 0.000244f * (1 << fs);                                    // scale resolutions per LSB for the sensor at 14-bit data 

   if(SerialDebug) Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   if(SerialDebug) {
    Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
    Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
    Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");
   }

   LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise);               // Initialize sensor in desired mode for application                     
   LIS2DW12.configureFIFO(fifoMode, 0x1F); // 32 levels of data
   delay(100); // let sensor settle

   //Configure the BME280 sensor
   BME280.reset();                                              // reset BME280 before initilization
   delay(100);

   BME280.init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy);         // Initialize BME280 altimeter
   BME280.forced();                                             // get initial data sample, then go back to sleep
 
   }
  else 
  {
   if(STHS34PF80_1_ID != 0xD3 && SerialDebug) Serial.println(" STHS34PF80 1 not functioning!");
   if(STHS34PF80_2_ID != 0xD3 && SerialDebug) Serial.println(" STHS34PF80 2 not functioning!");
   if(LIS2DW12_ChipID != 0x44 && SerialDebug) Serial.println(" LIS2DW12 not functioning!");
   if(BME280_ChipID != 0x60 && SerialDebug) Serial.println(" BME280 not functioning!");
  }

  // Set the time
  SetDefaultRTC();

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  digitalWrite(VBAT_en, HIGH);
  VBAT = 1.27f * VDDA * ((float) analogRead(VBAT_sense)) / 4095.0f;
  digitalWrite(VBAT_en, LOW);
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  if(SerialDebug) {
    Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
    Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    if(VBUS ==  1)  Serial.println("USB Connected!"); 
    Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
    Serial.println(" ");
  }
  
  // set alarm to update the RTC periodically
  RTC.setAlarmTime(12, 0, 0);
  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute

  RTC.attachInterrupt(alarmMatch);

  // Configuree LoRaWAN connection
  /*
    - Asia       AS923
    - Australia  AU915
    - Europe     EU868
    - India      IN865
    - Korea      KR920
    - US         US915 (64 + 8 channels)
   */
    LoRaWAN.begin(US915);
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(1);
    LoRaWAN.setTxPower(10);
    LoRaWAN.setSubBand(2); // 1 for SEMTECH, 2 for TTN servers

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    // Configure timers
    LoRaTimer.start(callbackLoRaTx, 60000, 600000);   // every 10 minutes   
 
    attachInterrupt(LIS2DW12_intPin1, myinthandler1, RISING);  // attach data ready/wake-up interrupt for INT1 pin output of LIS2DW12
    attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING);  // attach no-motion          interrupt for INT2 pin output of LIS2DW12 
    attachInterrupt(STHS34PF80_1_intPin, myinthandler3, RISING);  // interrupt for IR sensor 1
    attachInterrupt(STHS34PF80_2_intPin, myinthandler4, RISING);  // interrupt for IR sensor 2

    LIS2DW12.getStatus(); // read status of interrupts to clear
    digitalWrite(IR_select, LOW);  // select bottom IR sensor 
    STHS34PF80.getFuncStatus(); // reset DR bit of STATUS to 0 before entering main loop
    digitalWrite(IR_select, HIGH);  // select top IR sensor 
    STHS34PF80.getFuncStatus(); // reset DR bit of STATUS to 0 before entering main loop
 
}  /* end of setup */

/* 
 * Everything in the main loop is based on interrupts, so that 
 * if there has not been an interrupt event the STM32L082 should be in STOP mode
*/
 
void loop()
{
  /* STHS34PF80 sensor 1 interrupt handling*/
  if(STHS34PF80_1_flag)
  {
   STHS34PF80_1_flag = false;    // clear the interrupt flag  

   digitalWrite(IR_select, LOW);  // select bottom IR sensor 
     if(SerialDebug) {Serial.println("Data for IR sensor 1:"); Serial.println(" ");}

   irstatus1 = STHS34PF80.getDataReadyStatus();
   
   if(FUNCTIONS) {
    funcstatus1 = STHS34PF80.getFuncStatus();
    if(funcstatus1 & 0x04 && SerialDebug) Serial.println("Presence detected !");
    if(funcstatus1 & 0x02 && SerialDebug) Serial.println("Motion detected !");
    if(funcstatus1 & 0x01 && SerialDebug) Serial.println("T Shock detected !");
   }
   
   if(irstatus1 & 0x04) {  // when data ready

    Presence = STHS34PF80.readPresence();  
    Motion =   STHS34PF80.readMotion();  
    AmbTemp =  STHS34PF80.readAmbTemp();
    ObjTemp =  STHS34PF80.readObjTemp();
    AmbShock = STHS34PF80.readAmbShock();

     if(SerialDebug) {
      Serial.println("Raw counts");
      Serial.print("STHS34PF80 ObjTemp = ");  Serial.println(ObjTemp);  
      Serial.print("STHS34PF80 AmbTemp = ");  Serial.println(AmbTemp);  
      Serial.print("STHS34PF80 Presence = "); Serial.println(Presence);  
      Serial.print("STHS34PF80 Motion = ");   Serial.println(Motion);  
      Serial.print("STHS34PF80 AmbShock = "); Serial.println(AmbShock);  
      Serial.println(" ");
   
      Serial.print("STHS34PF80 1 Ambient Temperature = "); Serial.print(float(AmbTemp) / 100.0f, 2); Serial.println(" C");
      Serial.print("STHS34PF80 1 Object Temperature = "); Serial.print(float(ObjTemp) / float(ObjSense), 2); Serial.println(" C");
      Serial.println(" ");
     }
   }
  } /* end of STHS34PF80 IR sensor 1 interrupt handling */


    /* STHS34PF80 sensor 2 interrupt handling*/
  if(STHS34PF80_2_flag)
  {
   STHS34PF80_2_flag = false;    // clear the interrupt flag  

   digitalWrite(IR_select, HIGH);  // select top IR sensor 
     if(SerialDebug) {Serial.println("Data for IR sensor 2:"); Serial.println(" ");}

   irstatus2 = STHS34PF80.getDataReadyStatus();
   
   if(FUNCTIONS) {
    funcstatus2 = STHS34PF80.getFuncStatus();
    if(funcstatus2 & 0x04 && SerialDebug) Serial.println("Presence detected !");
    if(funcstatus2 & 0x02 && SerialDebug) Serial.println("Motion detected !");
    if(funcstatus2 & 0x01 && SerialDebug) Serial.println("T Shock detected !");
   }
   
   if(irstatus2 & 0x04) {  // when data ready

    Presence = STHS34PF80.readPresence();  
    Motion =   STHS34PF80.readMotion();  
    AmbTemp =  STHS34PF80.readAmbTemp();
    ObjTemp =  STHS34PF80.readObjTemp();
    AmbShock = STHS34PF80.readAmbShock();

     if(SerialDebug) {
      Serial.println("Raw counts");
      Serial.print("STHS34PF80 ObjTemp = ");  Serial.println(ObjTemp);  
      Serial.print("STHS34PF80 AmbTemp = ");  Serial.println(AmbTemp);  
      Serial.print("STHS34PF80 Presence = "); Serial.println(Presence);  
      Serial.print("STHS34PF80 Motion = ");   Serial.println(Motion);  
      Serial.print("STHS34PF80 AmbShock = "); Serial.println(AmbShock);  
      Serial.println(" ");
   
      Serial.print("STHS34PF80 2 Ambient Temperature = "); Serial.print(float(AmbTemp) / 100.0f, 2); Serial.println(" C");
      Serial.print("STHS34PF80 2 Object Temperature = "); Serial.print(float(ObjTemp) / float(ObjSense), 2); Serial.println(" C");
      Serial.println(" ");
     }
   }
  } /* end of STHS34PF80 IR sensor 2 interrupt handling */

  
  /* LIS2DW12 wake detect*/
  if(LIS2DW12_wake_flag)
  {
   LIS2DW12_wake_flag = false;    // clear the wake flag if wake event

   InMotion = true;               // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is awake! **");

   LIS2DW12.activateNoMotionInterrupt();  
   attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of LIS2DW12 

   digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);  // toggle blue led when motion detected      
  } /* end of LIS2DW12 wake detect */
  
  
  /* LIS2DW12 sleep detect*/ // Not needed for basic wake/GNSS on motion case
  if(LIS2DW12_sleep_flag)
  {
   LIS2DW12_sleep_flag = false;                // clear the sleep flag
   InMotion = false;                           // set motion state latch
   if(SerialDebug) Serial.println("** LIS2DW12 is asleep! **");
      
   detachInterrupt(LIS2DW12_intPin2);          // Detach the LIS2DW12 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L0
   LIS2DW12.deactivateNoMotionInterrupt();     // disable no-motion interrupt to save power 

   digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);  // toggle blue led when no motion detected
  } /* end of LIS2DW12 sleep detect */ 


  /*RTC*/
  if (alarmFlag) { // update serial output  
    alarmFlag = false;

    ax = ay = az = 0.0f;
    if(InMotion) {
      
      LIS2DW12.readAccelData(accelCount); // get 14-bit signed accel data

     // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
      ay = (float)accelCount[1]*aRes - offset[1];   
      az = (float)accelCount[2]*aRes - offset[2]; 
     
      if(SerialDebug) {
        Serial.println(" ");
        Serial.print("ax = ");  Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.println(" ");
      }
    }

    VDDA = STM32L0.getVDDA();
    digitalWrite(VBAT_en, HIGH);
    VBAT = 1.27f * VDDA * ((float) analogRead(VBAT_sense)) / 4095.0f;
    digitalWrite(VBAT_en, LOW);
    if(SerialDebug) {
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
    }

    tempCount = LIS2DW12.readTempData();  // Read the accel chip temperature adc values
    temperature =  ((float) tempCount) + 25.0f; // 8-bit accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        
    }

    // BME280 Data
    BME280.forced();  // get one data sample, then go back to sleep
    
    rawTemp =  BME280.readTemperature();
    compTemp = BME280.compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readPressure();
    compPress = BME280.compensate_P(rawPress);
    pressure = (float) compPress/25600.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    rawHumidity =  BME280.readHumidity();
    compHumidity = BME280.compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH
 
    if(SerialDebug){
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");
    }
    
    // Read RTC
    Serial.println("RTC:");
    RTC.getDateTime(day, month, year, hours, minutes, seconds, milliseconds);

    if(SerialDebug) {
        Serial.print("RTC: ");
        Serial.print(2000 + year);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(day);
        Serial.print(" ");
        if (hours <= 9) {
            Serial.print("0");
        }
        Serial.print(hours);
        Serial.print(":");
        if (minutes <= 9) {
            Serial.print("0");
        }
        Serial.print(minutes);
        Serial.print(":");
        if (seconds <= 9) {
            Serial.print("0");
        }
        Serial.print(seconds);
        Serial.print(".");
        if (milliseconds <= 9) {
            Serial.print("0");
        }
        if (milliseconds <= 99) {
            Serial.print("0");
        }
        Serial.print(milliseconds);       
        Serial.println(" ");
    }
    
    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);
        
    } // end of RTC alarm section
    
     STM32L0.deepsleep();        // Enter STOP mode if possible and wait for an interrupt
        
}  /* end of loop*/


/* Useful functions */

void callbackLoRaTx(void)
{     
    if (!LoRaWAN.joined()) 
    {
        LoRaWAN.joinOTAA(appEui, appKey, devEui);
        delay(1000);
    }
        
    if (!LoRaWAN.busy() && LoRaWAN.joined())
    {
        myLPP.reset();
        myLPP.addTemperature(1, temperature_C);
        myLPP.addRelativeHumidity(2, humidity);
        myLPP.addBarometricPressure(3, pressure);
        myLPP.addAnalogInput(4, VBAT);

        LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
    } 

}


void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}


void myinthandler1()
{
  LIS2DW12_wake_flag = true; 
  STM32L0.wakeup();
}


void myinthandler2()
{
  LIS2DW12_sleep_flag = true;
  STM32L0.wakeup();
}


void myinthandler3()
{
  STHS34PF80_1_flag = true; 
  STM32L0.wakeup();
}


void myinthandler4()
{
  STHS34PF80_2_flag = true; 
  STM32L0.wakeup();
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

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
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
