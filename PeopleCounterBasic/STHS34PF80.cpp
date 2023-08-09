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
#include "STHS34PF80.h"
#include "I2CDev.h"

STHS34PF80::STHS34PF80(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


  uint8_t STHS34PF80::getChipID()
  {
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_WHO_AM_I);
  return temp;
  }


  void STHS34PF80::reset()
  {
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp | 0x80); // set bit 7 to force device reset, wait 2.5 ms for reboot
  delay(3);
  }
  

  void STHS34PF80::powerDown() 
  {
  _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_STATUS); // Reset STATUS register to 0
  
  uint8_t stat = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_STATUS);
  while( (stat & 0x04) )
  {
    stat = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_STATUS);
    delay(1);
  } // wait for STATUS DR bit --> 0
    
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL1);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS,  STHS34PF80_CTRL1, temp & ~(0x07) );  //  set bits 0 - 3 to 0 to power down
  _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_STATUS); // reset DR bit of STATUS to 0
  }


  void STHS34PF80::powerUp(uint8_t odr) 
  {
   uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL1);  
   _i2c_bus->writeByte(STHS34PF80_ADDRESS,  STHS34PF80_CTRL1, temp | 0x40 | odr );  //  set bdu = 1 (bit 4 == 1) and odr
  }

  
  void STHS34PF80::setLowpassFilters(uint8_t lpf_P, uint8_t lpf_M, uint8_t lpf_PM, uint8_t lpf_Tshock)
 {
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_LPF1, lpf_PM << 3 | lpf_M);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_LPF2, lpf_P  << 3 | lpf_Tshock);  
 }


  void STHS34PF80::config(uint8_t avgt, uint8_t avgtmos,  uint8_t gain, bool functions)
 {
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_AVG_TRIM, avgt << 4 |  avgtmos ); // set temperature averaging 
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL0);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL0, temp | gain << 4);  // set gain

 // Configure interrupt behavior
 // select active HIGH (bit 7 == 0), push-pull (bit 6 == 0), pulsed mode (bit 2 == 0)
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL3, 0x01 ); // configure for data ready only
//  if(functions) _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL3, 0x32 ); // configure INT_OR for either Presence or Motion detection
  if(functions) _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL3, 0x3A ); // configure INT_OR for either Presence or Motion or Tamb_shock detection
 }


  void STHS34PF80::oneShot()
 {
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp | 0x01); // set bit 0 to force one-time measurement
 }


  uint8_t STHS34PF80::getFuncStatus()
  {
   uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_STATUS);
   return temp;
  }


    uint8_t STHS34PF80::getDataReadyStatus()
  {
   uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_STATUS);
   return temp;
  }


  int16_t STHS34PF80::readAmbTemp()
  {
   uint8_t rawData[2];  // 16-bit register data stored here
   _i2c_bus->readBytes(STHS34PF80_ADDRESS, STHS34PF80_TAMBIENT_L, 2, &rawData[0]);
   return (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);
  }


  int16_t STHS34PF80::readObjTemp()
  {
   uint8_t rawData[2];  // 16-bit register data stored here
   _i2c_bus->readBytes(STHS34PF80_ADDRESS, STHS34PF80_TOBJECT_L, 2, &rawData[0]);
   return (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);
  }


  int16_t STHS34PF80::readPresence()
  {
   uint8_t rawData[2];  // 16-bit register data stored here
   _i2c_bus->readBytes(STHS34PF80_ADDRESS, STHS34PF80_TPRESENCE_L, 2, &rawData[0]);
   return (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);
  }


  int16_t STHS34PF80::readMotion()
  {
   uint8_t rawData[2];  // 16-bit register data stored here
   _i2c_bus->readBytes(STHS34PF80_ADDRESS, STHS34PF80_TMOTION_L, 2, &rawData[0]);
   return (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);
  }


  int16_t STHS34PF80::readAmbShock()
  {
   uint8_t rawData[2];  // 16-bit register data stored here
   _i2c_bus->readBytes(STHS34PF80_ADDRESS, STHS34PF80_TAMB_SHOCK_L, 2, &rawData[0]);
   return (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);
  }


 void STHS34PF80::resetAlgo()
 {
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp | 0x10);                   // enable access to embedded function register
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x40);                   // enable write to embedded function register
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_RESET_ALGO); // set embedded register address to RESET_ALGO
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, 0x01);                  // set ALGO_ENABLE_RESET bit
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x00);                   // disable write to embedded function register
          temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp & ~(0x10));                // disable access to embedded function register
 }


  void STHS34PF80::configAlgo(uint16_t presence_ths, uint16_t motion_ths, uint16_t tamb_shock_ths, uint8_t presence_hyst, uint8_t motion_hyst, uint8_t tamb_shock_hyst)
 {
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp | 0x10);                            // enable access to embedded function register
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x40);                     // enable write to embedded function register
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_PRESENCE_THS_L);      // set embedded register address to PRESENCE_THS_L
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, presence_ths & 0x00FF);          // write lower byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_PRESENCE_THS_H);      // set embedded register address to PRESENCE_THS_H
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, presence_ths >> 8);              // write higher byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_MOTION_THS_L);        // set embedded register address to MOTION_THS_L
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, motion_ths & 0x00FF);            // write lower byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_MOTION_THS_H);        // set embedded register address to MOTION_THS_H
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, motion_ths >> 8);                // write higher byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_TAMB_SHOCK_THS_L);    // set embedded register address to TAMB_SHOCK_THS_L
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, tamb_shock_ths & 0x00FF);        // write lower byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_TAMB_SHOCK_THS_H);    // set embedded register address to TAMB_SHOCK_THS_H
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, tamb_shock_ths >> 8);            // write higher byte

  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_PRESENCE);       // set embedded register address to HYST_PRESNCE
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, presence_hyst);                  // write byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_MOTION);         // set embedded register address to HYST_MOTION
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, motion_hyst);                    // write byte
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_TAMB_SHOCK);     // set embedded register address to HYST_TAMB_SHOCK
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, tamb_shock_hyst);                // write byte

// set algorithm interrupt to pulsed mode (bit 3 = 1) or latch mode (bit 3 = 0)
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_ALGO_CONFIG);         // set embedded register address to ALGO_CONFIG
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA, 0x08);                           // write 0x08 (bit 3) to enable pulsed mode
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x00);                            // disable write to embedded function register
          temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp & ~(0x10));                         // disable access to embedded function register
 }


   void STHS34PF80::readAlgoConfig(uint16_t  * dest)
 {
  uint8_t rawData[2] = {0, 0};  // register data stored here
  uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp | 0x10);                            // enable access to embedded function register
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x20);                            // enable read from embedded function register
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_PRESENCE_THS_L);        // set embedded register address to PRESENCE_THS_L  
  rawData[0] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_PRESENCE_THS_H);        // set embedded register address to PRESENCE_THS_H  
  rawData[1] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  dest[0]  =  (uint16_t)(((uint16_t)rawData[1]) << 8 | rawData[0]);
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_MOTION_THS_L);          // set embedded register address to MOTION_THS_L
  rawData[0] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_MOTION_THS_H);          // set embedded register address to MOTION_THS_H
  rawData[1] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  dest[1]  =  (uint16_t)(((uint16_t)rawData[1]) << 8 | rawData[0]);
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_TAMB_SHOCK_THS_L);      // set embedded register address to TAMB_SHOCK_THS_L
  rawData[0] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_TAMB_SHOCK_THS_H);      // set embedded register address to TAMB_SHOCK_THS_H
  rawData[1] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  dest[2]  =  (uint16_t)(((uint16_t)rawData[1]) << 8 | rawData[0]);
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_MOTION);         // set embedded register address to HYST_MOTION
  rawData[0] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_PRESENCE);       // set embedded register address to HYST_PRESENCE
  rawData[1] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  dest[3]  =  (uint16_t)(((uint16_t)rawData[1]) << 8 | rawData[0]);
  
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_ALGO_CONFIG);         // set embedded register address to ALGO_CONFIG
  rawData[0] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_ADDR, STHS34PF80_HYST_TAMB_SHOCK);     // set embedded register address to HYST_TAMB_SHOCK
  rawData[1] = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_CFG_DATA);
  dest[4]  =  (uint16_t)(((uint16_t)rawData[1]) << 8 | rawData[0]);
   
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_FUNC_PAGE_RW, 0x00);                            // disable read from embedded function register
          temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2);
  _i2c_bus->writeByte(STHS34PF80_ADDRESS, STHS34PF80_CTRL2, temp & ~(0x10));                         // disable access to embedded function register
 }


   int16_t STHS34PF80::readSenseData()
  {
   uint8_t temp = _i2c_bus->readByte(STHS34PF80_ADDRESS, STHS34PF80_SENS_DATA);
   return (int16_t)((int16_t)temp << 8 | 0x00);
  }
  
