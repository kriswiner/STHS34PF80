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
 
#ifndef STHS34PF80_h
#define STHS34PF80_h

#include "Arduino.h"
#include "I2CDev.h"
#include <Wire.h>

/*
 * https://www.st.com/resource/en/datasheet/sths34pf80.pdf
 */
// Register mapping
#define STHS34PF80_LPF1                     0x0C
#define STHS34PF80_LPF2                     0x0D   
#define STHS34PF80_WHO_AM_I                 0x0F // 0xD3 return
#define STHS34PF80_AVG_TRIM                 0x10
#define STHS34PF80_CTRL0                    0x17
#define STHS34PF80_SENS_DATA                0x1D
#define STHS34PF80_CTRL1                    0x20
#define STHS34PF80_CTRL2                    0x21
#define STHS34PF80_CTRL3                    0x22
#define STHS34PF80_STATUS                   0x23
#define STHS34PF80_FUNC_STATUS              0x25
#define STHS34PF80_TOBJECT_L                0x26
#define STHS34PF80_TOBJECT_H                0x27
#define STHS34PF80_TAMBIENT_L               0x28
#define STHS34PF80_TAMBIENT_H               0x29
#define STHS34PF80_TOBJ_COMP_L              0x38
#define STHS34PF80_TOBJ_COMP_H              0x39
#define STHS34PF80_TPRESENCE_L              0x3A
#define STHS34PF80_TPRESENCE_H              0x3B
#define STHS34PF80_TMOTION_L                0x3C
#define STHS34PF80_TMOTION_H                0x3D
#define STHS34PF80_TAMB_SHOCK_L             0x3E
#define STHS34PF80_TAMB_SHOCK_H             0x3F

// Embedded functions page register mapping
#define STHS34PF80_FUNC_CFG_ADDR            0x08
#define STHS34PF80_FUNC_CFG_DATA            0x09
#define STHS34PF80_FUNC_PAGE_RW             0x11

#define STHS34PF80_PRESENCE_THS_L           0x20
#define STHS34PF80_PRESENCE_THS_H           0x21
#define STHS34PF80_MOTION_THS_L             0x22
#define STHS34PF80_MOTION_THS_H             0x23
#define STHS34PF80_TAMB_SHOCK_THS_L         0x24
#define STHS34PF80_TAMB_SHOCK_THS_H         0x25
#define STHS34PF80_HYST_MOTION              0x26
#define STHS34PF80_HYST_PRESENCE            0x27
#define STHS34PF80_ALGO_CONFIG              0x28
#define STHS34PF80_HYST_TAMB_SHOCK          0x29
#define STHS34PF80_RESET_ALGO               0x2A


#define STHS34PF80_ADDRESS  0x5A  // only available I2C address

#define NORMAL_MODE 0x07  // default gain mode
#define WIDE_MODE   0x00

// Averaging for ambient temperature
typedef enum {
  STHS34PF80_AVGT_8                = 0x00, // default
  STHS34PF80_AVGT_4                = 0x01,  
  STHS34PF80_AVGT_2                = 0x02,
  STHS34PF80_AVGT_1                = 0x03
} AVGT;

// Averaging for object temperature
typedef enum {
  STHS34PF80_AVGTMOS_2                = 0x00,  
  STHS34PF80_AVGTMOS_8                = 0x01,  
  STHS34PF80_AVGTMOS_32               = 0x02,
  STHS34PF80_AVGTMOS_128              = 0x03,  // default
  STHS34PF80_AVGTMOS_256              = 0x04,  
  STHS34PF80_AVGTMOS_512              = 0x05,  
  STHS34PF80_AVGTMOS_1024             = 0x06,
  STHS34PF80_AVGTMOS_2048             = 0x07
} AVGTMOS;


// Low-pass filter options
typedef enum  {
  STHS34PF80_ODR_09           = 0x00, // low pass filter = ODR divided by 9, etc.
  STHS34PF80_ODR_20           = 0x01,
  STHS34PF80_ODR_50           = 0x02,
  STHS34PF80_ODR_100          = 0x03,
  STHS34PF80_ODR_200          = 0x04,
  STHS34PF80_ODR_400          = 0x05,
  STHS34PF80_ODR_800          = 0x06
} LPF;


// ODR (data sample rate)
typedef enum {
  STHS34PF80_PWRDOWN            = 0x00,  
  STHS34PF80_ODR_0_25           = 0x01,  
  STHS34PF80_ODR_0_50           = 0x02,
  STHS34PF80_ODR_1              = 0x03,  
  STHS34PF80_ODR_2              = 0x04,  
  STHS34PF80_ODR_4              = 0x05,  
  STHS34PF80_ODR_8              = 0x06,
  STHS34PF80_ODR_15             = 0x07,
  STHS34PF80_ODR_30             = 0x08
} IRODR;


class STHS34PF80
{
  public: 
  STHS34PF80(I2Cdev* i2c_bus);
  uint8_t getChipID();
  void reset();
  void powerDown();
  void powerUp(uint8_t odr);
  void setLowpassFilters(uint8_t lpf_P, uint8_t lpf_M, uint8_t lpf_PM, uint8_t lpf_Tshock);
  void config(uint8_t avgt, uint8_t avgtmos, uint8_t gain, bool functions);
  void oneShot();
  int16_t readAmbTemp();
  int16_t readObjTemp();
  int16_t readPresence();
  int16_t readMotion();
  int16_t readAmbShock();
  int16_t readSenseData();
  uint8_t getDataReadyStatus();
  uint8_t getFuncStatus();
  void    configAlgo(uint16_t presence_ths, uint16_t motion_ths, uint16_t tamb_shock_ths, uint8_t presence_hyst, uint8_t motion_hyst, uint8_t tamb_shock_hyst);
  void    readAlgoConfig(uint16_t * dest);
  void    resetAlgo();

  private:
  I2Cdev* _i2c_bus;
};

#endif
