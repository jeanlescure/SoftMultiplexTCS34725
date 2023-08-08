#ifndef SoftMultiplexTCS34725_h
#define SoftMultiplexTCS34725_h

#include "Arduino.h"

#define TCS34725_ENABLE                 (0x00)
#define TCS34725_POWER_ON               (0x01)
#define TCS34725_INTERRUPT_ENABLE       (0x10)
#define TCS34725_ADC_ENABLE             (0x02)
#define TCS34725_ADDRESS                (0x29)
#define TCS34725_ID                     (0x12)
#define TCS34725_ATIME                  (0x01)
#define TCS34725_CONTROL                (0x0F)
#define TCS34725_CDATAL                 (0x14)

// Constants and enums from Adafruit_TCS34725_SoftI2C
typedef enum
{
  TCS34725_INTEGRATIONTIME_2_4MS = 0xFF,
  TCS34725_INTEGRATIONTIME_24MS = 0xF6,
  TCS34725_INTEGRATIONTIME_50MS = 0xEB,
  // ... (other integration times)
} tcs34725IntegrationTime_t;

typedef enum
{
  TCS34725_GAIN_1X = 0x00,
  TCS34725_GAIN_4X = 0x01,
  // ... (other gain values)
} tcs34725Gain_t;

class SoftMultiplexTCS34725 {
public:
    SoftMultiplexTCS34725(
      tcs34725IntegrationTime_t = TCS34725_INTEGRATIONTIME_2_4MS,
      tcs34725Gain_t = TCS34725_GAIN_1X,
      uint8_t sdaPin = 2,
      uint8_t sclPin = 3,
      uint8_t i2cDelay = 10
    );
    bool begin();
    void setInterrupt(bool flag);
    void getRawData(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);

    uint8_t SoftI2CReadRegister(uint8_t address, uint8_t reg, uint8_t *pData, uint8_t nLen);
    uint8_t SoftI2CWrite(uint8_t address, uint8_t reg, uint8_t *pData, uint8_t nLen);
    bool SoftI2CWriteByte(uint8_t byte);
    uint8_t SoftI2CReadByte(bool ack);

private:
    uint8_t _sdaPin, _sclPin;
    tcs34725IntegrationTime_t _integrationTime;
    tcs34725Gain_t _gain;
    uint8_t _i2cDelay; // Delay for I2C communication
    
    // Other private members and methods from Adafruit_TCS34725_SoftI2C and Multi_BitBang
    // ...
};

#endif
