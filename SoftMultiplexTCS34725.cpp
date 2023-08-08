#include "SoftMultiplexTCS34725.h"

SoftMultiplexTCS34725::SoftMultiplexTCS34725(tcs34725IntegrationTime_t integrationTime, tcs34725Gain_t gain, uint8_t sdaPin, uint8_t sclPin)
    : _integrationTime(integrationTime), _gain(gain), _sdaPin(sdaPin), _sclPin(sclPin) {
    
    // Initialize the pins for software I2C
    if (_sdaPin < A0) { // Digital pin
        pinMode(_sdaPin, INPUT_PULLUP);
    } else { // Analog pin
        pinMode(_sdaPin, INPUT);
    }

    if (_sclPin < A0) { // Digital pin
        pinMode(_sclPin, OUTPUT);
        digitalWrite(_sclPin, HIGH); // Set SCL high
    } else { // Analog pin
        pinMode(_sclPin, OUTPUT);
        analogWrite(_sclPin, 255); // Set SCL high
    }
}

bool SoftMultiplexTCS34725::begin() {
    // Try to read the device ID from the TCS34725 sensor
    uint8_t id;
    if (!SoftI2CReadRegister(TCS34725_ADDRESS, TCS34725_ID, &id, 1)) {
        return false; // Failed to read from the sensor
    }

    // Check if the ID matches the expected values for TCS34725
    Serial.println(id, HEX);
    if ((id != 0x4d) && (id != 0x44) && (id != 0x10)) {
        return false; // Wrong sensor ID
    }

    // Set the integration time and gain
    if (!SoftI2CWrite(TCS34725_ADDRESS, TCS34725_ATIME, (uint8_t *)&_integrationTime, 1)) {
        return false; // Failed to set integration time
    }

    if (!SoftI2CWrite(TCS34725_ADDRESS, TCS34725_CONTROL, (uint8_t *)&_gain, 1)) {
        return false; // Failed to set gain
    }

    // Enable the sensor
    uint8_t enable = TCS34725_POWER_ON | TCS34725_ADC_ENABLE;
    if (!SoftI2CWrite(TCS34725_ADDRESS, TCS34725_ENABLE, &enable, 1)) {
        return false; // Failed to enable the sensor
    }

    return true; // Initialization successful
}

void SoftMultiplexTCS34725::setInterrupt(bool flag) {
    uint8_t regValue;
    
    // Read the current value of the ENABLE register
    if (!SoftI2CReadRegister(TCS34725_ADDRESS, TCS34725_ENABLE, &regValue, 1)) {
        return; // Failed to read from the sensor
    }

    // Set or clear the interrupt enable bit based on the flag
    if (flag) {
        regValue |= TCS34725_INTERRUPT_ENABLE; // Enable interrupt
    } else {
        regValue &= ~TCS34725_INTERRUPT_ENABLE; // Disable interrupt
    }

    // Write the updated value back to the ENABLE register
    SoftI2CWrite(TCS34725_ADDRESS, TCS34725_ENABLE, &regValue, 1);
}

void SoftMultiplexTCS34725::getRawData(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear) {
    uint8_t data[8]; // Buffer to store raw data from the sensor

    // Read the raw color data from the sensor
    if (!SoftI2CReadRegister(TCS34725_ADDRESS, TCS34725_CDATAL, data, 8)) {
        return; // Failed to read from the sensor
    }

    // Convert the 8 bytes of data into 4 16-bit values
    *clear = ((uint16_t)data[1] << 8) | data[0];
    *red   = ((uint16_t)data[3] << 8) | data[2];
    *green = ((uint16_t)data[5] << 8) | data[4];
    *blue  = ((uint16_t)data[7] << 8) | data[6];
}

uint8_t SoftMultiplexTCS34725::SoftI2CReadRegister(uint8_t address, uint8_t reg, uint8_t *pData, uint8_t nLen) {
    // Helper functions to handle both digital and analog pins
    auto writePin = [this](uint8_t pin, uint8_t value) {
        if (pin < A0) {
            digitalWrite(pin, value);
        } else {
            analogWrite(pin, value ? 255 : 0);
        }
    };

    auto readPin = [this](uint8_t pin) -> uint8_t {
        if (pin < A0) {
            return digitalRead(pin);
        } else {
            return analogRead(pin) > 512 ? HIGH : LOW;
        }
    };

    // Start the I2C communication
    writePin(_sclPin, HIGH);
    pinMode(_sdaPin, OUTPUT);
    writePin(_sdaPin, LOW);
    delayMicroseconds(10); // Delay for the start condition
    writePin(_sclPin, LOW);

    // Send the device address with write flag (last bit is 0)
    for (uint8_t i = 0; i < 8; i++) {
        writePin(_sdaPin, address & 0x80);
        delayMicroseconds(1);
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
        address <<= 1;
    }

    // Check for acknowledgment
    pinMode(_sdaPin, INPUT);
    writePin(_sclPin, HIGH);
    if (readPin(_sdaPin)) {
        return 0; // No acknowledgment
    }
    writePin(_sclPin, LOW);

    // Send the register address
    for (uint8_t i = 0; i < 8; i++) {
        writePin(_sdaPin, reg & 0x80);
        delayMicroseconds(1);
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
        reg <<= 1;
    }

    // Check for acknowledgment
    writePin(_sclPin, HIGH);
    if (readPin(_sdaPin)) {
        return 0; // No acknowledgment
    }
    writePin(_sclPin, LOW);

    // Restart for reading data
    writePin(_sdaPin, HIGH);
    delayMicroseconds(1);
    writePin(_sclPin, HIGH);
    delayMicroseconds(1);
    writePin(_sdaPin, LOW);
    delayMicroseconds(1);
    writePin(_sclPin, LOW);

    // Send the device address with read flag (last bit is 1)
    address |= 0x01; // Set the read flag
    for (uint8_t i = 0; i < 8; i++) {
        writePin(_sdaPin, address & 0x80);
        delayMicroseconds(1);
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
        address <<= 1;
    }

    // Check for acknowledgment
    writePin(_sclPin, HIGH);
    if (readPin(_sdaPin)) {
        return 0; // No acknowledgment
    }
    writePin(_sclPin, LOW);

    // Read the data bytes
    for (uint8_t byteCount = 0; byteCount < nLen; byteCount++) {
        uint8_t byte = 0;
        for (uint8_t bitCount = 0; bitCount < 8; bitCount++) {
            byte <<= 1;
            writePin(_sclPin, HIGH);
            if (readPin(_sdaPin)) {
                byte |= 1;
            }
            writePin(_sclPin, LOW);
        }
        pData[byteCount] = byte;

        // Send acknowledgment except for the last byte
        if (byteCount < nLen - 1) {
            writePin(_sdaPin, LOW);
        } else {
            writePin(_sdaPin, HIGH);
        }
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
    }

    // Stop condition
    writePin(_sdaPin, LOW);
    delayMicroseconds(1);
    writePin(_sclPin, HIGH);
    delayMicroseconds(1);
    writePin(_sdaPin, HIGH);

    return 1; // Successful read
}

uint8_t SoftMultiplexTCS34725::SoftI2CWrite(uint8_t address, uint8_t reg, uint8_t *pData, uint8_t nLen) {
    // Helper functions to handle both digital and analog pins
    auto writePin = [this](uint8_t pin, uint8_t value) {
        if (pin < A0) {
            digitalWrite(pin, value);
        } else {
            analogWrite(pin, value ? 255 : 0);
        }
    };

    auto readPin = [this](uint8_t pin) -> uint8_t {
        if (pin < A0) {
            return digitalRead(pin);
        } else {
            return analogRead(pin) > 512 ? HIGH : LOW;
        }
    };

    // Start the I2C communication
    writePin(_sclPin, HIGH);
    pinMode(_sdaPin, OUTPUT);
    writePin(_sdaPin, LOW);
    delayMicroseconds(10); // Delay for the start condition
    writePin(_sclPin, LOW);

    // Send the device address with write flag (last bit is 0)
    for (uint8_t i = 0; i < 8; i++) {
        writePin(_sdaPin, address & 0x80);
        delayMicroseconds(1);
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
        address <<= 1;
    }

    // Check for acknowledgment
    pinMode(_sdaPin, INPUT);
    writePin(_sclPin, HIGH);
    if (readPin(_sdaPin)) {
        return 0; // No acknowledgment
    }
    writePin(_sclPin, LOW);

    // Send the register address
    for (uint8_t i = 0; i < 8; i++) {
        writePin(_sdaPin, reg & 0x80);
        delayMicroseconds(1);
        writePin(_sclPin, HIGH);
        delayMicroseconds(1);
        writePin(_sclPin, LOW);
        reg <<= 1;
    }

    // Check for acknowledgment
    writePin(_sclPin, HIGH);
    if (readPin(_sdaPin)) {
        return 0; // No acknowledgment
    }
    writePin(_sclPin, LOW);

    // Write the data bytes
    for (uint8_t byteCount = 0; byteCount < nLen; byteCount++) {
        uint8_t byte = pData[byteCount];
        for (uint8_t bitCount = 0; bitCount < 8; bitCount++) {
            writePin(_sdaPin, byte & 0x80);
            delayMicroseconds(1);
            writePin(_sclPin, HIGH);
            delayMicroseconds(1);
            writePin(_sclPin, LOW);
            byte <<= 1;
        }

        // Check for acknowledgment
        writePin(_sclPin, HIGH);
        if (readPin(_sdaPin)) {
            return 0; // No acknowledgment
        }
        writePin(_sclPin, LOW);
    }

    // Stop condition
    writePin(_sdaPin, LOW);
    delayMicroseconds(1);
    writePin(_sclPin, HIGH);
    delayMicroseconds(1);
    writePin(_sdaPin, HIGH);

    return 1; // Successful write
}

