# SoftMultiplexTCS34725

The `SoftMultiplexTCS34725` library allows users to interface with multiple TCS34725 RGB sensors using software-based I2C multiplexing. It's designed to work with both digital and analog pins on Arduino platforms.

## Features

- Software-based I2C multiplexing for TCS34725 RGB sensors.
- Support for both digital and analog pins.
- Easy-to-use interface for reading raw color data.

## Installation

1. Download the ZIP file from the [GitHub repository](https://github.com/ArduinoGPT/SoftMultiplexTCS34725).
2. Open the Arduino IDE, go to Sketch -> Include Library -> Add .ZIP Library, and select the downloaded ZIP file.
3. Include the library in your sketch by going to Sketch -> Include Library -> SoftMultiplexTCS34725.

## Usage

### Instantiation

Create instances for each TCS34725 sensor, specifying the integration time, gain, SDA pin, and SCL pin:

```cpp
#define SDA_PIN_A 3
#define SDA_PIN_B 4
#define SCL_PIN 5

SoftMultiplexTCS34725 tcsA = SoftMultiplexTCS34725(
  TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X,
  SDA_PIN_A,
  SCL_PIN
);

SoftMultiplexTCS34725 tcsB = SoftMultiplexTCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X,
  SDA_PIN_B,
  SCL_PIN
);
```

Note: the `SCL_PIN`(clock) pin can be shared by multiple instances.

### Initialization

Initialize each sensor using the `begin` method:

```cpp
void beginTcs(SoftMultiplexTCS34725& tcs) {
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

beginTcs(tcsA);
beginTcs(tcsB);
```

### Reading Data

Read raw color data from each sensor:

```cpp
uint16_t red, green, blue, clear;

// Read data from sensor A
tcsA.getRawData(&red, &green, &blue, &clear);
// ...

// Read data from sensor B
tcsB.getRawData(&red, &green, &blue, &clear);
// ...
```

## License

This project is licensed under the Apache-2.0 License.

## Support

For support, please open an issue on the [GitHub repository](https://github.com/jeanlescure/SoftMultiplexTCS34725/issues).
