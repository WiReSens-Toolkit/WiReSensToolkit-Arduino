# WiReSensToolkit Arduino Library

This is the WiReSensToolkit Arduino Library for programming microcontrollers to read and send voltages from resistive tactile sensing arrays. 

## Pre-Requisites

* Arduino ([Install](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE))
* Dependent Packages, including:
  * [NimBLE](https://www.arduino.cc/reference/en/libraries/nimble-arduino/)
  * [MCP4018 Library](https://www.arduino.cc/reference/en/libraries/ds-mcp4018-library/)
  * [ArduinoJSON](https://www.arduino.cc/reference/en/libraries/arduinojson/)
  * [Streamutils](https://www.arduino.cc/reference/en/libraries/streamutils/)

## Installation

Download this repo as a zip file and follow the Arduino IDE's instructions for installation: [installation](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE#:~:text=Importing%20a%20.zip%20Library)

## Using the Library

To use the library, initiate the serial connection (using the same baudrate configured in your JSON) and create a WiSensToolkit object from your JSON configuration (or use the most recently saved configuration on the device) using the createKit method:

```cpp
#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
  Serial.begin(250000);
  kit = createKit(false);
}
```
When the *createKit()* method is called with useSaved=true, the method will use the JSON configuration stored in Non-Volatile Storage on the MCU.
When called with useSaved=false, it will wait for the JSON configuration to be sent over serial as a utf-8 encoded string. 

### Methods

The created WiSensToolkit object has three main methods which can be composed to read and send the voltages from a resistive sensing matrix: calibrate(), scanArray(), and readNode()

#### calibrate(int duration, double saturatedPercentage)
*calibrate()* takes as input a duration in milliseconds, and automatically calibrates the gain for readout based on the pressure applied to the sensor during the calibration period, by choosing a value of the resistance that avoids saturation in the percentage of the sensing array specified by saturatedPercentage.

```cpp
void setup()
{
    Serial.begin(250000);
    kit = createKit(false);
    // Calibrate the device for 5 minutes, monitoring 100% of the array for saturation
    kit->calibrate(300000, 1);
}
```

An event that triggers that start of the calibration duration can be configured by writing a custom callback function with the name *calibrationCallback()*. By default, the device will wait for the message "start" sent over serial connection:

```cpp
boolean calibrationCallback()
{
    if (Serial.available() > 0)
    {
        String message = Serial.readStringUntil('\n');
        if (message == "start")
        {
            return true;
        }
    }
    return false;
}
```

#### scanArray()

*scanArray()* will read the sensor nodes specified by the bounding box (startCoord, endCoord) in your json configuration. It will send these values every numNodes readings using your configured communication protocol during normal operation. During intermittent operation (configured by your JSON), the device will only send the packet if its prediction is above your configured error threshold, saving power. 

```cpp
void loop()
{
    kit->scanArray();
}
```

#### readNode(int readWire,int groundWire)

*readNode()* will read the voltage at the intersection of the readWire and groundWire specified. This provides more configurable readout, allowing for any subset of nodes in your configured sensing array to be read in any order. It will send these values every numNodes/2 readings (in this method we reserve packet space for specifying node location) using your configured communication protocol. 

```cpp
void loop()
{
  kit->readNode(0, 0);
  kit->readNode(0, 1);
  kit->readNode(0, 2);
  kit->readNode(0, 3);
}
```







