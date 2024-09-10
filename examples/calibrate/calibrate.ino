#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
    Serial.begin(250000);
    kit = createKit(false);
    // Calibrate the device for 5 minutes, monitoring 100% of the array for saturation
    kit->calibrate(300000, 1);
}

void loop()
{
    kit->scanArray();
}