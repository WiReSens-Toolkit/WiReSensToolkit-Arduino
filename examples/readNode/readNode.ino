#include <WiSensToolkit.h>

WiSensToolkit *kit = nullptr;

void setup()
{
  Serial.begin(250000);
  kit = createKit(false);
}

void loop()
{
  kit->readNode(0, 0);
  kit->readNode(0, 1);
  kit->readNode(0, 2);
  kit->readNode(0, 3);
}