#include "application.h"
#include "tlc_driver.h"


TlcDriver tlc;
int colorStep = 0;
int frameStep = 0;

void setup()
{
    Serial.begin(9600);
    tlc.init();
}


void loop()
{
   tlc.clear();

    colorStep = frameStep%3;

    tlc.set(0+colorStep, 4095);
    tlc.set(3+colorStep, 4095);
    tlc.set(6+colorStep, 4095);
    tlc.set(9+colorStep, 4095);
    tlc.set(12+colorStep, 4095);

    tlc.update();

    frameStep++;
    delay(1000);
}


