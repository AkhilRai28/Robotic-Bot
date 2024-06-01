#include <HCSR04.h>

HCSR04 hc(5, 6); //initialisation class HCSR04 (trig pin , echo pin)
HCSR04 hcl(3, 2);   //Left Sensor
HCSR04 hcc(7, 6);   //Center Sensor (trig pin , echo pin)
HCSR04 hcr(5, 4); //Right Sensor

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    float ul = hcl.dist();
    float uc = hcc.dist();
    float ur = hcr.dist();
    Serial.println(100*ul); // return curent distance in serial
    Serial.println(1000*uc); 
    Serial.println(10000*ur); 
    delay(100);                 // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.
}