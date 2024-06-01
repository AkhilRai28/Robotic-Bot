#include <SoftwareSerial.h>
#include <HCSR04.h>


HCSR04 hc(5, 6); //initialisation class HCSR04 (trig pin , echo pin)

void speed(float distance)
{
  float sent_speed;
  sent_speed = ((255*distance)/400);
  Serial.println("Speed: " + String(sent_speed));
  delay(60); 
//  analogWrite(3, int(sent_speed));
}

void setup()
{
//    pinMode(3, INPUT);
    Serial.begin(9600);
}

void loop()
{
    Serial.println("Distance: " + String(hc.dist())); // return curent distance in serial
    speed(hc.dist());
    delay(60);                 // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.
}