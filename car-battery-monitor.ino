  #include <LowPower.h>

/*--------------------------------------------------------------
  Program:      volt_measure

  Description:  Reads value on analog input A2 and calculates
                the voltage assuming that a voltage divider
                network on the pin divides by 11.
  
  Hardware:     Arduino Uno with voltage divider on A2.
                
  Software:     Developed using Arduino 1.0.5 software
                Should be compatible with Arduino 1.0 +

  Date:         22 May 2013
 
  Author:       W.A. Smith, http://startingelectronics.org
--------------------------------------------------------------*/

// number of analog samples to take per reading
#define NUM_SAMPLES 50
const int relayPin = 11;
const int accPin = 2;

int sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float voltage = 0.0;            // calculated voltage
float voltOut = 0.0;
int relayStatus = 0;

void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void setup()
{
    Serial.begin(9600);
    pinMode(relayPin, OUTPUT);
    pinMode(accPin, INPUT);
}

void loop()
{
  attachInterrupt(0, wakeUp, HIGH);
  
    // take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum += analogRead(A2);
        sample_count++;
        delay(10);
    }
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 3.574 is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 3.595) / 1024.0;
    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.161 is the calibrated voltage divide
    // value and 0.695 is the diode voltage drop
    voltOut = voltage * 11.161 + 0.695;
    Serial.println(voltage);  
    Serial.print(voltOut);
    Serial.println (" V");
    sample_count = 0;
    sum = 0;
    if (voltOut < 12.25) { 
     if (relayStatus == 1) {
      Serial.println("low voltage, turning off relay");
      digitalWrite(relayPin, LOW); 
      relayStatus = 0;
     }
     if (digitalRead(accPin) == 0) {
        Serial.println("low voltage and car not running so going to sleep");
        delay(500);
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        detachInterrupt(0); 
     }
    }
    else { 
      if (relayStatus == 0) {
        Serial.println("voltage has recovered, turning on relay");
        digitalWrite(relayPin, HIGH);
        relayStatus = 1;
      } 
      }
    delay(1000);
}
