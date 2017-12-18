/*
  MegaMoto Test Sketch
  Simply runs a motor back and forth
  ramping the speed from 0 to full (255)

  This example code is in the public domain.
*/

int EnablePin = 12;
int duty;
int PWMPin = 9;  // Timer2
int PWMPin2 = 10;

const byte CPin = 0;  // analog input channel
int CRaw;      // raw A/D value
float CVal;    // adjusted Amps value

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(EnablePin, OUTPUT);     
  pinMode(PWMPin, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  setPwmFrequency(PWMPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
}

void loop() {
    // To drive the motor in H-bridge mode
    // the power chip inputs must be opposite polarity
    // and the Enable input must be HIGH
    digitalWrite(EnablePin, HIGH);
    analogWrite(PWMPin2, 0);
    for(duty = 0; duty <= 255; duty += 5){
      analogWrite(PWMPin, duty);
      delay(20);
    }
    analogWrite(PWMPin, 255);
    CRaw = analogRead(CPin);
    delay(2000);
    for(duty = 255; duty>=0; duty -= 5){
      analogWrite(PWMPin, duty);   
      delay(20);   
    }
    analogWrite(PWMPin, 0);
    delay(500);
    // Toggle enable to reset the power chips if we have had 
    // an overcurrent or overtemp fault
    digitalWrite(EnablePin, LOW);
    delay(500);
    
    // Swap pins to make the motor reverse
    if(PWMPin == 9) {
      PWMPin = 10;
      PWMPin2 = 9;
    } else {
      PWMPin = 9;
      PWMPin2 = 10;
    }
}
/*
* Divides a given PWM pin frequency by a divisor.
* 
* The resulting frequency is equal to the base frequency divided by
* the given divisor:
*   - Base frequencies:
*      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
*      o The base frequency for pins 5 and 6 is 62500 Hz.
*   - Divisors:
*      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
*        256, and 1024.
*      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
*        128, 256, and 1024.
* 
* PWM frequencies are tied together in pairs of pins. If one in a
* pair is changed, the other is also changed to match:
*   - Pins 5 and 6 are paired (Timer0)
*   - Pins 9 and 10 are paired (Timer1)
*   - Pins 3 and 11 are paired (Timer2)
* 
* Note that this function will have side effects on anything else
* that uses timers:
*   - Changes on pins 5, 6 may cause the delay() and
*     millis() functions to stop working. Other timing-related
*     functions may also be affected.
*   - Changes on pins 9 or 10 will cause the Servo library to function
*     incorrectly.
* 
* Thanks to macegr of the Arduino forums for his documentation of the
* PWM frequency divisors. His post can be viewed at:
*   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
*/

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) { // Timer0 or Timer1
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) { 
      TCCR0B = TCCR0B & 0b11111000 | mode; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode; // Timer1
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode; // Timer2
  }
}

