
/*  
 *   Test code that controls actuator a throttle potentiometer & PID
 *   @author Jonathan
 */
 
const int POTENTIOMETER_SIGNAL = A0; // potentiometer from actuator
const int ACTUATOR_SIGNAL = A1;      // potentiometer from throttle


const int ENABLE= 8; // enables board unless 5V is used which will keep enabled as long as Arduino has power.
const int PWMA = 11; // sends signals to control retraction
const int PWMB = 3;  // sends signals to control extension

const int POTENTIOMETER_MIN = 0;    // minimum position of throttle
const int POTENTIOMETER_MAX = 1023; // maximum position of throttle

const int ACTUATOR_MIN_POTENTIOMETER = 72;  // minimum position of actuator
const int ACTUATOR_MAX_POTENTIOMETER = 943; // maximum position of actuator

const double ACTUATOR_MIN_LENGTH = 16.23; // inches
const double ACTUATOR_MAX_LENGTH = 26.23; // inches

const double ROCKET_BEAM_LENGTH = 23.622; // inches
const double BALOON_BEAM_LENGTH = 10.63; // inches

/*================== TUNABLES ==================*/
const int BUFFER = 3;   // final position must be within range of the buffer

const double pK = 1;    // proportional constant // TUNE!
const double iK = 0.02; // integral constant     // TUNE!
const double dK = 0.00; // derivative constant   // TUNE!
/*================== TUNABLES ==================*/

int raw_throttle_feedback;
int throttle_feedback;
double actuator_position;
double actuator_potentiometer_position;
double error;            // records error between desired postion from current postion
double cumulative_error; // error built up over time
double previous_error;
double gradient;         
double pCorrection;      
double iCorrection;
double dCorrection;

/* Constants for second degree of freedom */
int EnablePin = 12;
int duty;
int PWMPin = 9;  // Timer2
int PWMPin2 = 10;

const byte CPin = 0;  // analog input channel
int CRaw;      // raw A/D value
float CVal;    // adjusted Amps value


/* put your setup code here, to run once */
void setup() {
  pinMode(ACTUATOR_SIGNAL, INPUT); // feedback from actuator
  pinMode(POTENTIOMETER_SIGNAL, INPUT); // feedback from potentiometer
  pinMode(ENABLE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  //second degree of freedom
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(EnablePin, OUTPUT);     
  pinMode(PWMPin, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  setPwmFrequency(PWMPin, 7);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  
  raw_throttle_feedback = analogRead(POTENTIOMETER_SIGNAL);
  throttle_feedback = map(raw_throttle_feedback, POTENTIOMETER_MIN, POTENTIOMETER_MAX, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL);
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);

  cumulative_error = 0;
  previous_error = 0;
  
  Serial.begin(9600);

}

/* put your setup code here, to run once */
void loop() {
  error = getDesiredPosition() - getCurrentPosition();
  pCorrection = error * pK;
  cumulative_error = integrate();
  iCorrection = cumulative_error * iK;
  gradient = differentiate();
  dCorrection = gradient * dK;
  previous_error = error;
  controlActuator(getCorrections());

  // Second degree of freedom
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

//second degree of freedom additon
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


double getDesiredPosition() {
  
    raw_throttle_feedback = analogRead(POTENTIOMETER_SIGNAL); // read the throttle potentiometer to get the destination
    throttle_feedback = map(raw_throttle_feedback, POTENTIOMETER_MIN, POTENTIOMETER_MAX, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // convert the potentiometer feedback to match the actuator
    
    actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL); // read the actuator potentiometer to get the current position
    actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH); // maps to find length given actuator pot reading (inches)
    double angle = acos((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH)); // (SLIGHTLY INNACURATE!) super insane math to find relative position given angle


//    Serial.print("Actuator Position: ");
//    Serial.println(actuator_position);

    Serial.print("Angle: ");
    Serial.println((angle * (180/PI))-5); // SLIGHTLY INNACURATE!
    
    
    return throttle_feedback;
}

int getCurrentPosition() {
  
//  Serial.print("Actuator Position: ");
//  Serial.println(analogRead(ACTUATOR_SIGNAL));
  
  return analogRead(ACTUATOR_SIGNAL);
}


void controlActuator(int speed) {
  if (error >= BUFFER) {
    pushActuator(clamp(speed));
  } else if (error <= -BUFFER) {
    pullActuator(clamp(-speed));
  } else {
    stopActuator();
    cumulative_error = 0;
  }

//  Serial.print("Error: ");
//  Serial.println((int) error);
//  Serial.print("Speed :");
//  Serial.println(clamp(speed));
}

int getCorrections() {
  return (int) (pCorrection + iCorrection + dCorrection);
}

void stopActuator() {
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
}

void pushActuator(int speed) { 
  analogWrite(PWMA, 0); 
  analogWrite(PWMB, speed);
}

void pullActuator(int speed) {
  analogWrite(PWMA, speed);
  analogWrite(PWMB, 0);
}

int clamp(int speed) {
  if (abs(speed) > 255) {
    return 255;
  } else {
    return speed;
  }
}

int integrate() {
  if (abs(error) >= BUFFER && (abs(error) + abs(cumulative_error) <= (512/iK))) {
    return error + cumulative_error;
  } else if (abs(error) >= BUFFER) {
    return cumulative_error;
  } else {
    return 0;
  }

//  Serial.print("Cumulative Error: ");
//  Serial.println(cumulative_error);
}

int differentiate() {
  if (abs(error) >= BUFFER) {
    return error - previous_error;
  }

//  Serial.print("Gradient: ");
//  Serial.println(gradient);
}

double mapFloat(double in, double in_min, double in_max, double out_min, double out_max) {
  return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
