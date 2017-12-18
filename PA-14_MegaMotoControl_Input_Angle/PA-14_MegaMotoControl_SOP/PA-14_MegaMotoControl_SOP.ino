
/*  
 *   Test code inputs desired angle of LOC using actuator potentiometer & PID
 *   @author Jonathan
 */
 
const int POTENTIOMETER_SIGNAL = A0; // potentiometer from actuator
const int ACTUATOR_SIGNAL = A1;      // potentiometer from throttle

int RUNSOP_BIT = 5; // bit to run SOP test procedure

const int ENABLE= 8; // enables board unless 5V is used which will keep enabled as long as Arduino has power.
const int PWMA = 11; // sends signals to control retraction
const int PWMB = 3;  // sends signals to control extension

const int POTENTIOMETER_MIN = 0;    // minimum position of throttle
const int POTENTIOMETER_MAX = 1023; // maximum position of throttle

const int ACTUATOR_MIN_POTENTIOMETER = 72;  // minimum position of actuator
const int ACTUATOR_MAX_POTENTIOMETER = 943; // maximum position of actuator

const double ACTUATOR_MIN_LENGTH = 16.23; // inches
const double ACTUATOR_MAX_LENGTH = 26.23; // inches

const double ROCKET_BEAM_LENGTH = 23.0;
const double BALOON_BEAM_LENGTH = 16.0;

//TEST BUTTON Constants
const int buttonPin = 10;     
const int ledPin = 9;

/*================== TUNABLES ==================*/
const int BUFFER = 3;   // final position must be within range of the buffer

const double pK = 1;    // proportional constant // TUNE!
const double iK = 0.02; // integral constant     // TUNE!
const double dK = 0.00; // derivative constant   // TUNE!
/*================== TUNABLES ==================*/

/*================ ANGLE INPUT ================*/

double DESIRED_ANGLE = 45; // degrees // TUNE!
//const double DOF2_DESIRED_ANGLE = 45; // degrees // TUNE!

/*================ ANGLE INPUT ================*/

int raw_throttle_feedback;
int throttle_feedback;

int testProcedure;
//TEST BUTTON Variables
int buttonState = 0;
int flagSOP=0;

double actuator_potentiometer_position;
double actuator_position;
double angle;
double desired_angle_rad;
double desired_position;
double desired_potentiometer_position;
double error;            // records error between desired postion from current postion
double cumulative_error; // error built up over time
double previous_error;
double gradient;         
double pCorrection;      
double iCorrection;
double dCorrection;

/* put your setup code here, to run once */
void setup() {
  pinMode(ACTUATOR_SIGNAL, INPUT); // feedback from actuator
  pinMode(POTENTIOMETER_SIGNAL, INPUT); // feedback from potentiometer
  pinMode(ENABLE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  pinMode(RUNSOP_BIT, INPUT);

  //TEST BUTTON 
  //Input or output?
  pinMode(ledPin, OUTPUT);      
  pinMode(buttonPin, INPUT_PULLUP);
  
//  raw_throttle_feedback = analogRead(POTENTIOMETER_SIGNAL);
//  throttle_feedback = map(raw_throttle_feedback, POTENTIOMETER_MIN, POTENTIOMETER_MAX, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL);
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
  angle = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
  desired_angle_rad = (PI/180) * DESIRED_ANGLE;
  desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_rad));
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

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

  //TEST BUTTON LOOP
  //Read button state (pressed or not pressed?)
  buttonState = digitalRead(buttonPin);

  //If button pressed...
  if (buttonState == LOW) { 
    //...ones, turn led on!
    if ( flagSOP == 0){
      digitalWrite(ledPin, HIGH);
      flagSOP=1; //change flag variable
      RUNSOP_BIT = 1;
      Serial.print("led on ");
    }
    //...twice, turn led off!
    else if ( flagSOP == 1){
      digitalWrite(ledPin, LOW);
      flagSOP=0; //change flag variable again
      Serial.print("led off ");
    }    
  }
  delay(200); //Small delay
  
  if(RUNSOP_BIT == 1) {
    runSOPInstructions();
  }
}
// angle relative to ground
// rotational angle of platform
// altitude
void runSOPInstructions() {
  testProcedure = RUNSOP_BIT;
  // 1DOF
  DESIRED_ANGLE = 45; // move to starting position
  while (!Serial.available()){
    Serial.print("Waiting...1 \n");
    error = getDesiredPosition() - getCurrentPosition();
    pCorrection = error * pK;
    cumulative_error = integrate();
    iCorrection = cumulative_error * iK;
    gradient = differentiate();
    dCorrection = gradient * dK;
    previous_error = error;
    controlActuator(getCorrections());
    //delay(1000);
    //Do Absolutely Nothing until something is received over the serial port
  }
  while (Serial.available()){
    Serial.read();
  }
  DESIRED_ANGLE == 15;
  Serial.print("Angle test 1\n"); // print current angle from IMU
  while (!Serial.available()){
    Serial.print("Waiting... 2\n");
    delay(1000);
    //Do Absolutely Nothing until something is received over the serial port
  }
  while (Serial.available()){
    Serial.read();
  }
  DESIRED_ANGLE = 30;
  Serial.print("Angle test 2\n"); // print current angle from IMU
  while (!Serial.available()){
    Serial.print("Waiting...3\n");
    delay(1000);
    //Do Absolutely Nothing until something is received over the serial port
  }
  while (Serial.available()){
    Serial.read();
  }
  DESIRED_ANGLE = 70; // or the maximum from vertical
  Serial.print("Angle test 3\n"); // print current angle from IMU
  while (!Serial.available()){
    Serial.print("Waiting...4\n");
    delay(1000);
    //Do Absolutely Nothing until something is received over the serial port
  }
  while (Serial.available()){
    Serial.read();
  }
  DESIRED_ANGLE = 45; // back to start position
  Serial.print("END 1 DOF\n");
  digitalWrite(ledPin, LOW);
  flagSOP = 0;
  RUNSOP_BIT = 0; 
  // 2DOF
  /*DOF2_DESIRED_ANGLE = 0;
  while (!Serial.available()){
      //Do Absolutely Nothing until something is received over the serial port
    }
  if(testProcedure == 4) {
    // DOF2_DESIRED_ANGLE = 360; CW
    // serial.print(IMU_angle); print current angle from IMU
    while (!Serial.available()){
      serial.print("Waiting... Press Enter to continue...")
      //Do Absolutely Nothing until something is received over the serial port
    }
    testProcedure++ // = 5
  }
  if(testProcedure == 5) {
    // DOF2_DESIRED_ANGLE = -360; CCW
    // serial.print(IMU_angle); print current angle from IMU
    while (!Serial.available()){
      serial.print("Waiting... Press Enter to continue...")
      //Do Absolutely Nothing until something is received over the serial port
    }
  }
  // DOF2_DESIRED_ANGLE = 0;
  // end test
  RUNSOP_BIT = 0;*/
}

void checkPulse() {
  if(RUNSOP_BIT == 1) {
    
  }
}

double getDesiredPosition() {
  
//    raw_throttle_feedback = analogRead(POTENTIOMETER_SIGNAL); // read the throttle potentiometer to get the destination
//    throttle_feedback = map(raw_throttle_feedback, POTENTIOMETER_MIN, POTENTIOMETER_MAX, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // convert the potentiometer feedback to match the actuator
    
  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL); // read the actuator potentiometer to get the current position
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH); // maps to find length given actuator pot reading (inches)
  angle = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH)); // super insane math to find relative position given angle (NEWTON WOULD BE IMPRESSED)
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // maps desired position taken from desired angle and maps to actuator pot

  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL);
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
  angle = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
  desired_angle_rad = (PI/180) * DESIRED_ANGLE;
  desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_rad));
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

//    Serial.print("feedback : ");
//    Serial.println(feedback);
    
//    Serial.print("Actuator Position: ");
//    Serial.println(actuator_position);

    Serial.print("Angle: ");
    Serial.println(angle * (180/PI));
    
    return desired_potentiometer_position;
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
