
/*  
 *   Test code inputs desird angle of LOC using actuator potentiometer & PID
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

const double ROCKET_BEAM_LENGTH = 23.0;
const double BALOON_BEAM_LENGTH = 16.0;

/*================== TUNABLES ==================*/
const int BUFFER = 3;   // final position must be within range of the buffer

const double pK = 1;    // proportional constant // TUNE!
const double iK = 0.02; // integral constant     // TUNE!
const double dK = 0.00; // derivative constant   // TUNE!
/*================== TUNABLES ==================*/

/*================ ANGLE INPUT ================*/

const double DESIRED_ANGLE = 45; // degrees // TUNE!

/*================ ANGLE INPUT ================*/

int raw_throttle_feedback;
int throttle_feedback;
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
}

double getDesiredPosition() {
  
//    raw_throttle_feedback = analogRead(POTENTIOMETER_SIGNAL); // read the throttle potentiometer to get the destination
//    throttle_feedback = map(raw_throttle_feedback, POTENTIOMETER_MIN, POTENTIOMETER_MAX, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // convert the potentiometer feedback to match the actuator
    
    actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL); // read the actuator potentiometer to get the current position
    actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH); // maps to find length given actuator pot reading (inches)
    angle = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH)); // super insane math to find relative position given angle (NEWTON WOULD BE IMPRESSED)
    desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // maps desired position taken from desired angle and maps to actuator pot

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
