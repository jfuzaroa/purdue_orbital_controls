
/*  
 *   Code for Launch Orientation Control System (LOCS) Standard Operating Procedure (SOP)
 *   @author Jonathan Alencar
 */
 
#define POTENTIOMETER_SIGNAL A0 // potentiometer from actuator
#define ACTUATOR_SIGNAL A1      // potentiometer from throttle

#define ENABLE 8 // enables board unless 5V is used which will keep enabled as long as Arduino has power.
#define PWMB 3   // sends signal to control extension
#define PWMA 11  // sends signal to control retraction

#define PULSE_PIN 2 // pulse to traverse through operating procedures
#define STOP_PIN 5  // emergency pin to stop all operating procedures
#define LED_PIN 9   // LED pin to light LED when pulse is triggered

#define POTENTIOMETER_MIN 0    // minimum position of throttle
#define POTENTIOMETER_MAX 1023 // maximum position of throttle

#define ACTUATOR_MIN_POTENTIOMETER 72  // minimum position of actuator
#define ACTUATOR_MAX_POTENTIOMETER 943 // maximum position of actuator

/* All Measurements in inches (in.) */

#define ACTUATOR_MIN_LENGTH 16.23
#define ACTUATOR_MAX_LENGTH 26.23

#define BALOON_BEAM_LENGTH 16.0
#define ROCKET_BEAM_LENGTH 23.0
  
/*================== TUNABLES ==================*/

#define BUFFER 3 // final position must be within range of the buffer // TUNE!
#define ANGLE_BUFFER 2 // 1st DOF must be within range of this buffer

#define pK 1    // proportional constant // TUNE!
#define iK 0.02 // integral constant     // TUNE!
#define dK 0.00 // derivative constant   // TUNE!

/*===============================================*/

/*================= ANGLES (°) ==================*/

#define SOP_TEST_ANGLE_DEFAULT 45
#define SOP_TEST_ANGLE_1 15
#define SOP_TEST_ANGLE_2 20
#define SOP_TEST_ANGLE_3 30

/*===============================================*/

/*==================== TESTS ====================*/

static boolean sop_test_1_complete = false;
static boolean sop_test_2_complete = false;
static boolean sop_test_3_complete = false;

/*===============================================*/

// int raw_throttle_feedback;
// int throttle_feedback;

static double actuator_potentiometer_position;
static double actuator_position;
static double angle_degrees;
static double angle_radians;
static double desired_angle_degrees;
static double desired_angle_radians;
static double desired_position;
static double desired_potentiometer_position;
static double error;            // records error between desired postion from current postion
static double cumulative_error; // error built up over time
static double previous_error;
static double gradient;         
static double pCorrection;      
static double iCorrection;
static double dCorrection;

static unsigned long interrupt_time;
static unsigned long last_interrupt_time;

void setup() {
  pinMode(ACTUATOR_SIGNAL, INPUT); // feedback from actuator
  pinMode(POTENTIOMETER_SIGNAL, INPUT); // feedback from potentiometer
  pinMode(ENABLE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  pinMode(PULSE_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), traverseProcedures, RISING);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopProcedures, RISING);

  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL);
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
  angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
  desired_angle_degrees = SOP_TEST_ANGLE_DEFAULT;
  desired_angle_radians = desired_angle_degrees * (PI/180);
  desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_radians));
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);
  cumulative_error = 0;
  previous_error = 0;
  last_interrupt_time = 0;
  
  Serial.begin(14400);
}

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
  angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH)); // super insane math to find relative position given angle (NEWTON WOULD BE IMPRESSED)
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // maps desired position taken from desired angle and maps to actuator pot

  actuator_potentiometer_position = analogRead(ACTUATOR_SIGNAL);
  actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
  angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
  angle_degrees = angle_radians * (180/PI);
  desired_angle_radians = desired_angle_degrees * (PI/180);
  desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_radians));
  desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

//    Serial.print("feedback : ");
//    Serial.println(feedback);
    
    Serial.print("Actuator Position: ");
    Serial.print(actuator_position);

    Serial.print(" Angle: ");
    Serial.println(angle_radians * (180/PI));
    
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
  desired_angle_radians = angle_radians;
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

void traverseProcedures() {


  interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 200) {
    return;
  }
  
  if (!sop_test_1_complete && checkTestProcedure()) {
    Serial.println("Running Test Angle 1: " + sop_test_1_complete);
    desired_angle_degrees = SOP_TEST_ANGLE_1; // 15°
    sop_test_1_complete = true; 
  } else if (!sop_test_2_complete && checkTestProcedure()) {
    Serial.println("Running Test Angle 2");
    desired_angle_degrees = SOP_TEST_ANGLE_2; // 30°
    sop_test_2_complete = true;
  } else if (!sop_test_3_complete && checkTestProcedure()) {
    Serial.println("Running Test Angle 3");
    desired_angle_degrees = SOP_TEST_ANGLE_3; // 70°
    sop_test_3_complete = true;
  } else if (checkTestProcedure()) {
    Serial.println("Running Default Angle");
    desired_angle_degrees = SOP_TEST_ANGLE_DEFAULT;  // 45°
    sop_test_1_complete = false;
    sop_test_2_complete = false;
    sop_test_3_complete = false;
  }

  last_interrupt_time = interrupt_time;
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

boolean checkTestProcedure() {
  if (sop_test_3_complete) {
    if (isWithinRange(SOP_TEST_ANGLE_3)) {
      Serial.println("Within SOP_TEST_ANGLE_3");
      return true;
    }
  } else if (sop_test_2_complete) {
    if (isWithinRange(SOP_TEST_ANGLE_2)) {
      Serial.println("Within SOP_TEST_ANGLE_2");
      return true;
    }
  } else if (sop_test_1_complete) {
    if (isWithinRange(SOP_TEST_ANGLE_1)) {
      Serial.println("Within SOP_TEST_ANGLE_1");
      return true;
    }
  } else if (isWithinRange(SOP_TEST_ANGLE_DEFAULT)) {
    Serial.println("Within SOP_ANGLE_DEFAULT");
    return true;
  }

  Serial.println("Running Procedure");
  return false;
}

void stopProcedures() {
  stopActuator();
  sop_test_1_complete = false;
  sop_test_2_complete = false;
  sop_test_3_complete = false;

  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
  }
}



double mapFloat(double in, double in_min, double in_max, double out_min, double out_max) {
  return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

boolean isWithinRange(double desired_angle) {
  if (angle_degrees > (desired_angle - ANGLE_BUFFER) && angle_degrees < (desired_angle + ANGLE_BUFFER)) {
    return true;
  } else {
    return false;
  }
}

