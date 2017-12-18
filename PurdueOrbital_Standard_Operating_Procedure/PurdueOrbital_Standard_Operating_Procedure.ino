/*
     Code for Launch Orientation Control System (LOCS) Standard Operating Procedure (SOP)
     @author Jonathan Alencar
*/

#define MEGAMOTO_ACTUATOR_ENABLE 8 // enables board unless 5V is used which will keep enabled as long as Arduino has power.
#define MEGAMOTO_ACTUATOR_PWM_A 11 // sends signal to control retraction
#define MEGAMOTO_ACTUATOR_PWM_B 3  // sends signal to control extension

#define MEGAMOTO_ROTARY_ENABLE 12
#define MEGAMOTO_ROTARY_PWM_A 9
#define MEGAMOTO_ROTARY_PWM_B 10

#define ACTUATOR_SIGNAL A1 // potentiometer from linear actuator

#define PULSE_PIN 5 // pulse to traverse through operating procedures
#define STOP_PIN 2  // emergency signal to stop all operating procedures

#define ACTUATOR_MIN_POTENTIOMETER 72  // minimum position of actuator
#define ACTUATOR_MAX_POTENTIOMETER 943 // maximum position of actuator

/* All Measurements in inches (in.) */

#define ACTUATOR_MIN_LENGTH 16.23
#define ACTUATOR_MAX_LENGTH 26.23

#define BALOON_BEAM_LENGTH 16.0
#define ROCKET_BEAM_LENGTH 23.0

/*================== TUNABLES ==================*/

#define BUFFER 3 // final position must be within range of the buffer // TUNE!

#define pK 1    // proportional constant // TUNE!
#define iK 0.02 // integral constant     // TUNE!
#define dK 0.00 // derivative constant   // TUNE!

/*===============================================*/

/*================= ANGLES (°) ==================*/

#define SOP_TEST_ANGLE_DEFAULT 45
#define SOP_TEST_ANGLE_1 15
#define SOP_TEST_ANGLE_2 20
#define SOP_TEST_ANGLE_3 30
#define SOP_TEST_ANGLE_4 45

/*===============================================*/

/*============ ROTATIONAL DIRECTIONS ============*/

#define CLOCKWISE true
#define COUNTER_CLOCKWISE false

/*===============================================*/


/*==================== TESTS ====================*/

#define SOP_TEST_RESTART -1
#define SOP_TEST_DEFAULT 0
#define SOP_TEST_1 1
#define SOP_TEST_2 2
#define SOP_TEST_3 3
#define SOP_TEST_4 4
#define SOP_TEST_5 5
#define SOP_TEST_6 6

/*===============================================*/

static int procedure_state;
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

void setup() {
    pinMode(ACTUATOR_SIGNAL, INPUT); // feedback from actuator
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(MEGAMOTO_ACTUATOR_ENABLE, OUTPUT);
    pinMode(MEGAMOTO_ACTUATOR_PWM_A, OUTPUT);
    pinMode(MEGAMOTO_ACTUATOR_PWM_B, OUTPUT);

    pinMode(MEGAMOTO_ROTARY_ENABLE, OUTPUT);
    pinMode(MEGAMOTO_ROTARY_PWM_A, OUTPUT);
    pinMode(MEGAMOTO_ROTARY_PWM_B, OUTPUT);

    pinMode(PULSE_PIN, INPUT);
    pinMode(STOP_PIN, INPUT_PULLUP);

    digitalWrite(MEGAMOTO_ACTUATOR_ENABLE, HIGH);
    digitalWrite(MEGAMOTO_ROTARY_ENABLE, HIGH);

    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopProcedures, RISING);

    actuator_potentiometer_position = getCurrentPosition();
    actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
    angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
    desired_angle_degrees = SOP_TEST_ANGLE_DEFAULT;
    desired_angle_radians = desired_angle_degrees * (PI / 180);
    desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_radians));
    desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);
    cumulative_error = 0;
    previous_error = 0;

    procedure_state = SOP_TEST_DEFAULT;

    Serial.begin(19200);
}

void loop() {
    calculateActuatorError();
    controlActuator(getCorrections());
    listenSOPInstructions();
}

void traverseProcedures() {
    switch (procedure_state) {
        case (SOP_TEST_1): {
            Serial.println("Running Test Angle 1 (15°)");
            desired_angle_degrees = SOP_TEST_ANGLE_1; // 15°
            break;
        }
        case (SOP_TEST_2): {
            Serial.println("Running Test Angle 2 (20°)");
            desired_angle_degrees = SOP_TEST_ANGLE_2; // 20°
            break;
        }
        case (SOP_TEST_3): {
            Serial.println("Running Test Angle 3 (30°)");
            desired_angle_degrees = SOP_TEST_ANGLE_3; // 30°
            break;
        }
        case (SOP_TEST_4): {
            Serial.println("Running Test Angle 4 (45°)");
            desired_angle_degrees = SOP_TEST_ANGLE_4; // 45°
            break;
        }
        case (SOP_TEST_5): {
            Serial.println("Running Rotary Test Clockwise");
            controlRotary(CLOCKWISE);
            break;
        }
        case (SOP_TEST_6): {
            Serial.println("Running Rotary Test Counter-Clockwise");
            controlRotary(COUNTER_CLOCKWISE);
            break;
        }
        default:
            Serial.println("Running Default Angle");
            desired_angle_degrees = SOP_TEST_ANGLE_DEFAULT;
            procedure_state = 0;
            break;
    }
}

void stopProcedures() {
    stopActuator();
    stopRotary();
    procedure_state = SOP_TEST_RESTART;
    Serial.println("SOP Reset");
    delay(500);
}

void listenSOPInstructions() {
    if (digitalRead(PULSE_PIN) == HIGH) {
        procedure_state++;
        delay(3000);
        traverseProcedures();
    }
}

void calculateActuatorError() {
    error = getDesiredPosition() - getCurrentPosition();
    pCorrection = error * pK;
    cumulative_error = integrate();
    iCorrection = cumulative_error * iK;
    gradient = differentiate();
    dCorrection = gradient * dK;
    previous_error = error;
}

double getDesiredPosition() {

    //    Serial.print("Actuator Position: ");
    //    Serial.print(actuator_position);
    //    Serial.print(" Angle: ");
    //    Serial.println(angle_radians * (180/PI));

    actuator_potentiometer_position = getCurrentPosition(); // read the actuator potentiometer to get the current position
    actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH); // maps to find length given actuator pot reading (inches)
    angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH)); // super insane math to find relative position given angle (NEWTON WOULD BE IMPRESSED)
    desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER); // maps desired position taken from desired angle and maps to actuator pot
    actuator_position = mapFloat(actuator_potentiometer_position, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH);
    angle_radians = asin((pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - pow(actuator_position, 2)) / (2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH));
    angle_degrees = angle_radians * (180 / PI);
    desired_angle_radians = desired_angle_degrees * (PI / 180);
    desired_position = sqrt(pow(ROCKET_BEAM_LENGTH, 2) + pow(BALOON_BEAM_LENGTH, 2) - 2 * ROCKET_BEAM_LENGTH * BALOON_BEAM_LENGTH * sin(desired_angle_radians));
    desired_potentiometer_position = mapFloat(desired_position, ACTUATOR_MIN_LENGTH, ACTUATOR_MAX_LENGTH, ACTUATOR_MIN_POTENTIOMETER, ACTUATOR_MAX_POTENTIOMETER);

    return desired_potentiometer_position;
}

int getCorrections() {
    return (int) (pCorrection + iCorrection + dCorrection);
}

int getCurrentPosition() {
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
}

void controlRotary(boolean rotational_direction) {
    if (rotational_direction == CLOCKWISE) {
        rotate(MEGAMOTO_ROTARY_PWM_A);
    } else if (rotational_direction == COUNTER_CLOCKWISE) {
        rotate(MEGAMOTO_ROTARY_PWM_B);
    }
}

void rotate(int megamoto_rotary_pwm) {

    for (int speed = 0; speed <= 255; speed += 5) {
        analogWrite(megamoto_rotary_pwm, speed);
        delay(65);
    }

    analogWrite(megamoto_rotary_pwm, 255);

    for (int speed = 255; speed >= 0; speed -= 5) {
        analogWrite(megamoto_rotary_pwm, speed);
        delay(65);
    }

    analogWrite(megamoto_rotary_pwm, 0);
}

void stopActuator() {
    desired_angle_radians = angle_radians;
    analogWrite(MEGAMOTO_ACTUATOR_PWM_A, 0);
    analogWrite(MEGAMOTO_ACTUATOR_PWM_B, 0);
}

void stopRotary() {
    analogWrite(MEGAMOTO_ROTARY_PWM_A, 0);
    analogWrite(MEGAMOTO_ROTARY_PWM_B, 0);
}

void pushActuator(int speed) {
    analogWrite(MEGAMOTO_ACTUATOR_PWM_A, 0);
    analogWrite(MEGAMOTO_ACTUATOR_PWM_B, speed);
}

void pullActuator(int speed) {
    analogWrite(MEGAMOTO_ACTUATOR_PWM_A, speed);
    analogWrite(MEGAMOTO_ACTUATOR_PWM_B, 0);
}

int clamp(int speed) {
    if (abs(speed) > 255) {
        return 255;
    } else {
        return speed;
    }
}

int integrate() {
    if (abs(error) >= BUFFER && (abs(error) + abs(cumulative_error) <= (512 / iK))) {
        return error + cumulative_error;
    } else if (abs(error) >= BUFFER) {
        return cumulative_error;
    } else {
        return 0;
    }
}

int differentiate() {
    if (abs(error) >= BUFFER) {
        return error - previous_error;
    }
}

double mapFloat(double in, double in_min, double in_max, double out_min, double out_max) {
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
