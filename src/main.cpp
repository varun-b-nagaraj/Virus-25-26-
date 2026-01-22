#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
// #include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/motor_group.hpp"
#include <cmath> // for pow/abs
#include "pros/optical.hpp"
#include "pros/distance.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// ====================
// DRIVETRAIN HARDWARE
// ====================
pros::MotorGroup leftMotors({-16, -17},
                           pros::MotorGearset::blue); // left motor group - ports 1 and 2 (reversed)
pros::MotorGroup rightMotors({9, 8},
                            pros::MotorGearset::blue); // right motor group - ports 3 and 4 (unreversed)


// ================
// INTAKE (NEW)
// ================
pros::Motor intake1(-20, pros::MotorGears::blue);
pros::Motor intake2(-15, pros::MotorGears::blue);

pros::Motor choice(19, pros::MotorGears::blue);

// =====================
// UNUSED SUBSYSTEMS (commented out but kept for reference)
// =====================
// pros::adi::Pneumatics MogoMech('h', true); // Pneumatics on port H
// pros::Motor highStake(7, pros::MotorGears::red);
// pros::Motor leftArm(20, pros::MotorGears::green);
pros::Distance leftSensor(4); //redid
pros::Distance rightSensor(3);
pros::Distance forwardSensor(14);
pros::Distance backSensor(5);

pros::Optical opticalSensor(2);// ================
// SENSORS (used for odom/drivetrain)
// ================
pros::Imu imu(13);                    // IMU
pros::Rotation verticalEnc(11);  // Rotation sensor on port 8, reversed
// pros::Rotation horizontalEnc(3);  // Rotation sensor on port 9, reversed
// Tracking wheel object (Vertical). 2" wheel, 0" offset (update if measured differently).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);



// ====================
// DRIVETRAIN SETTINGS
// ====================
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                             &rightMotors,                // right motor group
                             15,                          // track width (inches)
                             lemlib::Omniwheel::NEW_325,  // using 3.25" omnis
                             360,                          // drivetrain rpm
                             8);                          // horizontal drift (unitless tuning)


// lateral motion controller
lemlib::ControllerSettings linearController(13, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              42, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);


// angular motion controller
lemlib::ControllerSettings angularController(
    5,  // kP (was 5)
    0.0,  // kI
    28.0, // kD (was 20)
    0.0,  // anti-windup
    1.0,  // small error range (deg) (was 2)
    0,  // small error timeout (ms) (was 650)
    3.0,  // large error range (deg) (was 6)
    1000, // large error timeout (ms) (was 1500)
    12    // max accel slew (was 0)
);



// sensors for odometry
lemlib::OdomSensors sensors(&vertical,  // vertical tracking wheel
                           nullptr,    // vertical tracking wheel 2
                           nullptr,    // horizontal tracking wheel
                           nullptr,    // horizontal tracking wheel 2
                           &imu);      // inertial sensor


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3,   // joystick deadband out of 127
                                    10,  // minimum output where drivetrain will move out of 127
                                    1.019); // expo curve gain


// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3,   // joystick deadband out of 127
                                 10,  // minimum output where drivetrain will move out of 127
                                 1.07); // expo curve gain


// create the chassis
lemlib::Chassis chassis(drivetrain,        // drivetrain settings
                       linearController,  // lateral PID settings
                       angularController, // angular PID settings
                       sensors);          // odometry sensors


pros::adi::Pneumatics Middle('e', false); // Pneumatics on port E
pros::adi::Pneumatics Descorer('a', false); // Pneumatics on port E
pros::adi::Pneumatics Grabber('c', false); // Pneumatics on port E

/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate();     // calibrate sensors
    imu.tare_heading();   
    chassis.setPose(0, 0, 90);
    // optional but recommended: turn on optical LED
    opticalSensor.set_led_pwm(100);

    pros::Task screenTask([&]() {
        while (true) {
            // pose
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);

            // optical
            //pros::lcd::print(3, "Hue: %d", (int)opticalSensor.get_hue());

            // distance sensors (mm, ints)
            //pros::lcd::print(4, "Dist L: %d", (int)leftSensor.get());
            //pros::lcd::print(5, "Dist R: %d", (int)rightSensor.get());
            //pros::lcd::print(6, "Dist F: %d", (int)forwardSensor.get());
            //pros::lcd::print(7, "Dist B: %d", (int)backSensor.get());

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}



/**
* Runs while the robot is disabled
*/
void disabled() {}


void competition_initialize() {}


// Helper Functions
int scaleInput(int input) {
   double scaled = std::pow(std::abs(input) / 127.0, 2) * 127.0;
   return input < 0 ? static_cast<int>(-scaled) : static_cast<int>(scaled);
}


// Spin intake forward for a duration (ms)
void spinIntakeMS(int duration) {
    intake1.move_velocity(-600);
    intake2.move_velocity(-600);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}

// Spin intake forward continuously
void spinIntake() {
    intake1.move_velocity(-600);
    intake2.move_velocity(-600);
}

// Stop all intake motors
void stopIntake() {
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}

void spinIntakeReverse() {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
}

// Spin intake reverse for a duration (ms)
void rejectIntakeMS(int duration) {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}

// Spin intake reverse continuously
void rejectIntake() {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
}

double inchesToCm(double inches) {
    return inches * 2.54;
}
double mmToInches(double mm) {
    return mm / 25.4;
}


// Set only the X coordinate (keep Y and heading)
void setPoseX(double measuredX) {
    double correctedX = measuredX + 4.5;    // shift to robot center
    lemlib::Pose pose = chassis.getPose();
    chassis.setPose(correctedX, pose.y, pose.theta);
}

void setPoseY(double measuredY) {
    double correctedY = measuredY + 4.5;    // shift to robot center
    lemlib::Pose pose = chassis.getPose();
    chassis.setPose(pose.x, correctedY, pose.theta);
}


// Set only the heading (keep X and Y)
void setPoseTheta(double newTheta) {
    lemlib::Pose pose = chassis.getPose();
    chassis.setPose(pose.x, pose.y, newTheta);
}

// === LEFT SENSOR ===
double getLeft(double offset = 4.5) {
    return mmToInches(leftSensor.get()) + offset;
}

// === RIGHT SENSOR ===
double getRight(double offset = 4.5) {
    return (mmToInches(rightSensor.get()) + offset);
}

// === FORWARD SENSOR ===
double getForward(double offset = 4.5) {
    return mmToInches(forwardSensor.get()) + offset;
}

// === BACK SENSOR ===
double getBack(double offset = 6) {
    return (mmToInches(backSensor.get()) + offset);
}


void spinChoice(const std::string& direction, int duration = 0) {
   int speed = 0;


   if (direction == "up") {
       speed = 600;
   } else if (direction == "down") {
       speed = -600;
   } else if (direction == "stop") {
       choice.move_velocity(0);
       return; // invalid direction
   }


   choice.move_velocity(speed);


   if (duration > 0) {
       pros::delay(duration);
       choice.move_velocity(0); // stop after the duration
   }
}
int slewRateLimit(int current, int target, int maxChange) {
    int difference = target - current;
    
    if (std::abs(difference) <= maxChange) {
        return target;  // We can reach the target in this cycle
    }
    
    // Move toward target by maxChange amount
    if (difference > 0) {
        return current + maxChange;
    } else {
        return current - maxChange;
    }
}
void stayInPosition(const std::string& sensor, int holdDurationMS, int toleranceMM = 10) {
    // Get the initial distance reading as the target
    int targetDistance = 0;
    
    if (sensor == "front") {
        targetDistance = forwardSensor.get();
    } else if (sensor == "left") {
        targetDistance = leftSensor.get();
    } else if (sensor == "right") {
        targetDistance = rightSensor.get();
    } else if (sensor == "back") {
        targetDistance = backSensor.get();
    } else {
        return; // invalid sensor
    }
    
    int elapsedTime = 0;
    int loopDelay = 20; // ms between checks
    int driveSpeed = 60; // speed for corrective movements (0-127)
    
    while (elapsedTime < holdDurationMS) {
        int currentDistance = 0;
        
        // Read current distance from the specified sensor
        if (sensor == "front") {
            currentDistance = forwardSensor.get();
        } else if (sensor == "left") {
            currentDistance = leftSensor.get();
        } else if (sensor == "right") {
            currentDistance = rightSensor.get();
        } else if (sensor == "back") {
            currentDistance = backSensor.get();
        }
        
        // Calculate difference from target
        int difference = currentDistance - targetDistance;
        
        if (std::abs(difference) > toleranceMM) {
            // Out of tolerance, make a correction
            if (difference > 0) {
                // Distance increased (moving away), drive forward to close gap
                leftMotors.move_velocity(driveSpeed);
                rightMotors.move_velocity(driveSpeed);
            } else {
                // Distance decreased (moving closer), drive backward to increase distance
                leftMotors.move_velocity(-driveSpeed);
                rightMotors.move_velocity(-driveSpeed);
            }
        } else {
            // Within tolerance, stop moving
            leftMotors.move_velocity(0);
            rightMotors.move_velocity(0);
        }
        
        pros::delay(loopDelay);
        elapsedTime += loopDelay;
    }
    
    // Stop motors when done
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

// === Autonomous ===
void autonomous() {
   // Read documentation for help: https://lemlib.readthedocs.io/en/stable/api/chassis.html
   // DO NOTTTTT DELETE ANY CODE I ALREADY PUT HERE.
// compute Y from left distance sensor (mm → inches + sensor→center offset

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(getRight(), getForward() - 72, 0);

    // Get block from other team
    spinIntake();
    chassis.moveToPoint(chassis.getPose().x, 0, 500, {.minSpeed = 80}); 
    
    // Align to right goal
    chassis.moveToPoint(chassis.getPose().x, -48, 1200, {.forwards = false, .minSpeed = 127});
    chassis.turnToHeading( 270, 700, {.minSpeed = 80});

    // Match load
    chassis.moveToPose(8, -48, 270, 700, { .minSpeed = 100});
    stayInPosition("front", 1000);

    // Score in right goal
    chassis.moveToPose(44, -48, 270, 1000, { .minSpeed = 127}, false);
    spinChoice("up");
    pros::delay(1500);
    chassis.setPose(getRight() - 72, getForward() , chassis.getPose().theta);
    spinChoice("stop");
    stopIntake();
    chassis.moveToPoint(24, -48, 1000, { .minSpeed = 100});

    // Pick up the right middle balls
    chassis.turnToHeading(45, 1200, {.minSpeed = 100});
    spinIntake();
    chassis.moveToPoint(48, -24, 2000, { .minSpeed = 100});
    chassis.waitUntil(35);
    Grabber.extend();
    chassis.turnToHeading(0, 500, {.minSpeed = 100}, false);

    // Pick up left middle balls
    chassis.moveToPoint(52, 24, 1500, { .minSpeed = 100});    
    chassis.waitUntil(24);
    Grabber.retract();
    chassis.waitUntil(26);
    Grabber.extend();

    // Align & Score for center goal
    chassis.turnToHeading(315, 1000, {.minSpeed = 100});
    chassis.moveToPose(61, 11, 315,1100, {.forwards = false});
    chassis.waitUntil(3);
    spinChoice("up");
    Middle.retract();
    pros::delay(500);
    spinChoice("stop");
    stopIntake();

    // Align to left goal
    chassis.moveToPoint(24, 48, 1200, {.minSpeed = 127, .earlyExitRange = 2});
    chassis.turnToHeading( 270, 700, {.minSpeed = 80});

    // left match load
    chassis.moveToPose(8, 48, 270, 700, { .minSpeed = 100});
    spinIntake();
    stayInPosition("front", 1000);

    // Score in left goal
    chassis.moveToPose(44, 48, 270, 1000, { .minSpeed = 127}, false);
    spinChoice("up");
    }


// === Driver control ===
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Toggle states
    bool grabberExtended = false;
    bool descorerExtended = false;
    bool middleExtended = true;   // starts extended (you said you're handling the actual extend elsewhere)

    // Edge-detect latches
    bool lastB = false;
    bool lastY = false;
    bool lastX = false;

    while (true) {
        // === DRIVE ===
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        bool slowdown = L1 && L2;

        leftY  = scaleInput(leftY);
        rightX = scaleInput(rightX);

        if (slowdown) {
            leftY  = static_cast<int>(leftY * 0.5);
            rightX = static_cast<int>(rightX * 0.5);
        }

        chassis.arcade(leftY, rightX);

        // === CHOICE (NO COLOR SORTER) ===
        // L1 or L2 spins choice "up" only
        int intakeCmd = 0; // final intake command for this loop

        if (L1 || L2) {
            choice.move_velocity(600); // always "up"
            intakeCmd = -600;          // optional: keep intake running while choice runs
        } else {
            choice.move_velocity(0);
        }

        // === R1/R2 intake control only if L1/L2 aren't held ===
        if (!L1 && !L2) {
            if (R1) {
                intakeCmd = -600; // intake in
            } else if (R2) {
                intakeCmd = 600;  // intake out
            } else {
                intakeCmd = 0;
            }
        }

        // apply intake command
        intake1.move_velocity(intakeCmd);
        intake2.move_velocity(intakeCmd);

        // === PNEUMATICS TOGGLES ===
        // Grabber toggle (B)
        bool bNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (bNow && !lastB) {
            grabberExtended = !grabberExtended;
            if (grabberExtended) Grabber.extend();
            else Grabber.retract();
        }
        lastB = bNow;

        // Descorer toggle (Y)
        bool yNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        if (yNow && !lastY) {
            descorerExtended = !descorerExtended;
            if (descorerExtended) Descorer.extend();
            else Descorer.retract();
        }
        lastY = yNow;

        // Middle goal toggle (X)
        bool xNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (xNow && !lastX) {
            middleExtended = !middleExtended;
            if (middleExtended) Middle.extend();
            else Middle.retract();
        }
        lastX = xNow;

        // === CONTROLLER DISPLAY (no flicker) ===
        static bool lastMiddleState = !middleExtended; // force first print

        if (middleExtended != lastMiddleState) {
            controller.print(0, 0, "Middle: %s   ",
                middleExtended ? "DOWN" : "UP");
            lastMiddleState = middleExtended;
        }

        pros::delay(10);
    }
}
