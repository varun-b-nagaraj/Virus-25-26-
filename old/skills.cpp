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
pros::MotorGroup rightMotors({9, 10},
                            pros::MotorGearset::blue); // right motor group - ports 3 and 4 (unreversed)


// ================
// INTAKE (NEW)
// ================
pros::Motor intake1(-20, pros::MotorGears::blue);
pros::Motor intake2(15, pros::MotorGears::blue);
pros::Motor intake3(8, pros::MotorGears::green);

pros::Motor choice(6, pros::MotorGears::blue);

// =====================
// UNUSED SUBSYSTEMS (commented out but kept for reference)
// =====================
// pros::adi::Pneumatics MogoMech('h', true); // Pneumatics on port H
// pros::Motor highStake(7, pros::MotorGears::red);
// pros::Motor leftArm(20, pros::MotorGears::green);
pros::Distance leftSensor(13);
pros::Distance rightSensor(1);
pros::Distance forwardSensor(14);
pros::Distance backSensor(5);

pros::Optical opticalSensor(2);// ================
// SENSORS (used for odom/drivetrain)
// ================
pros::Imu imu(12);                    // IMU
pros::Rotation verticalEnc(-11);  // Rotation sensor on port 8, reversed
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
lemlib::ControllerSettings linearController(18.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              8.61, // derivative gain (kD)
                                              0, // anti windup
                                              .5, // small error range, in inches
                                              700, // small error range timeout, in milliseconds
                                              4, // large error range, in inches
                                              2000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// angular motion controller
lemlib::ControllerSettings angularController(3.36, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              0, // anti windup
                                              .5, // small error range, in inches
                                              700, // small error range timeout, in milliseconds
                                              4, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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


pros::adi::Pneumatics MogoMech('e', false); // Pneumatics on port E
pros::adi::Pneumatics Descorer('e', false); // Pneumatics on port E
pros::adi::Pneumatics Grabber('e', false); // Pneumatics on port E

/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate();     // calibrate sensors
    imu.set_heading(270);

    // optional but recommended: turn on optical LED
    opticalSensor.set_led_pwm(100);

    pros::Task screenTask([&]() {
        while (true) {
            // pose
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);

            // optical
            pros::lcd::print(3, "Hue: %d", (int)opticalSensor.get_hue());

            // distance sensors (mm, ints)
            pros::lcd::print(4, "Dist L: %d", (int)leftSensor.get());
            pros::lcd::print(5, "Dist R: %d", (int)rightSensor.get());
            pros::lcd::print(6, "Dist F: %d", (int)forwardSensor.get());
            pros::lcd::print(7, "Dist B: %d", (int)backSensor.get());

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
    intake3.move_velocity(-200);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake forward continuously
void spinIntake() {
    intake1.move_velocity(-600);
    intake2.move_velocity(-600);
    intake3.move_velocity(-200);
}

// Stop all intake motors
void stopIntake() {
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake reverse for a duration (ms)
void rejectIntakeMS(int duration) {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    intake3.move_velocity(200);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake reverse continuously
void rejectIntake() {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    intake3.move_velocity(200);
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
    return mmToInches(rightSensor.get()) + offset;
}

// === FORWARD SENSOR ===
double getForward(double offset = 4.5) {
    return mmToInches(forwardSensor.get()) + offset;
}

// === BACK SENSOR ===
double getBack(double offset = 5.75) {
    return (mmToInches(backSensor.get()) + offset);
}


void spinChoice(const std::string& direction, int duration = 0) {
   int speed = 0;


   if (direction == "up") {
       speed = 200;
   } else if (direction == "down") {
       speed = -200;
   } else {
       choice.move_velocity(0);
       return; // invalid direction
   }


   choice.move_velocity(speed);


   if (duration > 0) {
       pros::delay(duration);
       choice.move_velocity(0); // stop after the duration
   }
}


// === Autonomous ===
void autonomous() {
   // Read documentation for help: https://lemlib.readthedocs.io/en/stable/api/chassis.html
   // DO NOTTTTT DELETE ANY CODE I ALREADY PUT HERE.
    // compute Y from left distance sensor (mm → inches + sensor→center offset)
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    double fieldX = mmToInches(leftSensor.get()) + 4.5;

    // grab whatever heading the IMU/odom thinks we have (should be ~270)
    double theta = chassis.getPose().theta;

    // set pose with the measured X/Y, keep heading consistent with IMU
    chassis.setPose(fieldX, 15.25, theta);

    // Starting pose  
// chassis.setPose(0, 0, 0); - True Origin

// Path points
chassis.setPose(-11, 24, 0, 5000); // - starting point

chassis.moveToPose(-11, 38, 90, 5000);

chassis.turnToPoint(-24,48,5000);

chassis.moveToPose(-24, 48, chassis.getPose().theta, 5000);

chassis.turnToHeading(45,5000);

chassis.moveToPose(-48, 29, chassis.getPose().theta, 5000, {.forwards = false});

chassis.turnToPoint(-48,15.5, 5000);

chassis.setPose(getRight()-72,getForward()+5,chassis.getPose().theta);

// put pneumatic down
// spin intake

// -31.5 stays
chassis.moveToPose(-48, 15.5, chassis.getPose().theta, 5000);

chassis.setPose(getRight()-72,getForward()+5,chassis.getPose().theta);

// -56.5 stays
chassis.moveToPose(-48, 40.5, chassis.getPose().theta, 5000, {.forwards = false});

// score up

chassis.moveToPose(40, 48, 90, 5000);

// 33.5 stays
chassis.moveToPose(33.5, 48, 90, 5000);

chassis.moveToPose(57, 48, 90, 5000);

chassis.moveToPose(33, 48, 195, 5000);

// keep decimals from Jerry export
chassis.moveToPose(23.985, -24.0806, 135, 5000);

chassis.moveToPose(48, -48, 90, 5000);

chassis.moveToPose(33, -48, 90, 5000);

chassis.moveToPose(57, -48, 90, 5000);

chassis.moveToPose(48, -48, 315, 5000);

chassis.moveToPose(-50, -48, 270, 5000);

// -56.5 stays
chassis.moveToPose(-56.5, -48, 270, 5000);

// -31.5 stays
chassis.moveToPose(-31.5, -48, 270, 5000);

// keep -7 exactly as file has
chassis.moveToPose(-64, -7, 0, 5000);


    }


// === Driver control ===
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (true) {
        // joystick
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // slowdown combo (kept): L2 + L1
        bool slowdown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
                        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // shape drive input
        leftY = scaleInput(leftY);
        rightX = scaleInput(rightX);

        if (slowdown) {
            leftY = static_cast<int>(leftY * 0.5);
            rightX = static_cast<int>(rightX * 0.5);
        }

        // drive
        chassis.arcade(leftY, rightX);
        // Up down deciding
        bool blueDetect = opticalSensor.get_hue() > 160 && opticalSensor.get_hue() < 260;
        int inverse = 1;

        if (blueDetect) {
            inverse = -1;
        } else {
            inverse = 1;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            choice.move_velocity(inverse * -600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            choice.move_velocity(inverse * 600);
        } else {
            choice.move_velocity(0);
        }
        // === Intake control (R1 forward, R2 reverse) ===
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake1.move_velocity(-600);
            intake2.move_velocity(-600);
            intake3.move_velocity(-200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake1.move_velocity(600);
            intake2.move_velocity(600);
            intake3.move_velocity(200);
        } else {
            intake1.move_velocity(0);
            intake2.move_velocity(0);
            intake3.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            Grabber.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            Grabber.retract();
        }

        pros::delay(10);
    }
}