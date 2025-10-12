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

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ====================
// DRIVETRAIN HARDWARE
// ====================
pros::MotorGroup leftMotors({-15, -12},
                            pros::MotorGearset::blue); // left motor group - ports 1 and 2 (reversed)
pros::MotorGroup rightMotors({18, 14},
                             pros::MotorGearset::blue); // right motor group - ports 3 and 4 (unreversed)

// ================
// INTAKE (NEW)
// ================
pros::Motor intake1(1, pros::MotorGears::blue);
pros::Motor intake2(-2, pros::MotorGears::green);
pros::Motor choice(10, pros::MotorGears::green);
// =====================
// UNUSED SUBSYSTEMS (commented out but kept for reference)
// =====================
// pros::adi::Pneumatics MogoMech('h', true); // Pneumatics on port H
// pros::Motor highStake(7, pros::MotorGears::red);
// pros::Motor leftArm(20, pros::MotorGears::green);

// ================
// SENSORS (used for odom/drivetrain)
// ================
pros::Imu imu(16);                    // IMU
pros::Rotation verticalEnc(-17);  // Rotation sensor on port 8, reversed
pros::Rotation horizontalEnc(3);  // Rotation sensor on port 9, reversed
// Tracking wheel object (Vertical). 2" wheel, 0" offset (update if measured differently).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);

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
lemlib::ControllerSettings linearController(2,  // kP
                                            0,   // kI
                                            10, // kD
                                            0,   // anti windup
                                            0,   // small error (deg)
                                            0, // small error timeout (ms)
                                            0,   // large error (deg)
                                            0, // large error timeout (ms)
                                            0);  // max accel (slew)

// angular motion controller
lemlib::ControllerSettings angularController(8,  // kP
                                            0,   // kI
                                            32.5, // kD
                                            0,   // anti windup
                                            0,   // small error (deg)
                                            0, // small error timeout (ms)
                                            0,   // large error (deg)
                                            0, // large error timeout (ms)
                                            0);  // max accel (slew)

// sensors for odometry
lemlib::OdomSensors sensors(&vertical,  // vertical tracking wheel
                            nullptr,    // vertical tracking wheel 2
                            &horizontal,    // horizontal tracking wheel
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors
    chassis.setPose({0, 0, 0}); // set the robot's pose to 0, 0, 0

    chassis.turnToHeading(90, 100000); // turn to heading to calibrate IMU
    // --- Unused subsystems commented out ---
    // highStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // leftArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

void competition_initialize() {}

// === Driver input shaping for drivetrain (keep) ===
int scaleInput(int input) {
    double scaled = std::pow(std::abs(input) / 127.0, 2) * 127.0;
    return input < 0 ? static_cast<int>(-scaled) : static_cast<int>(scaled);
}


// === Autonomous ===
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.turnToHeading(90, 5000);
    // (No other actions; subsystems are commented out)
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            choice.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            choice.move_velocity(-200);
        } else {
            choice.move_velocity(0);
        }
        // === Intake control (R1 forward, R2 reverse) ===
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake1.move_velocity(600);
            intake2.move_velocity(174);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake1.move_velocity(-200);
            intake2.move_velocity(-80);
        } else {
            intake1.move_velocity(0);
            intake2.move_velocity(0);
        }

        pros::delay(10);
    }
}
