#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/motor_group.hpp"
#include <cmath> // for pow/abs

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-1, -2},
                            pros::MotorGearset::blue); // left motor group - ports 1 and 2 (reversed)
pros::MotorGroup rightMotors({3, 4},
                             pros::MotorGearset::blue); // right motor group - ports 3 and 4 (unreversed)

// pneumatics
pros::adi::Pneumatics MogoMech('h', true); // Pneumatics on port H

// Define Intake Motors
pros::Motor intake1(5, pros::MotorGears::blue);
pros::Motor intake2(6, pros::MotorGears::green);

// Define HighStake Motor
pros::Motor highStake(7, pros::MotorGears::red);

// Arm
pros::Motor leftArm(20, pros::MotorGears::green);

// Inertial Sensor on port 18
pros::Imu imu(18);

// Tracking wheel encoder (Vertical). Rotation sensor on port 8, reversed.
pros::Rotation verticalEnc(-8);

// Tracking wheel object (Vertical). 2" wheel, 0" offset (update if you measure differently).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                              &rightMotors,                // right motor group
                              15,                          // track width (inches)
                              lemlib::Omniwheel::NEW_325,  // using 3.25" omnis
                              450,                         // drivetrain rpm
                              8);                          // horizontal drift (unitless tuning)

// lateral motion controller
lemlib::ControllerSettings linearController(14,  // kP
                                            0,   // kI
                                            108, // kD
                                            3,   // anti windup
                                            1,   // small error (deg)
                                            100, // small error timeout (ms)
                                            3,   // large error (deg)
                                            500, // large error timeout (ms)
                                            0);  // max accel (slew)

// angular motion controller
lemlib::ControllerSettings angularController(10,  // kP
                                             0,   // kI
                                             108, // kD
                                             3,   // anti windup
                                             1,   // small error (deg)
                                             100, // small error timeout (ms)
                                             3,   // large error (deg)
                                             500, // large error timeout (ms)
                                             0);  // max accel (slew)

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

    leftArm.set_zero_position(0); // set the zero position for the left arm
    highStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // hold for highStake motor
    leftArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   // hold for leftArm

    // thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);       // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);       // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "HighStake Rot: %f", highStake.get_position()); // highStake rotation
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

// === Helpers ===
void spinIntakeMS(int duration) {
    intake1.move_velocity(600);
    intake2.move_velocity(174);
    pros::delay(duration); // Wait for the specified duration
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}
void spinIntake() {
    intake1.move_velocity(600);
    intake2.move_velocity(174);
}
void stopIntake() {
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}

// === Autonomous ===
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    highStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// === Driver control ===
int scaleInput(int input) {
    double scaled = std::pow(std::abs(input) / 127.0, 2) * 127.0;
    return input < 0 ? static_cast<int>(-scaled) : static_cast<int>(scaled);
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    highStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Check for slowdown
        bool slowdown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
                        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        // Apply input shaping
        leftY = scaleInput(leftY);

        // slowdown scaling
        if (slowdown) {
            leftY = static_cast<int>(leftY * 0.5);
            rightX = static_cast<int>(rightX * 0.5);
        }

        // drive
        chassis.arcade(leftY, rightX);

        // intake control
        if (slowdown) {
            intake1.move_velocity(0);
            intake2.move_velocity(0);
        } else {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake1.move_velocity(600);
                intake2.move_velocity(174);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intake1.move_velocity(-200);
                intake2.move_velocity(-80);
            } else {
                intake1.move_velocity(0);
                intake2.move_velocity(0);
            }
        }

        // high stake control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            highStake.move_velocity(40);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            highStake.move_velocity(-100);
        } else {
            highStake.move_velocity(0);
        }

        // arm control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            leftArm.move_absolute(220, 100); // bring arm down
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            leftArm.move_absolute(0, 100);
        }

        // mogo mech control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            MogoMech.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            MogoMech.retract();
        }

        // delay to save resources
        pros::delay(10);
    }
}
