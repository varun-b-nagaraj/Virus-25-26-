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
pros::Imu imu(16);                      // IMU
pros::Rotation verticalEnc(-17);       // Rotation sensor on port 8, reversed
pros::Rotation horizontalEnc(3);       // Rotation sensor on port 9, reversed
// Tracking wheel object (Vertical). 2" wheel, 0" offset (update if measured differently).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 1.5);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);

// ====================
// DRIVETRAIN SETTINGS
// ====================
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15, // track width (inches)
                              lemlib::Omniwheel::NEW_325, // using 3.25" omnis
                              360, // drivetrain rpm
                              8); // horizontal drift (unitless tuning)

// lateral motion controller
lemlib::ControllerSettings linearController(15, // kP
                                            .9, // kI
                                            65, // kD
                                            2, // anti windup
                                            1, // small error (deg)
                                            100, // small error timeout (ms)
                                            3, // large error (deg)
                                            500, // large error timeout (ms)
                                            120); // max accel (slew)

// angular motion controller
lemlib::ControllerSettings angularController(8, // kP
                                            0, // kI old - -0.0042257
                                            50, // kD
                                            3, // anti windup
                                            1, // small error (deg)
                                            100, // small error timeout (ms)
                                            3, // large error (deg)
                                            500, // large error timeout (ms)
                                            0); // max accel (slew)

// sensors for odometry
lemlib::OdomSensors sensors(&vertical,  // vertical tracking wheel
                            nullptr,    // vertical tracking wheel 2
                            &horizontal,    // horizontal tracking wheel
                            nullptr,    // horizontal tracking wheel 2
                            &imu);      // inertial sensor

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3,// joystick deadband out of 127
                                     10,  // minimum output where drivetrain will move out of 127
                                     1.019); // expo curve gain

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3,   // joystick deadband out of 127
                                  10,  // minimum output where drivetrain will move out of 127
                                  1.07); // expo curve gain

// create the chassis
lemlib::Chassis chassis(drivetrain,        // drivetrain settings
                        linearController,  // lateral PID settings
                        angularController, // angular PID settings
                        sensors);          // odometry sensors

pros::adi::Pneumatics MogoMech('e', false); // Pneumatics on port E
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    
    chassis.calibrate();     // calibrate sensors
    // chassis.setPose({0, 0, 0}); // set the robot's pose to 0, 0, 0

    // chassis.turnToHeading(90, 100000); // turn to heading to calibrate IMU
    // --- Unused subsystems commented out ---
    // highStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // leftArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
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

// Helper Functions
int scaleInput(int input) {
    double scaled = std::pow(std::abs(input) / 127.0, 2) * 127.0;
    return input < 0 ? static_cast<int>(-scaled) : static_cast<int>(scaled);
}

// 1 to spin forward, -1 to spin reverse, 0 to stop
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

void rejectIntakeMS(int duration) {
    intake1.move_velocity(-200);
    intake2.move_velocity(-80);
    pros::delay(duration); // Wait for the specified duration
    intake1.move_velocity(0);
    intake2.move_velocity(0);
}

void rejectIntake() {
    intake1.move_velocity(-200);
    intake2.move_velocity(-80);
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
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    
    // // Use this to turn(positive is to the right and negative is to the left) and the timeout is in milliseconds
    // // and specifies how long the robot has to complete the action
    //chassis.turnToHeading(90, 5000);
    // // Use this to move to a specific x, y coordinate (inches) and heading (degrees). The code below moves the robot 48 inches forward
    // // 1 tile is 24 inches. The timeout is in milliseconds and specifies how long the robot has to complete the action
    // // The third number is the heading you want the robot to be at once it reaches the point. 0 degrees is facing "forward"
    intake1.move_velocity(600);
    intake2.move_velocity(174);
    chassis.moveToPose(0, 8, 0, 2000);
    stopIntake();
    chassis.turnToHeading(85, 5000);
    chassis.moveToPose(0, 8, 85, 2000);
    spinChoice("down");
    spinChoice("stop");
    // // Spin the main intake/storage forward(Will spin forever) -- You can also use negative numbers to spin it backwards.
            // intake1.move_velocity(600); // Spin intake 1 forward
            // intake2.move_velocity(174); // Spin intake 2 forward (slower motor or different gear ratio)
    // // You can also use the spinIntake() function I made above to do the same thing
            // spinIntake();
            // spinIntake(2000); // Spin intake for 2 seconds
            // spinIntake(2000, false); // Spin intake in reverse for 2 seconds

    // //When you want to stop just set the velocity to 0
            // intake1.move_velocity(0);
            // intake2.move_velocity(0);
    // // You can also use the stopIntake() function I made above to do the same thing
            // stopIntake();

    // // To use the top motor to make your choice of the higher or lower goal use the choice motor.
            // choice.move_velocity(200); // Spin choice motor to score to the higher scoring element
            // choice.move_velocity(-200); // Spin choice motor to score to the lower scoring element
            // choice.move_velocity(0); // Stop the choice motor
    // // You can also use the spinChoice() function I made above to do the same thing
            // spinChoice("up"); // Spin choice motor to score to the higher scoring element
            // spinChoice("down"); // Spin choice motor to score to the lower scoring element
            // spinChoice("up", 1000); // You can also add an optional timeout(ms) - Spin choice motor up for 1 second
            // spinChoice("stop"); // Stop the choice motor

    // // Chaining too much motion or moveToPose commands might create innacuracies, between commands a small timeout allows 
    // // the robot to settle and alleviates any innacuracies
            // pros::delay(1000); // wait for 1 second

    // // Pneumatics usage example. First code extends the pnuematic the second one shrinks it back.
    // // Limit usuage as much as possible since air tank depletes and we can't shove it in like we do for driver controller
    // // So low air pressure is worse in autonomous.
            // MogoMech.extend();
            // MogoMech.retract();
    
    //  Test routine below
    // // === 1. DRIVE TEST ===
    // pros::lcd::print(5, "Step 1: Driving forward & backward...");
    // chassis.moveToPose(0, 24, 0, 3000);  // forward 24 inches
    // pros::delay(500);
    // chassis.moveToPose(0, 0, 0, 3000);   // back to start
    // pros::delay(1000);

    // // === 2. TURN TEST ===
    // pros::lcd::print(5, "Step 2: Turning...");
    // chassis.turnToHeading(90, 3000);     // turn 90° right
    // pros::delay(500);
    // chassis.turnToHeading(0, 3000);      // return to 0°
    // pros::delay(1000);

    // // === 3. INTAKE TEST ===
    // pros::lcd::print(5, "Step 3: Testing intake...");
    // spinIntake(1500, true);              // forward full speed for 1.5s
    // pros::delay(500);
    // spinIntake(1500, false);             // reverse slower for 1.5s
    // pros::delay(1000);

    // // === 4. CHOICE MOTOR TEST ===
    // pros::lcd::print(5, "Step 4: Testing choice motor...");
    // spinChoice("up", 1000);              // spin upward 1s
    // pros::delay(500);
    // spinChoice("down", 1000);            // spin downward 1s
    // pros::delay(500);
    // spinChoice("stop");                  // ensure stop
    // pros::delay(1000);

    // // === 5. DRIVE + INTAKE COMBINATION TEST ===
    // pros::lcd::print(5, "Step 5: Combined movement...");
    // spinIntake();                        // run intake continuously
    // chassis.moveToPose(0, 24, 0, 3000);  // move forward while intaking
    // stopIntake();                        // stop intake after motion
    // pros::delay(1000);

    // // === COMPLETE ===
    // pros::lcd::print(5, "Autonomous test complete!");

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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            choice.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            MogoMech.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            MogoMech.retract();
        }

        pros::delay(10);
    }
}