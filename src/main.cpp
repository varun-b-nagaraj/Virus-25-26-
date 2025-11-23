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


pros::adi::Pneumatics MogoMech('b', false); // Pneumatics on port E
pros::adi::Pneumatics Descorer('c', false); // Pneumatics on port E
pros::adi::Pneumatics Grabber('a', false); // Pneumatics on port E

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
    return (mmToInches(rightSensor.get()) + offset);
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
       speed = 600;
   } else if (direction == "down") {
       speed = -600;
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

    chassis.moveToPose(chassis.getPose().x,23,chassis.getPose().theta,1000);
    chassis.turnToHeading(90,1000);
    pros::delay(500);
    chassis.setPose(getBack(),72-getLeft(),chassis.getPose().theta);
    spinIntake();
    chassis.moveToPose(36,chassis.getPose().y,chassis.getPose().theta,1500, {.minSpeed = 60, .earlyExitRange = 4});
    pros::delay(500);
    chassis.moveToPose(50,chassis.getPose().y,chassis.getPose().theta,2500, {.maxSpeed = 40});
    pros::delay(500);
    // chassis.setPose(getBack(),72-getLeft(),chassis.getPose().theta);
    pros::delay(500);
    chassis.turnToHeading(-45,1500);
    pros::delay(500);
    chassis.moveToPose(24,50,chassis.getPose().theta,2500, {.maxSpeed = 100}); 
    chassis.turnToHeading(-92,1500);
    pros::delay(500);
    chassis.setPose(chassis.getPose().x,72-getRight(),chassis.getPose().theta);
    Grabber.set_value(true); // extend descorer
    chassis.moveToPose(32,chassis.getPose().y,chassis.getPose().theta,1500,{.forwards = false, .maxSpeed = 50});
    //chassis.setPose(chassis.getPose().x,72-getRight(),chassis.getPose().theta);
    // spinIntake(); //spins intake for 2 seconds to make sure ball is in
    pros::delay(500);                                                           
    spinChoice("up",4000); //spins choice motor up for 2 seconds to score in tall goal
    /*
    chassis.moveToPose(24,48,chassis.getPose().theta,2500); 
    orrrrrr 
    { Goes by the line
    chassis.turnToHeading(270,1500);
    chassis.moveToPose(24,24,chassis.getPose().theta,2500);
    chassis.setPose(getFront(),72-getRight(),chassis.getPose().theta);
    chassis.turnToHeading(0,1500);
    chassis.moveToPose(24,48,chassis.getPose().theta,2500);
    }
    chassis.turnToHeading(270,1500);
    chassis.setPose(getFront()+(lenght of tall goal thing),72-getRight(),chassis.getPose().theta);
    grabber.extend();
    chassis.moveToPose(14,chassis.getPose().y,chassis.getPose().theta,2500,{.maxSpeed = 50});
    spinIntakeMS(2000); //spins intake for 2 seconds to make sure ball is in
    spinIntake(); // keeps intake spinning to hold ball in
    chassis.moveToPose(24,48,chassis.getPose().theta,2500,{.forwards = false}); 
    grabber.retract();
    chassis.setPose(getFront()+(lenght of tall goal thing),72-getRight(),chassis.getPose().theta);
    chassis.moveToPose(37,48,chassis.getPose().theta,1500);
    chassis.setPose(getFront()+(lenght of tall goal thing),72-getRight(),chassis.getPose().theta);
    spinChoice("up",2000); //spins choice motor up for 2 seconds to score in tall goal
    pros::delay(5000);
    stopIntake(); // stops intake

    */

    //chassis.turnToHeading(-45,2500);
    //chassis.moveToPose(24,56,chassis.getPose().theta,2500);
    //chassis.setPose(chassis.getPose().x,72-getRight(),chassis.getPose().theta);
    /*
    pros::delay(1000); // wait for 1 second before starting autonomous
    spinIntake();
    chassis.moveToPose(0,27,0,9000, {.maxSpeed = 113}); // Move forward to intake rings
    chassis.turnToHeading(72.5,5000);
    pros::delay(10000);
    chassis.moveToPose(15,29,72.5,14000, {.maxSpeed = 113}); // Move forward to intake rings
    spinChoice("down", 2000); // Score lower goal
    */
    }


// === Driver control ===
// Helper Functions

// Scale input with quadratic curve

// === Driver control ===
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // State variables for smooth drive control
    static int currentLeftY = 0;
    static int currentRightX = 0;
    
    // Slew rate limits (adjust these values to tune smoothness)
    const int NORMAL_SLEW_RATE = 15;      // Normal acceleration/deceleration per loop (10ms)
    const int DIRECTION_CHANGE_SLEW_RATE = 8;  // Slower rate when changing directions
    
    // State for blue-flip logic
    static bool flippingBlue = false;
    static uint32_t flipStartTime = 0;
    static int flipDirection = 0;
    static bool lastSeesBlue = false;

    while (true) {
        // === DRIVE WITH SMOOTH TRANSITIONS ===
        
        // Read raw joystick values
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Apply scaling curve
        int targetLeftY = scaleInput(leftY);
        int targetRightX = scaleInput(rightX);

        // Determine if we're changing direction (sign change)
        bool leftYDirectionChange = (currentLeftY > 0 && targetLeftY < 0) || 
                                    (currentLeftY < 0 && targetLeftY > 0);
        bool rightXDirectionChange = (currentRightX > 0 && targetRightX < 0) || 
                                     (currentRightX < 0 && targetRightX > 0);

        // Choose slew rate based on whether we're changing direction
        int leftYSlewRate = leftYDirectionChange ? DIRECTION_CHANGE_SLEW_RATE : NORMAL_SLEW_RATE;
        int rightXSlewRate = rightXDirectionChange ? DIRECTION_CHANGE_SLEW_RATE : NORMAL_SLEW_RATE;

        // Apply slew rate limiting
        currentLeftY = slewRateLimit(currentLeftY, targetLeftY, leftYSlewRate);
        currentRightX = slewRateLimit(currentRightX, targetRightX, rightXSlewRate);

        // Check for slowdown mode BEFORE applying to chassis
        bool choiceUp   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool choiceDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool slowdown = choiceUp && choiceDown;

        // Apply slowdown multiplier if active
        int finalLeftY = currentLeftY;
        int finalRightX = currentRightX;
        if (slowdown) {
            finalLeftY = static_cast<int>(currentLeftY * 0.5);
            finalRightX = static_cast<int>(currentRightX * 0.5);
        }

        // Send to chassis
        chassis.arcade(finalLeftY, finalRightX);

        // ============================
        // CHOICE MOTOR WITH BLUE-FLIP
        // ============================
        
        int driverChoiceCmd = 0;
        if (choiceUp) {
            driverChoiceCmd = 600;
        } else if (choiceDown) {
            driverChoiceCmd = -600;
        } else {
            driverChoiceCmd = 0;
        }

        // Optical sensor read
        int hue = opticalSensor.get_hue();
        bool seesBlue = (hue > 160 && hue < 260);
        uint32_t now = pros::millis();

        // Start a flip on new blue detection
        if (!flippingBlue && !lastSeesBlue && seesBlue && driverChoiceCmd != 0) {
            flippingBlue = true;
            flipStartTime = now;
            flipDirection = (driverChoiceCmd > 0 ? -1 : 1);
        }

        // Determine final choice command
        int choiceCmd;
        if (flippingBlue) {
            const int FLIP_DELAY_MS    = 50;
            const int FLIP_DURATION_MS = 5000;

            uint32_t dt = now - flipStartTime;

            if (dt >= FLIP_DELAY_MS && dt < FLIP_DELAY_MS + FLIP_DURATION_MS) {
                choiceCmd = flipDirection * 600;
            } else if (dt >= FLIP_DELAY_MS + FLIP_DURATION_MS) {
                flippingBlue = false;
                choiceCmd = driverChoiceCmd;
            } else {
                choiceCmd = driverChoiceCmd;
            }
        } else {
            choiceCmd = driverChoiceCmd;
        }

        lastSeesBlue = seesBlue;
        choice.move_velocity(choiceCmd);

        // ============================
        // INTAKE CONTROL
        // ============================
        int intakeCmd = 0;

        bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (choiceUp || choiceDown || intakeForwardButton) {
            intakeCmd = -600;
        } else if (intakeReverseButton) {
            intakeCmd = 600;
        } else {
            intakeCmd = 0;
        }

        intake1.move_velocity(intakeCmd);
        intake2.move_velocity(intakeCmd);
        intake3.move_velocity(intakeCmd/3);

        // ============================
        // PNEUMATICS
        // ============================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            MogoMech.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            MogoMech.retract();
        }

        pros::delay(10);
    }
}