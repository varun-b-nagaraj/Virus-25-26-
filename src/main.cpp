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
pros::Motor intake3(8, pros::MotorGears::blue);

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
pros::Distance backSensor(19);

pros::Optical opticalSensor(2);// ================
// SENSORS (used for odom/drivetrain)
// ================
pros::Imu imu(12);                    // IMU
pros::Rotation verticalEnc(11);  // Rotation sensor on port 8, reversed
// pros::Rotation horizontalEnc(3);  // Rotation sensor on port 9, reversed
// Tracking wheel object (Vertical). 2" wheel, 0" offset (update if measured differently).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 1.5);
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
lemlib::ControllerSettings linearController(15,  // kP
                                       .9,   // kI
                                           65, // kD
                                           2,   // anti windup
                                           1,   // small error (deg)
                                           100, // small error timeout (ms)
                                           3,   // large error (deg)
                                           500, // large error timeout (ms)
                                           0);  // max accel (slew)


// angular motion controller
lemlib::ControllerSettings angularController(8,  // kP
                                           0,   // kI old - -0.0042257
                                           50, // kD
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


pros::adi::Pneumatics MogoMech('e', false); // Pneumatics on port E
/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate();     // calibrate sensors

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
            pros::lcd::print(4, "Proximity: %d", (int)opticalSensor.get_proximity());

            // distance sensors (mm, ints)
            pros::lcd::print(5, "Dist L: %d", (int)leftSensor.get());
            pros::lcd::print(6, "Dist R: %d", (int)rightSensor.get());
            pros::lcd::print(7, "Dist F: %d", (int)forwardSensor.get());
            // if you want back sensor too, temporarily swap it in:
            // pros::lcd::print(7, "Dist B: %d", (int)backSensor.get());

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
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    intake3.move_velocity(600);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake forward continuously
void spinIntake() {
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    intake3.move_velocity(600);
}

// Stop all intake motors
void stopIntake() {
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake reverse for a duration (ms)
void rejectIntakeMS(int duration) {
    intake1.move_velocity(-600);
    intake2.move_velocity(-600);
    intake3.move_velocity(-600);
    pros::delay(duration);
    intake1.move_velocity(0);
    intake2.move_velocity(0);
    intake3.move_velocity(0);
}

// Spin intake reverse continuously
void rejectIntake() {
    intake1.move_velocity(-600);
    intake2.move_velocity(-600);
    intake3.move_velocity(-600);
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

    pros::delay(1000); // wait for 1 second before starting autonomous
    spinIntake();
    chassis.moveToPose(0,27,0,9000, {.maxSpeed = 113}); // Move forward to intake rings
    chassis.turnToHeading(72.5,5000);
    pros::delay(10000);
    chassis.moveToPose(15,29,72.5,14000, {.maxSpeed = 113}); // Move forward to intake rings
    spinChoice("down", 2000); // Score lower goal



   // // Use this to turn(positive is to the right and negative is to the left) and the timeout is in milliseconds
   // // and specifies how long the robot has to complete the action
   //chassis.turnToHeading(90, 5000);
   // // Use this to move to a specific x, y coordinate (inches) and heading (degrees). The code below moves the robot 48 inches forward
   // // 1 tile is 24 inches. The timeout is in milliseconds and specifies how long the robot has to complete the action
   // // The third number is the heading you want the robot to be at once it reaches the point. 0 degrees is facing "forward"
   // spinIntake();
   // chassis.moveToPose(0, 38, 0, 2000);


   // chassis.turnToHeading(35, 5000);
   // chassis.moveToPose(10, 45, 35, 5000);
   // pros::delay(1000);
   // spinChoice("down");
  
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
  
   //  Test routine below
   // // === 1. DRIVE TEST ===
   // pros::lcd::print(5, "Step 1: Driving forward & backward...");
   // chassis.moveToPose(0, 24, 0, 3000);  // forward 24 inches
   // pros::delay(500);
   // chassis.moveToPose(0, 0, 0, 3000);   // back to start
   // pros::delay(1000);


   // // === 2. TURN TEST ===
   // pros::lcd::print(5, "Step 2: Turning...");
   // chassis.turnToHeading(90, 3000);     // turn 90° right
   // pros::delay(500);
   // chassis.turnToHeading(0, 3000);      // return to 0°
   // pros::delay(1000);


   // // === 3. INTAKE TEST ===
   // pros::lcd::print(5, "Step 3: Testing intake...");
   // spinIntake(1500, true);              // forward full speed for 1.5s
   // pros::delay(500);
   // spinIntake(1500, false);             // reverse slower for 1.5s
   // pros::delay(1000);


   // // === 4. CHOICE MOTOR TEST ===
   // pros::lcd::print(5, "Step 4: Testing choice motor...");
   // spinChoice("up", 1000);              // spin upward 1s
   // pros::delay(500);
   // spinChoice("down", 1000);            // spin downward 1s
   // pros::delay(500);
   // spinChoice("stop");                  // ensure stop
   // pros::delay(1000);


   // // === 5. DRIVE + INTAKE COMBINATION TEST ===
   // pros::lcd::print(5, "Step 5: Combined movement...");
   // spinIntake();                        // run intake continuously
   // chassis.moveToPose(0, 24, 0, 3000);  // move forward while intaking
   // stopIntake();                        // stop intake after motion
   // pros::delay(1000);


   // // === COMPLETE ===
   // pros::lcd::print(5, "Autonomous test complete!");


}


// === Driver control ===
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // state for blue-flip logic (persist across loop)
    static bool flippingBlue = false;
    static uint32_t flipStartTime = 0;
    static int flipDirection = 0;      // +1 or -1
    static bool lastSeesBlue = false;

    while (true) {
        // === DRIVE ===
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // slowdown combo: L2 + L1
        bool slowdown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
                        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        leftY = scaleInput(leftY);
        rightX = scaleInput(rightX);

        if (slowdown) {
            leftY = static_cast<int>(leftY * 0.5);
            rightX = static_cast<int>(rightX * 0.5);
        }

        chassis.arcade(leftY, rightX);

        // ============================
        // CHOICE MOTOR WITH BLUE-FLIP
        // ============================
        int choiceCmd = 0;

        bool choiceUp   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2); // score high
        bool choiceDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); // score low

        if (choiceUp) {
            choiceCmd = 600;      // spin up
        } else if (choiceDown) {
            choiceCmd = -600;     // spin down
        } else {
            choiceCmd = 0;
        }

        // Optical sensor read
        int hue = opticalSensor.get_hue();

        // basic blue detection (tune as needed)
        bool seesBlue = (hue > 160 && hue < 260);

        uint32_t now = pros::millis();

        // Start a flip on a new blue detection while spinning (either direction)
        if (!flippingBlue && !lastSeesBlue && seesBlue && choiceCmd != 0) {
            flippingBlue = true;
            flipStartTime = now;
            // flip opposite of current direction
            flipDirection = (choiceCmd > 0 ? -1 : 1);
        }

        // Handle flip timing (delay + eject window)
        if (flippingBlue) {
            const int FLIP_DELAY_MS    = 50;   // wait a bit after seeing blue
            const int FLIP_DURATION_MS = 500;  // how long to eject

            uint32_t dt = now - flipStartTime;

            if (dt >= FLIP_DELAY_MS && dt < FLIP_DELAY_MS + FLIP_DURATION_MS) {
                // in eject window: force opposite direction
                choiceCmd = flipDirection * 600;
            } else if (dt >= FLIP_DELAY_MS + FLIP_DURATION_MS) {
                // done flipping; allow new detections
                flippingBlue = false;
                // after this, choiceCmd is whatever L1/L2 says this loop
            }
        }

        lastSeesBlue = seesBlue;

        // Apply choice motor command
        choice.move_velocity(choiceCmd);

        // ============================
        // INTAKE CONTROL
        // ============================
        int intakeCmd = 0;

        bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        // When using the choice motor (up or down), always push rings forward
        if (choiceUp || choiceDown || intakeForwardButton) {
            intakeCmd = -600;          // forward
        } else if (intakeReverseButton) {
            intakeCmd = 600;         // reverse
        } else {
            intakeCmd = 0;            // stop
        }

        intake1.move_velocity(intakeCmd);
        intake2.move_velocity(intakeCmd);
        intake3.move_velocity(intakeCmd);

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


