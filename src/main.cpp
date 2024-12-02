#include "main.h"

// Drivetrain config

// Controller
pros::Controller controller(CONTROLLER_MASTER);

// Create motor group with two motors
pros::MotorGroup left_motor_group({2}, pros::v5::MotorGears::green);
pros::MotorGroup right_motor_group({-1}, pros::v5::MotorGears::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,
                              &right_motor_group,
                              11.125, // track width
                              lemlib::Omniwheel::OLD_4, // wheel type
                              200, // rpm
                              2 // horizontal drift
);

// create sensors
// imu
pros::Imu imu(12);
// horizontal tracking wheel encoder
pros::adi::Encoder horizontal_encoder('A', 'B');
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D');
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_4, -10.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::OLD_4, -5.625);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9.8, // proportional gain (kP) 9.8
                                              0, // integral gain (kI)
                                              1.6, // derivative gain (kD) 1.6
                                              0, // anti windup
                                              0.05, // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              0.1, // large error range, in inches
                                              1500, // large error range timeout, in milliseconds
                                              0.8 // maximum acceleration (slew) 0.8
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.3, // proportional gain (kP) 0.9 1.8 1.4
                                              0, // integral gain (kI)
                                              1.4, // derivative gain (kD) 8 1.1 8.625
                                              0, // anti windup 13.65
                                              0.05, // small error range, in degrees
                                              300, // small error range timeout, in milliseconds
                                              0.1, // large error range, in degrees
                                              1500, // large error range timeout, in milliseconds
                                              0.6 // maximum acceleration (slew) 0.8
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(5, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(1, // joystick deadband out of 127
                                  1, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve,
						&steer_curve

);

// Arm config
pros::Motor arm_motor(5, pros::v5::MotorGears::red);
pros::ADIEncoder arm_encoder (5, 6, false);
pros::ADIDigitalIn arm_limit(7);

// Set bottom position to 0
void arm_calibrate() {
	while (!arm_limit.get_value()) {
		arm_motor.move(-63);
	}

	arm_motor.brake();
	arm_encoder.reset();
}

// PID class for arm
lemlib::PID arm_controller(5,  // kP
						   0,  // kI
						   0,  // kD
						   0,  // integral anti windup range
						   false  // don't reset integral when sign of error flips
);

// Variable to track the current limited power
double currentPower = 0;

void arm_moveToAngle(double angle, double maxSpeed) {
	double error = arm_encoder.get_value() - angle;
	double power = arm_controller.update(error);

	// Apply the slew rate limiter to the power
	double limitedPower = lemlib::slew(power, currentPower, 5); //set maxAccel as slew rate

	// Maximum speed limit
	if (fabs(limitedPower) > maxSpeed) {
		limitedPower = maxSpeed;
	}

	// Update the current power
	currentPower = limitedPower;

	// Move the arm motor
	arm_motor.move(limitedPower);
}

// Arm scrolling
// Predefined arm positions
double arm_positions[] = {0.0, 10, 20, 30};
int position_count = sizeof(arm_positions) / sizeof(arm_positions[0]);

void arm_scroll(double maxSpeed) {
	static int i = 0; // Current selected position

	// Check button states and update position index
	if (controller.get_digital(DIGITAL_L1) && i < position_count - 1) {
		// Increment index only if not at the last position
		i++;
	} else if (controller.get_digital(DIGITAL_L2) && i > 0) {
		// Decrement index only if not at the first position
		i--;
	}

	// Move to the selected position
	arm_moveToAngle(arm_positions[i], maxSpeed);
}

// Create conveyor motor group
pros::MotorGroup intake_motors({6, -7}, pros::v5::MotorGears::blue);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::print(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

ASSET(start_goal_txt);
ASSET(bb1_txt);
ASSET(bb1_reverse_txt);
ASSET(bb2_txt);
ASSET(bb2_reverse_txt);
ASSET(bb3_txt);
ASSET(bb_retrieve_txt);
ASSET(bb_score_txt);

void autonomous() { //turn maxSpeed = 50  drive = 60
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
	arm_calibrate();
	arm_moveToAngle(10, 127);
	
    // follow path
    chassis.follow(start_goal_txt, 15, 10000, false);
	pros::delay(100);
	intake_motors.move(127);
    chassis.follow(bb1_txt, 15, 10000, false);
	pros::delay(300);
	intake_motors.brake();
    chassis.follow(bb1_reverse_txt, 15, 10000, true);
	intake_motors.move(127);
    chassis.follow(bb2_txt, 15, 10000, false);
	pros::delay(300);
	intake_motors.brake();
    chassis.follow(bb2_reverse_txt, 15, 10000, true);
	intake_motors.move(127);
    chassis.follow(bb3_txt, 15, 10000, false);
	pros::delay(300);
	intake_motors.brake();
    chassis.follow(bb_retrieve_txt, 15, 10000, true);
	pros::delay(100);
	arm_moveToAngle(75, 127);
	pros::delay(100);
    chassis.follow(bb_score_txt, 15, 10000, false);
	pros::delay(100);
	arm_moveToAngle(0, 127);
	pros::delay(500);
	intake_motors.move(-127);
	pros::delay(2000);
	intake_motors.brake();
}

void opcontrol() {
	while (true) {
		// start auton
		if (controller.get_digital(DIGITAL_A)) {
			
			controller.print(0,0, "AUTON STARTED");
			// delay for text to uplaod
			pros::delay(110);
			
			autonomous();

			controller.clear_line(0);
			pros::delay(110);
			controller.print(0,0, "AUTON ENDED");
			pros::delay(110);
		}

		if (controller.get_digital(DIGITAL_B)) {
			
			controller.clear_line(0);
			pros::delay(110);
			controller.print(0,0, "DRIVER CONTROL");
			pros::delay(110);

			// driver control loop
			while (true) {
				
				// get left y and right x positions
				int leftY = controller.get_analog(ANALOG_LEFT_Y);
				int rightX = controller.get_analog(ANALOG_RIGHT_X);

				// If turning in place (forward == 0), square the turn input and limit its max speed
				if (std::abs(leftY) < 5) {
					rightX = (rightX * rightX * (rightX > 0 ? 1 : -1))/127; // Preserve sign when squaring
					rightX *= 0.9; // Limits the maximum speed
				}

				// move the robot
				chassis.curvature(leftY, rightX);

				// move the arm
				arm_scroll(127);

				if (controller.get_digital(DIGITAL_R1)) {
					intake_motors.move(127);
				}
				if (controller.get_digital(DIGITAL_R2)) {
					intake_motors.move(-127);
				}

				// delay to save resources
				pros::delay(10);

				if (controller.get_digital(DIGITAL_A)) {
					break;
				}
			}
		}
	}	
}