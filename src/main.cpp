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
                              11.5, // track width
                              lemlib::Omniwheel::OLD_4, // wheel type
                              200, // rpm
                              8 // horizontal drift
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


// PID controller config

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10.5, // proportional gain (kP) 9.8
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
lemlib::ControllerSettings angular_controller(1.4, // proportional gain (kP) 0.9 1.8 1.4
                                              0, // integral gain (kI)
                                              8.6, // derivative gain (kD) 8 1.1 8.625
                                              0, // anti windup 13.65
                                              0.05, // small error range, in degrees
                                              300, // small error range timeout, in milliseconds
                                              0.1, // large error range, in degrees
                                              1500, // large error range timeout, in milliseconds
                                              0.8 // maximum acceleration (slew) 0.8
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(1, // joystick deadband out of 127
                                      1, // minimum output where drivetrain will move out of 127
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
pros::MotorGroup arm_motor({-20}, pros::v5::MotorGears::red);
pros::ADIEncoder arm_encoder (6, 7, false);
pros::ADIDigitalIn arm_limit(5);

// PID class for arm (unused, arm encoder was not functioning correctly at comp)
lemlib::PID arm_controller(20,  // kP
						   0,  // kI
						   0,  // kD
						   0,  // integral anti windup range
						   false  // don't reset integral when sign of error flips
);

// Set bottom position to 0
void arm_calibrate() {
	arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm_motor.move(127);
	pros::delay(600);
	while (!arm_limit.get_value()) {
		arm_motor.move(-63);
	}

	arm_motor.brake();
	arm_encoder.reset();
}

// Variable to track the current limited power
double currentPower = 0;

// Unused arm movement code that utilizses PID
void arm_moveToAngle(double angle, double maxSpeed) {
	double error = arm_encoder.get_value() - angle;
	double power = arm_controller.update(error);

	// We attempted implenting slew rate and speed limits to the unused arm PID controller but we commented them out in a troubleshooting attempt

	// Apply the slew rate limiter to the power
	// double limitedPower = lemlib::slew(power, currentPower, 5); //set maxAccel as slew rate

	// Maximum speed limit
	// if (fabs(limitedPower) > maxSpeed) {
	// 	limitedPower = maxSpeed;
	// }

	// Update the current power
	// currentPower = limitedPower;

	// Move the arm motor
	arm_motor.move(currentPower);
}

// Arm scrolling (unused)
// Predefined arm positions
double arm_positions[] = {0.0, 10, 80, 128};
int position_count = sizeof(arm_positions) / sizeof(arm_positions[0]);

// move arm to specific postions when right shoulder buttons are pressed (unused)
void arm_scroll(double maxSpeed) {
	static int i = 0; // Current selected position


	// Move to the selected position
	arm_motor.move_absolute(7 * arm_positions[i], 127);
	
	// arm_moveToAngle(arm_positions[i], maxSpeed);
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


// Initialization code. This occurs as soon as the program is started.
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
            pros::delay(20);
        }
    });
}


// Unused PATH.JERRIO implementation
ASSET(start_goal_txt);
ASSET(bb1_txt);
ASSET(bb1_reverse_txt);
ASSET(bb2_txt);
ASSET(bb2_reverse_txt);
ASSET(bb3_txt);
ASSET(bb_retrieve_txt);
ASSET(bb_score_txt);
ASSET(test_txt);
ASSET(example_txt);


// Autonomous control code
void autonomous() { //turn maxSpeed = 50  drive = 60
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
	// move forward to score soccer ball
	chassis.moveToPoint(0, 20, 5000, {.maxSpeed = 100});
	pros::delay(1000);

}

// Main code, automatically runs after initialization
void opcontrol() {
	while (true) {
		// start auton
		if (controller.get_digital(DIGITAL_A)) {
			// Prompt remote
			controller.print(0,0, "AUTON STARTED");
			// delay for text to uplaod
			pros::delay(110);
			
			autonomous();

			controller.clear_line(0);
			pros::delay(110);
			controller.print(0,0, "AUTON ENDED");
			pros::delay(110);
		}

		// Start operator control when B is pressed
		if (controller.get_digital(DIGITAL_B)) {
			
			controller.clear_line(0);
			pros::delay(110);
			// Prompt remote
			controller.print(0,0, "DRIVER CONTROL");
			pros::delay(110);

			arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

			// driver control loop
			while (true) {
				// get left y and right x positions
				int leftY = controller.get_analog(ANALOG_LEFT_Y);
				int rightX = controller.get_analog(ANALOG_RIGHT_X);

				// scaling controls by 0.85
				leftY *= 0.85;
				rightX *= 0.85;

				// move the robot
				chassis.curvature(leftY, rightX);

				// move the arm
				if (controller.get_digital(DIGITAL_L1))
				{
					arm_motor.move(100);
				}
				else if (controller.get_digital(DIGITAL_L2))
				{
					arm_motor.move(-100);
				}
				else
				{
					arm_motor.brake();
				}

				// move the intake
				if (controller.get_digital(DIGITAL_R1)) {
					intake_motors.move(127);
				}
				else if (controller.get_digital(DIGITAL_R2)) {
					intake_motors.move(-127);
				}
				else
				{
					intake_motors.brake();
				}
				
				// arm calibration for arm scroll (unused)
				if (controller.get_digital(DIGITAL_X)) {
					arm_calibrate();
				}

				// delay to save resources
				pros::delay(10);

				// exit driver control
				if (controller.get_digital(DIGITAL_A)) {
					break;
				}
			}
		}
	}	
}