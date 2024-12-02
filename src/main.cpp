#include "main.h"

/**
 * Cheesy Drive Constants
 */
#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.02f
#define CD_TURN_NONLINEARITY                                                   \
  0.65 // This factor determines how fast the wheel
       // traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 1.0

/**
 * lemlib config
 */
// Create motor group with two motors
pros::MotorGroup left_motor_group({2}, pros::v5::MotorGears::green);
pros::MotorGroup right_motor_group({-1}, pros::v5::MotorGears::green);
pros::MotorGroup intake_motors({6, -7}, pros::v5::MotorGears::blue);
pros::MotorGroup arm_motor({5}, pros::v5::MotorGears::red);

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
void autonomous() { //turn maxSpeed = 50  drive = 60
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(170, 3000, {.maxSpeed = 50});
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


/**
// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(M_PI / 2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration =
	    sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
double quickStopAccumlator = 0.0;
double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
	if (negInertiaAccumlator > 1) {
		negInertiaAccumlator -= 1;
	} else if (negInertiaAccumlator < -1) {
		negInertiaAccumlator += 1;
	} else {
		negInertiaAccumlator = 0;
	}

	if (quickStopAccumlator > 1) {
		quickStopAccumlator -= 1;
	} else if (quickStopAccumlator < -1) {
		quickStopAccumlator += 1;
	} else {
		quickStopAccumlator = 0.0;
	}
}

double prevTurn = 0.0;
double prevThrottle = 0.0;
std::pair<double,double> cheesyDrive(double ithrottle, double iturn) {
	bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (ithrottle - prevThrottle > DRIVE_SLEW) {
		linearCmd = prevThrottle + DRIVE_SLEW;
	} else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
		// We double the drive slew rate for the reverse direction to get
		// faster stopping.
		linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	}

	double remappedTurn = _turnRemapping(iturn);

	double left, right;
	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * std::abs(remappedTurn);
		right = -remappedTurn * std::abs(remappedTurn);

		// The FRC Cheesy Drive Implementation calculated the
		// quickStopAccumulator here:
		// if (Math.abs(linearPower) < 0.2) {
		// 	double alpha = 0.1;
		// 	quickStopAccumulator = (1 - alpha) * quickStopAccumulator
		// 			+ alpha * Util.limit(wheel, 1.0) * 5;
		// }
		// But I apparently left that out of my implementation? Seemed to work
		// without it though.
	} else {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		    std::abs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

	prevTurn = iturn;
	prevThrottle = ithrottle;

	return std::make_pair(left,right);
}
 */

pros::Controller controller(CONTROLLER_MASTER);

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

				// if;

				// delay to save resources
				pros::delay(10);

				if (controller.get_digital(DIGITAL_A)) {
					break;
				}
			}
		}
	}
	/**
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Cheesy Drive
		double iturn = master.get_analog(ANALOG_LEFT_X);
		double ithrottle = master.get_analog(ANALOG_LEFT_Y);

		const int maxRPM = 200; // Example max RPM for green gear cartridge
		left_wheels.move_velocity(maxRPM * cheesyDrive(ithrottle, iturn).first);
		right_wheels.move_velocity(maxRPM * cheesyDrive(ithrottle, iturn).second);

		
		pros::delay(2);                               // Run for 20 ms then update
	} */
	
}