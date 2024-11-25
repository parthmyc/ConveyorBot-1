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
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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



// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(PI / 2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration =
	    sin(PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
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
		    abs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

	controller->tank(left, right);

	prevTurn = iturn;
	prevThrottle = ithrottle;

	return std::make_pair(left,right)
}


#define left_wheels_port 1
#define right_wheels_port 2

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_wheels(left_wheels_port);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::Motor right_wheels(right_wheels_port, true);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Cheesy Drive
		double iturn = master.get_analog(ANALOG_RIGHT_X);
		double ithrottle = master.get_analog(ANALOG_LEFT_Y);

		left_wheels.move_velocity(cheesyDrive(ithrottle, iturn).left);
		right_wheels.move_velocity(cheesyDrive(ithrottle, iturn).right);
		
		pros::delay(20);                               // Run for 20 ms then update
	}
}