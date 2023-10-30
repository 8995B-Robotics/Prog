#include "main.h"
#include "lemlib/api.hpp"

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
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr2(1, pro::E_Motor_Gear_06,false) // port 2, blue gearbox, not reversed
	pros::Motor left_mtr3(2, pro::E_Motor_Gear_06,false) // port 3, blue gearbox, not reversed
	pros::Motor Topleft_mtr(3, pro::E_Motor_Gear_06,false) // port 1, blue gearbox, not reversed
	pros::Motor right_mtr2(4, pro::E_Motor_Gear_06, true) // port 5, blue gearbox, reversed
	pros::Motor right_mtr3(5, pro::E_Motor_Gear_06, true) // port 6, blue gearbox, reversed
	pros::Motor Topright_mtr1(6, pro::E_Motor_Gear_06,true) // port 4, blue gearbox, reversed    

	pros::MotorGroup  left_side_motors({left_mtr2,left_mtr3, Topleft_mtr})
	pros::MotorGroup  right_side_motors({right_mtr2,right_mtr3, Topright_mtr})

	lemlib::Drivetrain_t drivetrain {
		&leftMotors, // left drivetrain motors
		&rightMotors // right drivetrain motors
		13, // track widith
		3.25, // wheel diameter
		360 // wheel rpm
	};
	
	// forward/bakcward PID
	lemlib::ChassisController_t lateralController {
		8, // kP
		30, // kD
		1, // smallErrorTimeout
		3, // largeErorRange
		500, // largeEroorTimeout
		5 // slew rate
	};

	// turning PID
	lemlib::ChassisController_t angularController {
		4, // kP
		40, // kD
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		0 // slew rate
	};

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		pros::delay(20);
	}
}
