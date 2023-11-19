#include "main.h"
#include "lemlib/api.hpp"
	

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

	// drivetrain motors: Port, Gearset, reversed/not
pros::Motor Motor1(1, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor Motor2(2, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor Motor3(3, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor Motor4(4, pros::E_MOTOR_GEARSET_06, true);
pros::Motor Motor5(5, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor Motor6(6, pros::E_MOTOR_GEARSET_06, false);

pros::Motor Intake(20, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Catapult(11, pros::E_MOTOR_GEARSET_36, true);
bool CataOn = false;

pros::ADIDigitalIn LimitSwitch ('B'); // Limit on 3-Wire port B
bool LimitOn = false;

pros::Optical OpticalSens(19, 50); // Port 19, delay 50ms
bool OpticalDetected = false;

pros::Controller master (CONTROLLER_MASTER);
 
// drivetrain motor groups
pros::MotorGroup left_side_motors({Motor1, Motor2, Motor3});
pros::MotorGroup right_side_motors({Motor4, Motor5, Motor6});
 
lemlib::Drivetrain_t drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    13, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};
 
// inertial sensor
pros::Imu inertial_sensor(10);
 
// odometry struct
lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 1
    &inertial_sensor // inertial sensor
};
 
// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
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
    40 // slew rate
};
 
 
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::Task screenTask(screen); // create a task to print the position to the screen
}

void disabled() {}

void autonomous() {
    chassis.turnTo(10, 0, 1000, false); // turn to the point (10, 0) with a timeout of 1000 ms
	chassis.moveTo(10, 0, 1000); // move to the point (10, 0) with a timeout of 1000 ms
}

void opcontrol() {    
  while (true) {
    int power = master.get_analog(ANALOG_LEFT_X);
    int turn = master.get_analog(ANALOG_LEFT_Y);

    if(LimitSwitch.get_value() == 1) {
        LimitOn = true;
    } else if(LimitSwitch.get_value() == 0) {
        LimitOn = false;
    }

    if(OpticalSens.get_hue() > 55 && OpticalSens.get_hue() < 85) {
        OpticalDetected = true;
    } else {
        OpticalDetected = false;
    }


    if (master.get_digital(DIGITAL_B)) { // If L2 is pressed, then the cata is on
        if(CataOn == true) {
            CataOn = false;
        } else if(CataOn == false) {
            CataOn = true;
        }
    }

    while (CataOn && !LimitOn) { // While Cata is on, and limit switch isn't pressed
            Catapult = 105; // Spin cata back
        }

        if(CataOn && LimitOn && !OpticalDetected) {
            Catapult = 0;
        } else if(CataOn && OpticalDetected && LimitOn) {
            while (LimitOn){ // Spin until limit switch isn't pressed (catapult is fired)
                Catapult = 105;
            }
        }
/*
    if(CataOn && OpticalDetected && LimitOn) { // Only fire if cata is on, there is a triball detected, and the limitswitch is pressed
            while (LimitSwitch.get_value() == 1){ // Spin until limit switch isn't pressed (catapult is fired)
                Catapult = 105;
            }
        }
*/
    int left = power + turn;
    int right = power - turn;
    left_side_motors.move(left);
    right_side_motors.move(right);

    pros::delay(2);
  }
}