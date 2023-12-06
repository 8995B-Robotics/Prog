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

    // Subsystem Motors
pros::Motor Intake(20, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Catapult(11, pros::E_MOTOR_GEARSET_36, true);

    //All sensors
pros::Optical OpticalSens(19, 50); // Port 19, delay 50ms
pros::ADIDigitalIn LimitSwitch ('B'); // Limit on 3-Wire port B
bool LimitOn = false; // Boolean for the catapult auto-reload

pros::Controller master (CONTROLLER_MASTER);
 
// drivetrain motor groups
pros::MotorGroup left_side_motors({Motor1, Motor2, Motor3});
pros::MotorGroup right_side_motors({Motor4, Motor5, Motor6});

lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drive motors
    &right_side_motors, // right drive motors
    12.25, // track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360, // drivetrain rpm is 360
    2 // Omni vs Traction?
);
 
// inertial sensor
pros::Imu inertial_sensor(10);
 
// odometry sensors
lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 1
    &inertial_sensor // inertial sensor
};
 
// forward/backward PID
lemlib::ControllerSettings linearController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};
 
 
// create the chassis for LemLib Autonomous
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

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
	chassis.setPose(0, 0, 0); // X, Y, Heading
    pros::Task screenTask(screen); // create a task to print the position to the screen

    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}


void autonomous() {
chassis.turnTo(10, 0, 0, 800); 
}

void opcontrol() {    
  while (true) {
    int PowerLeft = master.get_analog(ANALOG_LEFT_X); 
    int TurnLeft = master.get_analog(ANALOG_LEFT_Y);

    int PowerRight = master.get_analog(ANALOG_RIGHT_X); 
    int TurnRight = master.get_analog(ANALOG_RIGHT_Y);

    // Set variables, like controller inputs 
    // int power = master.get_analog(ANALOG_LEFT_X); 
    // int turn = master.get_analog(ANALOG_LEFT_Y);
    bool LimitOn = false;

    if(master.get_digital(DIGITAL_R2)) {
        Intake = 105;
    } else if(master.get_digital(DIGITAL_R1)) {
        Intake = -105;
    } else {
        Intake = 0;
    }

// Catapult code
if(master.get_digital(DIGITAL_L2)) {
    Catapult = 127; // If we press L2, it shoots the catapult
    LimitOn = false; // Also resets the auto-reload

} else if(OpticalSens.get_hue() > 50 && OpticalSens.get_hue() < 130) { // GREEN TRIBALL
    Catapult = 127; // Shoots if it detects a triball in the low-arc area
    LimitOn = false;

} else if(OpticalSens.get_hue() > 200 && OpticalSens.get_hue() < 230) {// BLUE TRIBALL
    Catapult = 127; 
    LimitOn = false;

} else if(OpticalSens.get_hue() > 300 && OpticalSens.get_hue() < 359) { // RED TRIBALL #1
    Catapult = 127; 
    LimitOn = false;

} else if(OpticalSens.get_hue() > 0 && OpticalSens.get_hue() < 8) { // RED TRIBALL #2
    Catapult = 127; 
    LimitOn = false;
    
} else {
    if(LimitSwitch.get_value() == 0) { 
        if(LimitOn == false) {
            Catapult = 127; // Pulls catapult back until it hits the limit switch
        } else {
            Catapult = 0; // Stops catapult
            LimitOn = true; // Makes sure it doesn't continue moving until it shoots
        }
    } else {
        Catapult = 0; // If none of the conditions are somehow met, just don't move the catapult
    }
}

    // Set controller inputs into the drive sides
    // int left = power + turn;
    // int right = power - turn;
    // left_side_motors.move(left);
    // right_side_motors.move(right);

    int left = PowerLeft + TurnLeft;
    int right = PowerRight - TurnRight;
    left_side_motors.move(left);
    right_side_motors.move(right);
    
    pros::delay(2); // Delay so that the program doesn't draw too much memory and checks
  }
}