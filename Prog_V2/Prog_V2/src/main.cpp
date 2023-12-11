#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"

	

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

pros::Motor Motor4(4, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Motor5(5, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor Motor6(6, pros::E_MOTOR_GEARSET_06, true);

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
    12, // kP was 8, 12 is nice
    35, // kD was 30
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {
    6, // kP was 4
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
    selector::init();
}

void disabled() {}

ASSET(jerry_txt);


void autonomous() {
if(selector::auton == 1){ //Offense Red 
chassis.moveTo(0, 10, 0, 4000);
}

if(selector::auton == 2){ //Defense Red 
chassis.moveTo(10, 0, 0, 4000);
}

if(selector::auton == 3){ //Offence AWP Red 
chassis.moveTo(30, 30, 0, 4000);
}

if(selector::auton == -1){ //Offense Blue 
chassis.turnTo(10, 0, 1000, false, 60);
}

if(selector::auton == -2){ //Defense Blue 
chassis.turnTo(-10, 0, 1000, false, 60);
}

if(selector::auton == -3){ //Offense AWP Blue 
chassis.turnTo(0, -10, 1000, false, 60);
}

if(selector::auton == 0){ //Skills 
chassis.turnTo(-10, 0, 1000, false, 60);
pros::delay(10);
chassis.turnTo(0, -10, 1000, false, 60);
pros::delay(10);
chassis.turnTo(10, 0, 1000, false, 60);
pros::delay(10);
chassis.turnTo(0, 10, 1000, false, 60);
}
}

void opcontrol() {    
  while (true) {

    //single stick
    int Left_Y = master.get_analog(ANALOG_LEFT_Y); 
    int Left_X  = master.get_analog(ANALOG_LEFT_X);

    int CurveYPrep = pow(Left_Y, 3);
    int CurveY = CurveYPrep/10000;
    int CurveXPrep = pow(Left_X, 3);
    int CurveX = CurveXPrep/10000;

    chassis.arcade(CurveY, CurveX);
    
    //dual stick
    // int Left_Y = master.get_analog(ANALOG_LEFT_Y); 
    // int Right_X  = master.get_analog(ANALOG_RIGHT_X);
    // chassis.arcade(Left_Y, Right_X);

    //tank drive
    // int Left_Y = master.get_analog(ANALOG_LEFT_Y); 
    // int Right_Y  = master.get_analog(ANALOG_RIGHT_Y);
    // left_side_motors.move(Left_Y);
    // right_side_motors.move(Right_Y);

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

} else if(OpticalSens.get_hue() > 90 && OpticalSens.get_hue() < 130) { // GREEN TRIBALL
    pros::delay(40); // Delay so we don't hit our hand
    Catapult = 127; // Shoots if it detects a triball in the low-arc area
    LimitOn = false;

} else if(OpticalSens.get_hue() > 200 && OpticalSens.get_hue() < 230) {// BLUE TRIBALL
    pros::delay(40);
    Catapult = 127; 
    LimitOn = false;

} else if(OpticalSens.get_hue() > 300 && OpticalSens.get_hue() < 359) { // RED TRIBALL #1
    pros::delay(40);
    Catapult = 127; 
    LimitOn = false;

} else if(OpticalSens.get_hue() > 0 && OpticalSens.get_hue() < 8) { // RED TRIBALL #2
     pros::delay(40);
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

    // Single stick OLD
    // int left = power + turn;
    // int right = power - turn;
    // left_side_motors.move(left);
    // right_side_motors.move(right);


    //Dual stick OLD
    // int left = PowerLeft + TurnLeft;
    // int right = PowerRight - TurnRight;
    // left_side_motors.move(left);
    // right_side_motors.move(right);
    
    pros::delay(10); // Brain only updates every 10 ms
  }
}