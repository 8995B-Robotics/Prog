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
    13, // track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360, // drivetrain rpm is 360
    8 // 2 = allOmni, 8 = w/ traction
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
	chassis.setPose(-35, 64, 0); // X, Y, Heading
    pros::Task screenTask(screen); // create a task to print the position to the screen
}

void disabled() {}

// Blue Offense paths

// Name is "B" for Blue
// "O" for offensive
// "04" is how many triballs we want to score
// "0.." is the number of the path
// Has to be "_txt" because then c++ is happy 

ASSET(BO4B00_txt); 
ASSET(BO4B01_txt);
ASSET(BO4B02_txt);
ASSET(BO4B03_txt);
ASSET(BO4B04_txt);
ASSET(BO4B05_txt);
ASSET(BO4B06_txt);
ASSET(BO4B07_txt);
ASSET(BO4B08_txt);
ASSET(BO4B09_txt);


void autonomous() {
// Blue Offensive zone (Top Left)

chassis.follow(BO4B00_txt, 10, 4000, false); // Path File, Lookahead, Timeout, backwards or not
pros::delay(40); // Triball rolls on floor
chassis.follow(BO4B01_txt, 10, 4000, false); // move to middle triball
Intake = 105; // grab triball #1
chassis.follow(BO4B02_txt, 10, 4000, true); // move triball with intake down
chassis.turnTo(-50, 12, 1000, true, 60); // turn around
Intake = -105; // let go of triball
pros::delay(30);
Intake = 0;
chassis.follow(BO4B03_txt, 10, 4000, false); // ram triball into goal
chassis.follow(BO4B04_txt, 10, 4000, true); // drive to second triball
chassis.turnTo(-4, 0, 1000, true, 60); // turn around to triball
chassis.follow(BO4B05_txt, 10, 4000, false); // get closer to second triball
Intake = 105; // grab triball #2
chassis.follow(BO4B06_txt, 10, 4000, true); // drive with it close to goal
chassis.turnTo(-50, 12, 1000, true, 60); // turn around
Intake = -105; // let go of triball
pros::delay(30);
Intake = 0;
chassis.follow(BO4B07_txt, 10, 4000, false); // ram triball #2 into goal
chassis.follow(BO4B08_txt, 10, 4000, true); // drive to push the preload into the goal
chassis.follow(BO4B09_txt, 10, 4000, false); // move to the horizontal elevation bar
Intake = 105; // Touch the bar with intake
}

void opcontrol() {    
  while (true) {
    // Set variables, like controller inputs 
    int power = master.get_analog(ANALOG_LEFT_X); 
    int turn = master.get_analog(ANALOG_LEFT_Y);
    bool LimitOn = false;

// Catapult code
if(master.get_digital(DIGITAL_L2)) {
    Catapult = 105; // If we press L2, it shoots the catapult
    LimitOn = false; // Also resets the auto-reload
} else if(OpticalSens.get_hue() > 55 && OpticalSens.get_hue() < 100) {
    Catapult = 105; // Shoots if it detects a triball in the low-arc area
    LimitOn = false;
} else {
    if(LimitSwitch.get_value() == 0) {
        if(LimitOn == false) {
            Catapult = 105; // Pulls catapult back until it hits the limit switch
        } else {
            Catapult = 0; // Stops catapult
            LimitOn = true; // Makes sure it doesn't continue moving until it shoots
        }
    } else {
        Catapult = 0; // If none of the conditions are somehow met, just don't move the catapult
    }
}
    // Set controller inputs into the drive sides
    int left = power + turn;
    int right = power - turn;
    left_side_motors.move(left);
    right_side_motors.move(right);
    
    pros::delay(2); // Delay so that the program doesn't draw too much memory and checks
  }
}