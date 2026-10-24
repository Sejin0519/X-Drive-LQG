#include "mylib/main.h"
#include "mylib/controller.hpp"
#include "mylib/sysid.hpp"
#include "mylib/paths.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
Config config = toSI({
	.centerToWheel = 6.46,
	.wheelRadius = 1.375,
	.gearRatio = 60.0/36.0,
	.motorPorts = {-12, -15, -17, -16},
	.verticalEncoderPort = 13,
	.horizontalEncoderPort = -18,
	.imuPort = 14,
	.horizontalOffset = 0,
	.verticalOffset = 0,
	.horizontalDiam = 2.125,
	.verticalDiam = 2.125,
	.dt = 0.01,
	
	.kV    = 0.165,   
	.kV_r  = 0.163,
	.kA    = 0.026,   
	.kA_r  = 0.0076,

	.vStatic = 0.45,
	.vMax = 12,

	.qPos = 1500, //needs tuning
	.qVel = 10,
	.rTrans = 0.0625,
	.qAng = 100,
	.qAngVel = 1,
	.rRot = 0.1,


    .qPos_f = 1e-8,
    .qVel_f = 4e-4,
    .rTrans_f = 4e-6,
    .qAng_f = 2.5e-7,
    .qAngVel_f = 0.0075,
    .rRot_f = 9e-6
});
Odometry odom(config);
Chassis chassis(config);
SysID sysId(chassis);
Controller controller(chassis, odom, config);

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
	odom.initialize();
	controller.runMotion();
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
	// sysId.moveRamp(5, 9000);
	// pros::delay(1000);
	// sysId.moveRamp(-5, 9000);
	// chassis.stop();

	// chassis.slowRamp(15, 5000, true);
	// pros::delay(1000);
	// chassis.slowRamp(-15, 5000, true);
	// chassis.stop();
	// printf("---\n");//borderline
	// odom.initialize();
	// sysId.turnStep(4000, 1500);
	// sysId.turnStep(-4000, 1500);
	// sysId.turnStep(8000, 1500);
	// sysId.turnStep(-8000, 1500);
	// sysId.turnStep(12000, 1500);
	// sysId.turnStep(-12000, 1500);
	// chassis.stop();
	// while(true) {
	// 	Vector6d x = odom.getState();
	// 	master.set_text(0, 0, "Vel: " + std::to_string(x(3)));
	// 	master.clear_line(0);

	// 	pros::delay(500);
	// }
	// DEBUG: test run, prints the logged filtered state at the end
	controller.followTrajectory(trajectory::Path);
	controller.waitUntilDone();

}