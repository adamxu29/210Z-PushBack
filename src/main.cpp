#include "main.h"

void initialize() {
	// GUI disabled - remove all GUI tasks and initialization
	// gui.initialize_styles();
	// gui.initialize_objects();
	// gui.display_home();
	// initialize_particles();
	
	util.set_drive_constants(2.75, 0.75, 600);
	util.set_tpi();

	odom.set_horizontal_tracker_specs(2, 0.2);
	odom.set_vertical_tracker_specs(2.75, 0);

	imu1.reset();

	vertical_tracking_wheel.reset_position();
	horizontal_tracking_wheel.reset_position();

	left_drive.set_zero_position(0);
    right_drive.set_zero_position(0);

	color.set_led_pwm(0);
	color.set_integration_time(3);

	pros::delay(3000);
	controller.rumble(".");
}

void disabled() {

}

void competition_initialize() {

}

// Global buffer removed - no longer needed with GUI disabled
void autonomous(){
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	// Run auton selector or skills (GUI disabled)
	if (driver.skills) {
		auton.skills();
	} else {
		// gui.run_selected_auton();  // GUI disabled
		auton.solo_awp();  // Default auton since GUI is disabled
	}
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

void opcontrol() {
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	bool tuning = false;

	while(true){
		// GUI disabled - controller.print(0, 0, "DT: %0.1f", util.get_drive_temp());
		if(tuning){
			tuner.driver_tuner();
		}
		else{
			driver.driver_control(driver.driver_disabled);
		}
		pros::delay(10);
	}
	// pros::lcd::initialize();

    // // Optional: wait a bit for sensors to boot
    // pros::delay(500);

    // initialize_particles();  // Spread out particles randomly

    // while (true) {
    //     run_localization_step();  // Run the MCL update loop
    //     pros::delay(10);  
	// }
}