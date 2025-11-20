#include "main.h"

void initialize() {
	gui.initialize_styles();
	gui.initialize_objects();
	gui.display_home();
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

	util.set_robot_position(0.0, 0.0, -90);

	pros::Task update_gui([]{
		while(true){
			odom.update_position_single_vertical();

			position_mutex.take();
			gui.update_sensors();
			gui.update_temps();
			gui.update_match_checklist();
			position_mutex.give();

			pros::delay(10);
		}
	});

	pros::Task color_sorting([]{
		int stop_delay_counter = 0;
		while(true){
			if(util.sorting){
				if(gui.selected_color == 0){
					util.sort_red();
				}
				else if(gui.selected_color == 1){
					util.sort_blue();
				}
			}
			else{
				stop_delay_counter++;
				if(stop_delay_counter == 25){
					stop_delay_counter = 0;
					util.sorting = true;
				}
			}
			pros::delay(10);
		}
	});
}

void disabled() {

}

void competition_initialize() {

}

char buffer[300];
void autonomous(){
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	// Run auton selector or skills
	driver.skills ? auton.skills() : gui.run_selected_auton();
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
	int start_time = pros::millis();

	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	bool tuning = false;
	
	while(true){
		controller.print(0, 0, "DT: %0.1f", util.get_drive_temp());
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