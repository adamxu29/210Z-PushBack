#include "main.h"

void initialize() {
	gui.initialize_styles();
	gui.initialize_objects();

	gui.display_sensors();
	// initialize_particles();
	
	util.set_drive_constants(2.75, 0.75, 600);
	util.set_tpi();

	odom.set_horizontal_tracker_specs(2, 0.2);
	odom.set_vertical_tracker_specs(2.75, 0);

	imu1.tare_rotation();
	imu2.tare_rotation();

	vertical_tracking_wheel.reset_position();
	horizontal_tracking_wheel.reset_position();

	left_drive.set_zero_position(0);
    right_drive.set_zero_position(0);
	pusher.set_zero_position(0);
	pusher.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	
	util.set_robot_position(0.0, 0.0);

	pros::delay(3000);
	controller.rumble(".");
	
	pros::Task update_gui([]{
		while(true){
			gui.update_sensors();
			gui.update_temps();
			gui.update_match_checklist();
			pros::delay(8);
		}
	});

	pros::Task update_odom([]{
		while(true){
			// Particle estimate = get_estimate();
			odom.update_position_single_vertical();
			// //predict();
			// run_localization_step();
			// pros::delay(10);

			// char buffer[300];
			// //sprintf(buffer, "Front: %.2f Back: %.2f Left: %.2f Right: %.2f", front_sensor.get(), back_sensor.get(), left_sensor.get(), right_sensor.get());
			// sprintf(buffer, "X: %.2f Y: %.2f Theta: %.2f", estimate.x, estimate.y, imu1.get_heading());
			// lv_label_set_text(gui.position_readings, buffer);
			pros::delay(8);
		}
	});

	pros::Task power_pusher([]{
		while(true){
			driver.power_pusher(driver.pusher_speed);
			pros::delay(8);
		}
	});

}

void disabled() {

}

void competition_initialize() {

}

char buffer[300];
void autonomous(){
	// t_pid constants: kp: 5, kd: 15
	// r_pid constants: kd: 2.5, kd: 15

	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	left_drive.set_zero_position(0);
    right_drive.set_zero_position(0);
	pusher.set_zero_position(0);

	// Run auton selector for
	// auton.skills();
	gui.run_selected_auton();
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
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bool tuning = false;

	while(true){
		controller.print(0, 0, "DT: %0.1f", util.get_drive_temp());
		if(tuning){
			tuner.driver_tuner();
		}
		else{
			driver.driver_control();
		}
		pros::delay(8);
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