#include "main.h"

void initialize() {
	gui.initialize_styles();
	gui.initialize_objects();
	// gui.load_auton_selection();
	// gui.apply_auton_selection_ui();
	// gui.display_auton_selector();
	gui.display_sensors();
	// initialize_particles();
	
	util.set_drive_constants(2.75, 0.75, 600);
	util.set_tpi();

	odom.set_horizontal_tracker_specs(2.00, 0.0);
	odom.set_vertical_tracker_specs(2.00, 0.0);

	imu1.reset();

	vertical_tracking_wheel.reset_position();
	horizontal_tracking_wheel.reset_position();

	left_drive.set_zero_position(0);
    right_drive.set_zero_position(0);

	color.set_led_pwm(0);
	color.set_integration_time(3);
	
	pros::delay(3000);
	controller.rumble("..");

	odom.set_robot_heading(180);

	update_telemetry = new pros::Task(Eclipse::Odom::update_telemetry_fn);

	odom.set_robot_position(0.0, 0.0);

	// pros::Task color_sorting([]{
		// std::cout << "intake: " << intake.get_voltage() << std::endl;
	// 	util.run_sort();
	// });
}

void disabled() {
}

void competition_initialize() {
}

char buffer[300];
void autonomous(){
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	int start_time = pros::millis();
	// Run auton selector or skills
	// driver.skills ? auton.skills() : gui.run_selected_auton();
	// auton.solo_awp();
	auton.test();
	// auton.left_half_awp();
	// auton.right_9();
	// auton.left_7();
	// auton.right_half_awp();
	// auton.skills();
	
	std::cout << "time elapsed: " << (pros::millis() - start_time) / 1000.0 << std::endl;
}

void opcontrol() {
	// odom_lift.set_value(true);
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	bool tuning = false;

	while(true){
		controller.print(0, 0, "%s: %s	 DT: %0.1f", (gui.selected_color == 0 ? "R" : (gui.selected_color == 1 ? "B" : "0")), 
			(gui.selected_path == 0 ? "AWP" : (gui.selected_path == -1 ? "E" : "S")) ,util.get_drive_temp());
		gui.update_sensors();
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
