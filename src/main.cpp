#include "main.h"

void initialize() {
	gui.initialize_styles();
	gui.initialize_objects();
	// gui.load_auton_selection();
	// gui.apply_auton_selection_ui();
	// gui.display_auton_selector();
	// gui.display_match_checklist();
	gui.display_debug_terminal();
	
	util.set_drive_constants(4.00, 0.5714285714, 600);
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

	// offset_x/y: body-frame mount position (inches)
	// offset_theta: body-frame pointing direction (radians)
	// sigma: Gaussian noise std-dev (inches) — start at 3", tighten if convergence is slow
	// max_range: reject readings beyond this (inches)
	mcl.sensors = {
		{ &front_sensor,  4.5920000005f,  5.903273329f,          0.0f,          3.0f, 100.0f },
		{ &back_sensor,   5.0187007815f, -6.194094488f,     (float)M_PI,        3.0f, 100.0f },
		{ &left_sensor,  -4.686594488f, -0.887590776f, -(float)M_PI / 2.0f,    3.0f, 100.0f },
		{ &right_sensor,  4.686594488f, -0.61425513f,   (float)M_PI / 2.0f,    3.0f, 100.0f },
	};

	pros::delay(3000);
	controller.rumble("..");

	odom.set_robot_heading(180);

	mcl.init(0.0f, -46.0f, 180.0f, 3.0f);
	mcl.enabled = true;

	update_telemetry = new pros::Task(Eclipse::Odom::update_telemetry_fn);
	// run_intake = new pros::Task(Eclipse::Intake::intake_task);

	odom.set_robot_position(0.0, 0.0);

	m_pid.set_constants(20, 5, 0, 3, 1, 1, 5, 127);

	run_intake = new pros::Task([]{
		while(true){
			scoring.intake_task();
			pros::delay(10);
		}
	});
}

void disabled() {
}

void competition_initialize() {
	front_wing.set_value(true);
	back_wing.set_value(true);
}

char buffer[300];
void autonomous(){
	front_wing.set_value(false);
	back_wing.set_value(false);
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	int start_time = pros::millis();

	// Run auton selector or skills
	driver.skills ? auton.skills() : gui.run_selected_auton();
	// auton.solo_awp();
	// auton.test();
	// auton.right_half_awp();
	// auton.left_half_awp();
	// auton.right_7();
	// auton.left_7();
	// auton.right_9();
	// auton.left_9();
	// auton.skills();
	
	std::cout << "time elapsed: " << (pros::millis() - start_time) / 1000.0 << std::endl;
	char buffer[300];
	sprintf(buffer, "Time: %f", (pros::millis() - start_time) / 1000.0);
    lv_label_set_text(gui.debug_line_2, buffer);
}

void opcontrol() {
	// odom_lift.set_value(true);
	left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	bool tuning = false;

	while(true){
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		// 	mcl.debug_sensors();
		// 	mcl.debug_likelihood(ex, ey, util.get_heading());
		// }

		controller.print(0, 0, "%s: %s	 DT: %0.1f", (scoring.selected_color == scoring.Color::Red ? "R" : (scoring.selected_color == scoring.Color::Blue ? "B" : "0")),
			(driver.skills ? "Skills" : ((gui.selected_path == 0) ? "WP" : (gui.selected_path == 1) ? "LWP" : (gui.selected_path == 2) ? "RWP" : (gui.selected_path == 3) ? "R7" : (gui.selected_path == 4) ? "L7" : (gui.selected_path == 5) ? "R9" : (gui.selected_path == 6) ? "L9" : "Test")) ,util.get_drive_temp());
		gui.update_sensors();
		if(tuning){
			tuner.driver_tuner();
		}
		else{
			gui.update_match_checklist();
			driver.driver_control(driver.driver_disabled);
		}
		pros::delay(10);
	}
}
