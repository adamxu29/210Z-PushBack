#include "main.h"

using namespace Eclipse;

const u_int16_t forward_curve       = 10; 
const u_int16_t turn_curve          = 3;
const double euler                  = 2.71828;
static bool turning_sharp           = false;
static bool forward_sharp           = false;

int32_t joystick_accelerator(bool red, int8_t input, const double t){
    int16_t value = 0;
    // exponential formula
    if (red) { value = (std::exp(-t / 10) + std::exp((std::abs(input) - 100) / 10) * (1 - std::exp(-t / 10))) * input; } 
    else { value = std::exp(((std::abs(input) - 100) * t) / 1000) * input; }
    return value;
}

void Eclipse::OPControl::exponential_curve_accelerator(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightXjoystick) < 10) rightXjoystick = 0;

    double turn_val = joystick_accelerator(turning_sharp, rightXjoystick, turn_curve);
    double forward_val = joystick_accelerator(forward_sharp, leftYjoystick, forward_curve);
    double turnVoltage = turn_val * (12000.0 / 127); 
    double forwardVoltage = forward_val * (12000.0 / 127);
    
    double left =  forwardVoltage + turnVoltage;
    double right = forwardVoltage - turnVoltage;
    left_drive.move_voltage(left);
    right_drive.move_voltage(right);
}

void Eclipse::OPControl::drivetrain_control(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) * 0.95;
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

    int32_t left_power = (leftYjoystick + (rightXjoystick));
    int32_t right_power = (leftYjoystick - (rightXjoystick));

    left_drive.move_voltage(left_power * (12000.0 / 127));
    right_drive.move_voltage(right_power * (12000.0 / 127));
}

void Eclipse::OPControl::power_intake(){
    // scoring
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        // mid + long scoring
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, driver.skills ? 230 : 600) : scoring.set_scoring_mode(scoring.ScoringMode::Top);
    }
    // intaking
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        // match load
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad) : scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    }
    // bottom goal
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        intake_lift_active ? scoring.set_scoring_mode(scoring.ScoringMode::Bottom, driver.skills ? 200 : 600, 600) : scoring.set_scoring_mode(scoring.ScoringMode::Outtake);
    }
    else{
        scoring.set_scoring_mode(scoring.ScoringMode::None);
    }
}

void Eclipse::OPControl::intake_lift_control(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        intake_lift_active = !intake_lift_active;
    }
}

void Eclipse::OPControl::activate_back_wing(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        back_wing.set_value(true);
    }
    else{
        back_wing.set_value(false);
    }
}

void Eclipse::OPControl::activate_front_wing(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        front_wing.set_value(true);
    }
    else{
        front_wing.set_value(false);
    }
}
/*
    L1: Intake
    L2: Wing
    R1: Score
    R2: Outtake

    X: Run Auton
    A: Mid Goal Macro
    B: Low Goal Macro
    Y: Intake Lift Toggle

    Right: DSR
    Down: Odom Lift
    Left: 
    Up: Color Select
*/
void Eclipse::OPControl::driver_control(bool disabled){
    if(!disabled){
        driver.drivetrain_control();
        // driver.exponential_curve_accelerator();
        driver.power_intake();
        // scoring.change_intake_speed();
        driver.intake_lift_control();
        driver.activate_front_wing();
        driver.activate_back_wing();

        // driver.activate_match_load();
        // driver.activate_trapdoor();
    }

    // dsr test
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        odom.distance_sensor_reset(10, true, Odom::Front);
        odom.distance_sensor_reset(10, false, Odom::Right);
        std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    }
    
    // odom lift
    if(pros::competition::get_status() == 4){ odom_lift.set_value(true);}
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){ 
        odom_lift.set_value(true);
    }

    // run auton
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        disabled = true;
        autonomous();
        disabled = false;
        left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	    right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    }

    // low goal macro
    // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        
    // }
    // mid goal macro
    // else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        
    // }

    // color select
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        if(scoring.selected_color == scoring.Color::None){
            scoring.set_color(scoring.Color::Red);
        }
        else if(scoring.selected_color == scoring.Color::Red){
            scoring.set_color(scoring.Color::Blue);
        }
        else{
            scoring.set_color(scoring.Color::None);
        }
    }

    // auto select
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        if((gui.selected_path > 5)){
            gui.selected_path = -1;
        }
        else{ gui.selected_path++; }
    }
}

// void Eclipse::OPControl::power_indexer(float speed){
//     if(!driver.color_sorting){
//         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
//             indexer.move_voltage(12000 * (speed / 127));
//             color.set_led_pwm(100);
//         }
//         else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
//             indexer.move_voltage(-12000 * (speed / 127));
//             color.set_led_pwm(100);
//         }
//         else{
//             indexer.move_voltage(0);
//             color.set_led_pwm(0);
//         }
//     }
// }

// void Eclipse::OPControl::activate_double_park(){
//     if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
//         if(this->driver_disabled){
//             this->driver_disabled = false;
//             park.set_value(false);
//         }
//         else{
//             this->driver_disabled = true;
//             int counter = 0;
//             while(!util.detect_ball() && counter < 250){
//                 intake.move_voltage(-10000);
//                 counter++;
//                 pros::delay(10);
//             }
//             pros::delay(75);
//             intake.move_voltage(0);
//             intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//             park.set_value(true);
//         }
//     }
// }

// void Eclipse::OPControl::activate_trapdoor(){
//     if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//         this->trapdoor_down = !this->trapdoor_down;
//         trapdoor.set_value(this->trapdoor_down);
//     }
// }

// void Eclipse::OPControl::activate_match_load(){
//     if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
//         this->match_loading = !this->match_loading;
//         match_loader.set_value(this->match_loading);
//     }
// }
