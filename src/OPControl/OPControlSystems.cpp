#include "main.h"

using namespace Eclipse;

const u_int16_t forwardCurve       = 10;
const u_int16_t turnCurve          = 3;
const double euler                 = 2.71828;
static bool toggleRedCurve         = false; // toggle red curve
static bool turningRed             = false; // turning curve
static bool forwardRed             = false; // linear curve

int32_t joystick_accelerator(bool red, int8_t input, const double t){
    int16_t value = 0;
    // exponential formula
    if (red) { value = (std::exp(-t / 10) + std::exp((std::abs(input) - 100) / 10) * (1 - std::exp(-t / 10))) * input; } 
    else { value = std::exp(((std::abs(input) - 100) * t) / 1000) * input; }
    return value;
}

void Eclipse::OPControl::exponential_curve_accelerator(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    double turn_val = joystick_accelerator(turningRed, rightXjoystick, turnCurve);
    double forward_val = joystick_accelerator(forwardRed, leftYjoystick, forwardCurve);
    double turnVoltage = turn_val * (12000.0 / 127) * .9; 
    double forwardVoltage = forward_val * (12000.0 / 127);
    // std:: cout << turnVoltage << std::endl;
    // std:: cout <<"linear: " << forwardVoltage << std::endl;
    double left =  forwardVoltage + turnVoltage;
    double right = forwardVoltage - turnVoltage;
    left_drive.move_voltage(left);
    right_drive.move_voltage(right);
}

void Eclipse::OPControl::drivetrain_control(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

    int32_t left_power = (leftYjoystick + (rightXjoystick));
    int32_t right_power = (leftYjoystick - (rightXjoystick));

    left_drive.move_voltage(left_power * (12000.0 / 127));
    right_drive.move_voltage(right_power * (12000.0 / 127));
}

void Eclipse::OPControl::power_intake(float speed){ // speed in percent
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(12000 * (speed / 100));
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake.move_voltage(-12000 * (speed / 100));
    }
    else{
        intake.move_voltage(0);
    }
}

void Eclipse::OPControl::manual_pusher(float speed){ // speed in percent
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        pusher.move_voltage(12000 * (speed / 100));
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        pusher.move_voltage(-12000 * (speed / 100));
    }
    else{
        pusher.move_voltage(0);
    }
}

void Eclipse::OPControl::pusher_control(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        this->multiple++;
        this->cycle_counter++;
    }
}

void Eclipse::OPControl::power_pusher(float speed){ // speed in percent
    if(this->cycle_counter > 0){
        m_pid.set_constants(2, 0, 4, 3, 100, 200, 5, this->pusher_speed);
        m_pid.motor_pid(pusher, multiple * 2225);
        this->cycle_counter--;
    }
}

void Eclipse::OPControl::raise_shooter(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        this->shooter_raised = !this->shooter_raised;
        scoring_adjuster.set_value(this->shooter_raised);
    }
}

void Eclipse::OPControl::activate_match_load(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        this->match_loading = !this->match_loading;
        match_loader.set_value(this->match_loading);
    }
}

void Eclipse::OPControl::activate_descore(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        this->descore_active = !this->descore_active;
        descore.set_value(this->descore_active);
    }
}

void Eclipse::OPControl::change_pusher_speed(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        if(this->pusher_speed == 127){
            this->pusher_speed = 75;
        }
        else if(this->pusher_speed == 75){
            this->pusher_speed = 127;
        }
        else{
            this->pusher_speed = 127;
        }
    }
}

void Eclipse::OPControl::driver_control(){
    odom.update_position_single_vertical();
    
    // driver.exponential_curve_accelerator();
    driver.drivetrain_control();
    driver.power_intake(100);
    // driver.manual_pusher(100);
    driver.pusher_control();
    driver.change_pusher_speed();

    driver.raise_shooter();
    driver.activate_match_load();
    driver.activate_descore();
}