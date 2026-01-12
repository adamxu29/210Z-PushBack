#include "main.h"

using namespace Eclipse;

const u_int16_t forward_curve       = 10; 
const u_int16_t turn_curve          = 10;
const double euler                 = 2.71828;
static bool turning_sharp             = false;
static bool forward_sharp             = false;

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
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

    int32_t left_power = (leftYjoystick + (rightXjoystick));
    int32_t right_power = (leftYjoystick - (rightXjoystick));

    left_drive.move_voltage(left_power * (12000.0 / 127));
    right_drive.move_voltage(right_power * (12000.0 / 127));
}

void Eclipse::OPControl::power_intake(float speed){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(12000 * (speed / 127));
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake.move_voltage(-12000 * (speed / 127));
    }
    else{
        intake.move_voltage(0);
    }
}

void Eclipse::OPControl::power_indexer(float speed){
    if(!driver.color_sorting){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            indexer.move_voltage(12000 * (speed / 127));
            color.set_led_pwm(100);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            indexer.move_voltage(-12000 * (speed / 127));
            color.set_led_pwm(100);
        }
        else{
            indexer.move_voltage(0);
            color.set_led_pwm(0);
        }
    }
}

void Eclipse::OPControl::activate_match_load(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        this->match_loading = !this->match_loading;
        match_loader.set_value(this->match_loading);
    }
}

void Eclipse::OPControl::activate_double_park(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        if(this->driver_disabled){
            this->driver_disabled = false;
            park.set_value(false);
        }
        else{
            this->driver_disabled = true;
            int counter = 0;
            while(!util.detect_ball() && counter < 250){
                intake.move_voltage(-10000);
                counter++;
                pros::delay(10);
            }
            pros::delay(75);
            intake.move_voltage(0);
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            park.set_value(true);
        }
    }
}

void Eclipse::OPControl::activate_trapdoor(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        this->trapdoor_down = !this->trapdoor_down;
        trapdoor.set_value(this->trapdoor_down);
    }
}

void Eclipse::OPControl::activate_wing(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        wing.set_value(true);
    }
    else{
        wing.set_value(false);
    }
}

void Eclipse::OPControl::change_shooter_speed(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        if(this->shooter_speed == 127){
            this->shooter_speed = 67;
        }
        else if(this->shooter_speed == 67){
            this->shooter_speed = 127;
        }
        else{
            this->shooter_speed = 127;
        }

        if(this->intake_speed == 127){
            this->intake_speed = 67;
        }
        else if(this->intake_speed == 67){
            this->intake_speed = 127;
        }
        else{
            this->intake_speed = 127;
        }
    }
}

void Eclipse::OPControl::driver_control(bool disabled){
    if(!disabled){
        driver.exponential_curve_accelerator();
        // driver.drivetrain_control();
        driver.power_intake(intake_speed);
        driver.power_indexer(shooter_speed);

        driver.change_shooter_speed();
    
        driver.activate_match_load();
        driver.activate_trapdoor();
        driver.activate_wing();
    }
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        odom.distance_sensor_reset(5, true);
    }
    // driver.activate_double_park();
}