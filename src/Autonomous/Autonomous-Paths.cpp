#include "main.h"
#include "pros/motors.h"
#include "map"

/*
Odometry coord finder: //https://path.jerryio.com/
*/

using namespace Eclipse;

// Translation PID constants: 5, 0.0025, 35, 600
// Rotation PID constants: 4.5, 0, (27 (θ = 45), 30 (θ = 90), 31 (θ = 180))
// Curve PID constants: 5, 0, 30

// Red paths
void Eclipse::Autonomous_Paths::Red::solo_awp()
{
}

void Eclipse::Autonomous_Paths::Red::left_half_awp()
{
   
}

void Eclipse::Autonomous_Paths::Red::right_half_awp()
{
    
}

void Eclipse::Autonomous_Paths::Red::goal_side_rush()
{
    
}

void Eclipse::Autonomous_Paths::Red::ring_side_rush()
{
}

// Blue paths
void Eclipse::Autonomous_Paths::Blue::solo_awp()
{
}

void Eclipse::Autonomous_Paths::Blue::left_half_awp()
{   
}

void Eclipse::Autonomous_Paths::Blue::right_half_awp()
{
}

void Eclipse::Autonomous_Paths::Blue::goal_side_rush()
{
}

void Eclipse::Autonomous_Paths::Blue::ring_side_rush()
{
}

// Misc paths
void Eclipse::Autonomous_Paths::skills()
{

}

void Eclipse::Autonomous_Paths::test()
{   
    intake.move_voltage(12000);
    scoring_adjuster.set_value(true);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(9, 31, false, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(21, 31, .7);

    driver.multiple += 2;

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(12, 90, .7);

    pros::delay(350);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-12, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(19, 21, false, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(24, 8, false, false, 1);

    pros::delay(250);
    
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(36, -3, false, false, 1);

    pros::delay(250);

    driver.multiple++;
    scoring_adjuster.set_value(false);

    pros::delay(750);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-6, 90, .7);

    scoring_adjuster.set_value(true);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(24, -21.7, false, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(27.7, -38, false, false, 1);

    pros::delay(250);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 127);
    drive.move_to_point(13, -66.2, false, true, 1.5);

    driver.multiple += 2;

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(15, -66.6, true, false, .5);

    left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    
    // t_pid.set_t_constants(3.5, 0.0, 10, 600);
    // t_pid.translation_pid(24, 90, 1);

    // r_pid.set_r_constants(2, 0.001, 13);
    // r_pid.rotation_pid(90, 90, 1);
}