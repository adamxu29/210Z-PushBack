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
void Eclipse::Autonomous_Paths::Red::solo_awp(){
    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-10, 17, false, false, .7);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(17, 30, 1.5);

    intake.move_voltage(1500);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-11, 32, true, false, 1);
    intake.move_voltage(0);

    driver.multiple++;
    driver.cycle_counter++;

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(20, 58, .7);

    pros::delay(350);
    scoring_adjuster.set_value(true);
    intake.move_voltage(12000);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(16, 9, false, true, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(26, 20, .7);

    driver.multiple = 1.29;
    driver.cycle_counter++;

    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(25, 30, 1.5);

    intake.move_voltage(0);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-8, 90, .4);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(23, 30, true, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-6, 58, .7);

    intake.move_voltage(-9000);
    pros::delay(600);
    intake.move_voltage(-12000);
    pros::delay(300);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(50, 0, false, true, 1.5);

    intake.move_voltage(0);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(50, -20, .7);
    match_loader.set_value(true);

    driver.multiple = 1;
    driver.cycle_counter++;

    intake.move_voltage(0);
    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(14, 90, .7);

    intake.move_voltage(12000);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(52, 40, 1);

    driver.multiple += 3;
    driver.cycle_counter++;

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(9, 75, 1);
}

void Eclipse::Autonomous_Paths::Red::left_half_awp(){
    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-10, 17, false, false, .7);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(17, 30, 1.5);

    intake.move_voltage(1500);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-11, 32, true, false, 1);
    intake.move_voltage(0);

    driver.multiple++;
    driver.cycle_counter++;
    intake.move_voltage(12000);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(21, 58, .7);

    scoring_adjuster.set_value(false);

    pros::delay(350);
    scoring_adjuster.set_value(true);
    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-7, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-40, 2, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(-39, 2, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-39, -20, .7);

    intake.move_voltage(12000);
    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(16, 90, .7);
    pros::delay(150);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    pros::delay(250);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(-39, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(11, 90, .7);

    intake.move_voltage(12000);

    driver.pusher_speed = 90;
    driver.multiple += 3;
    driver.cycle_counter++;

    pros::delay(2500);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-9, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-39, -20, .7);

    intake.move_voltage(0);
    match_loader.set_value(true);
    pros::delay(150);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(12, 90, .7);
}

void Eclipse::Autonomous_Paths::Red::right_half_awp(){
    intake.move_voltage(12000);
    scoring_adjuster.set_value(true);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(6, 16, false, false, .7);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(21, 30, 1.5);

    intake.move_voltage(0);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-4, 30, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(7, 32, true, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-21, 59, .4);

    intake.move_voltage(-12000);
    pros::delay(750);
    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-5, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(33, 2, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(34, 2, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(34, -20, .7);

    scoring_adjuster.set_value(true);
    intake.move_voltage(0);
    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(15, 90, .7);

    intake.move_voltage(12000);
    pros::delay(500);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    pros::delay(250);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(34, 30, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(11, 90, .7);

    intake.move_voltage(12000);
    driver.pusher_speed = 90;
    driver.multiple += 3;
    driver.cycle_counter++;
    pros::delay(2300);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-9, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(34, -20, .7);

    intake.move_voltage(0);
    match_loader.set_value(true);
    pros::delay(150);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(12, 90, .7);
}

void Eclipse::Autonomous_Paths::Red::goal_side_rush()
{
    
}

void Eclipse::Autonomous_Paths::Red::ring_side_rush()
{
}

// Blue paths
void Eclipse::Autonomous_Paths::Blue::solo_awp(){
    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-10, 17, false, false, .7);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(17, 30, 1.5);

    intake.move_voltage(1000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-10, 31, true, false, 1);
    intake.move_voltage(0);

    driver.multiple++;
    driver.cycle_counter++;

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(20, 58, .7);

    pros::delay(350);
    scoring_adjuster.set_value(true);
    intake.move_voltage(12000);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(17, 8, false, true, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(26, 20, .7);

    driver.multiple = 1.29;
    driver.cycle_counter++;

    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(25, 30, 1.5);

    intake.move_voltage(0);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-8, 90, .4);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(23, 30, true, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-5, 58, .7);

    intake.move_voltage(-9000);
    pros::delay(600);
    intake.move_voltage(-12000);
    pros::delay(250);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(52, 0, false, true, 1.5);

    intake.move_voltage(0);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(50, -20, .7);
    match_loader.set_value(true);

    driver.multiple = 1;
    driver.cycle_counter++;

    pros::delay(350);
    intake.move_voltage(12000);
    
    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(14, 90, .7);

    pros::delay(50);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(54, 40, 1);

    driver.multiple += 2;
    driver.cycle_counter++;

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(9, 75, 1);
}

void Eclipse::Autonomous_Paths::Blue::left_half_awp(){
    intake.move_voltage(12000);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-10, 17, false, false, .7);

    driver.multiple = 0.29;
    driver.cycle_counter++;

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(21, 30, 1.5);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-4, 30, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(-11, 31, true, false, 1);

    driver.multiple = 1;
    driver.cycle_counter++;

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(21, 58, .7);

    scoring_adjuster.set_value(false);

    pros::delay(350);
    scoring_adjuster.set_value(true);
    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-7, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-15, 41, .7);

    c_pid.set_c_constants(5, 0, 30);
    c_pid.curve_pid(-90, 90, 1.5, .2, false);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(6, 50, .5);

    match_loader.set_value(true);
    pros::delay(500);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-6, 90, .7);

    match_loader.set_value(false);
    pros::delay(150);

    c_pid.set_c_constants(5, 0, 30);
    c_pid.curve_pid(0, 90, 1.5, .2, true);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(-43, 2, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-42, -20, .7);

    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(14, 90, .7);

    pros::delay(50);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(-42, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(10, 90, .7);

    intake.move_voltage(12000);
    driver.multiple += 1;
    driver.cycle_counter++;

    pros::delay(100);

    driver.multiple += 2;
    driver.cycle_counter++;   
}

void Eclipse::Autonomous_Paths::Blue::right_half_awp(){
    intake.move_voltage(12000);
    scoring_adjuster.set_value(true);
    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(9, 16, false, false, .7);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(21, 30, 1.5);

    intake.move_voltage(0);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-4, 30, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 75, 75);
    drive.move_to_point(12, 30, true, false, 1);

    intake.move_voltage(-9000);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-16, 59, .7);

    pros::delay(100);
    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-8, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(15, 41, .7);

    c_pid.set_c_constants(5, 0, 30);
    c_pid.curve_pid(90, 90, 1.5, .19, false);

    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(6, 50, .5);

    match_loader.set_value(true);
    pros::delay(1000);
    match_loader.set_value(false);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-6, 90, .7);

    c_pid.set_c_constants(5, 0, 30);
    c_pid.curve_pid(0, 90, 1.5, .19, true);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(42, 2, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(42, -20, .7);

    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(16, 90, .7);

    pros::delay(150);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(42, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(10, 90, .7);

    intake.move_voltage(12000);
    driver.multiple += 1;
    driver.cycle_counter++;

    pros::delay(100);

    driver.multiple += 2;
    driver.cycle_counter++;
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
    driver.pusher_speed = 90;
    intake.move_voltage(12000);
    scoring_adjuster.set_value(true);
    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(6, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(38, 2, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(39, -20, .7);

    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(18, 90, .7);

    pros::delay(2000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(40, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(9, 90, .7);

    intake.move_voltage(12000);
    driver.multiple += 3;
    driver.cycle_counter++;

    pros::delay(2500);
    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-5, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(-48, 0, true, false, 3);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-48, -20, .7);

    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(18, 90, .7);

    pros::delay(2000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(-48, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(9, 90, .7);

    intake.move_voltage(12000);
    driver.multiple += 4;
    driver.cycle_counter++;

    pros::delay(2500);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-5, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.move_to_point(-30, -15, false, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(0, -15, 1);

    intake.move_voltage(-12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(25, 127, 1.5);
}

void Eclipse::Autonomous_Paths::test(){   
    driver.pusher_speed = 90;
    intake.move_voltage(12000);
    scoring_adjuster.set_value(true);
    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(6, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(39, 0, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(39, -20, .7);

    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(15, 90, .7);

    pros::delay(400);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-10, 90, .7);
    match_loader.set_value(false);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 15, 90, 90);
    drive.turn_to_point(39, 40, 1);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(9, 90, .7);

    intake.move_voltage(12000);
    driver.multiple = 2;
    driver.cycle_counter++;

    pros::delay(1500);
    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-5, 90, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(5, 9, .7);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.move_to_point(7, 9, true, false, 1.5);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(14, 20, .7);

    driver.multiple++;
    driver.cycle_counter++;

    intake.move_voltage(12000);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(25, 30, 1.5);

    intake.move_voltage(0);

    t_pid.set_t_constants(3.5, 0.0, 10, 600);
    t_pid.translation_pid(-8, 90, .4);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 40, 40);
    drive.move_to_point(13, 30, true, false, 1);

    drive.set_constants(3.5, 0, 10, 2, 0.001, 13, 90, 90);
    drive.turn_to_point(-16, 58, .7);

    intake.move_voltage(-9000);
    pros::delay(350);
    intake.move_voltage(-11000);
    pros::delay(250);
}