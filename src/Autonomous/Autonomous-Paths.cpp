#include "main.h"
#include "pros/motors.h"
#include "map"

/*
Odometry coord finder: //https://path.jerryio.com/
*/

using namespace Eclipse;

// Translation PID constants: 5.5, 0, 11, 100
// Rotation PID constants: 2, 0, (0 (~20), 8 (~45), 11 (90))
// Curve PID constants: 5, 0, 30

void Eclipse::Autonomous_Paths::solo_awp(){
    util.set_robot_position(0, 0, -14);

    intake.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    indexer.move_voltage(12000);
    match_loader.set_value(true);
    pros::delay(150);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-27, 39, false, false, .9, false);

    match_loader.set_value(true);
    pros::delay(225);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(5, 20, false, true, .7);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(10, 41, true, true, .9, false);

    indexer.move_voltage(-8500);
    pros::delay(450);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-31, 5, false, false, 1.3, false);

    drive.set_constants(5.5, 0, 11, 1.5, 0, 11, 90, 90);
    drive.turn_to_point(-33, -7, false, .4, false);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(15, 90, .6);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-32, 30, true, .4, false);

    drive.set_constants(5.5, 0, 15, 2, 0, 0, 75, 90);
    drive.move_to_point(-32, 25, false, true, .9, false);

    trapdoor.set_value(true);
    match_loader.set_value(false);
    t_pid.set_t_constants(10, 0, 11, 100);
    t_pid.translation_pid(-5, 90, .4);
    pros::delay(550);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.turn_to_point(31, 23, false, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 127);
    drive.move_to_point(33, 23, false, false, 1.7, false);
    match_loader.set_value(true);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(55, 0, false, false, .9, false);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(58, -20, false, .7, false);
    
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(16, 90, .7);
    pros::delay(200);

    drive.set_constants(5.5, 0, 15, 2, 0, 0, 90, 127);
    drive.move_to_point(60, 27, false, true, .8, false);
    trapdoor.set_value(true);
    match_loader.set_value(false);

    t_pid.set_t_constants(10, 0, 11, 100);
    t_pid.translation_pid(-5, 127, .4);
}

void Eclipse::Autonomous_Paths::left_half_awp(){
    util.set_robot_position(0, 0, -14);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(-6, 24, false, false, .5);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(-24, 32, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(-28, 40, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(10, 45, .5);

    match_loader.set_value(true);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 60, 60);
    drive.move_to_point(-15, 7, false, true, 1.2);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-26, 7, true, false, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(-30, 30, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(-30, 24, false, true, .9);

    match_loader.set_value(true);
    trapdoor.set_value(true);
    pros::delay(1500);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-30, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 90, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-30, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(15, 90, .7);
    pros::delay(400);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.turn_to_point(4, 38, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(4, 38, false, true, 1.2);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .7);
    indexer.move_voltage(-8000);
    pros::delay(100);

    match_loader.set_value(false); 
}

void Eclipse::Autonomous_Paths::right_half_awp(){
    util.set_robot_position(0, 0, 14);
    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(4, 15, false, false, .6);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(6, 18, false, false, .5);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(24, 37, false, false, .9);
    match_loader.set_value(true);

    // drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    // drive.turn_to_point(29, 40, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(4, 60, .7);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 60, 60);
    drive.move_to_point(15, 7, false, true, 1.2);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(31, 7, true, false, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(31, 30, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(31, 24, false, true, .9);

    match_loader.set_value(true);
    trapdoor.set_value(true);
    pros::delay(1000);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(30, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(18, 90, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(30, -7, false, .4);
    intake.move_voltage(8000);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(18, 75, .7);
    pros::delay(275);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-8, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.turn_to_point(-4, 38, false, .7);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-4, 38, false, false, 1.2);
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-2, 90, .2);
    intake.move_voltage(-5500);
}

void Eclipse::Autonomous_Paths::left_7()
{   
    util.set_robot_position(0, 0, -14);

    intake.move_voltage(12000);
    // indexer.move_voltage(12000);
    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    match_loader.set_value(true);
    pros::delay(50);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(-6, 24, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(-31, 5, true, false, .9);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-33, -10, false, .4);
    pros::delay(250);
    indexer.move_voltage(0);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(14, 75, .7);
    pros::delay(450);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-33, 35, true, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(-31, 35, false, true, .5);

    indexer.move_voltage(12000);
    trapdoor.set_value(true);
    pros::delay(900);
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .2);
    indexer.move_voltage(0);

    t_pid.set_t_constants(9, 0, 11, 600);
    t_pid.translation_pid(3, 90, .2);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-19, 12, true, false, .7);
    trapdoor.set_value(false);
    wing.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-19, 34, true, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-19, 37, false, true, .9);
    match_loader.set_value(false);
}

void Eclipse::Autonomous_Paths::right_7(){
    util.set_robot_position(0, 0, 14);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(4, 15, false, false, .6);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(6, 24, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(32, 5, true, false, 1.2, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(34, -10, false, .4, false);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(16, 75, .7);
    pros::delay(450);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(32, 30, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(32, 24, false, true, .9);
    intake.move_voltage(-12000);
    pros::delay(200);
    intake.move_voltage(12000);

    indexer.move_voltage(12000);
    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(850);
    t_pid.set_t_constants(10, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);
    indexer.move_voltage(0);

    // go under goal
    c_pid.set_c_constants(5, 0, 30);
    c_pid.curve_pid(-10, 90, 1.5, -.13, false);

    wing.set_value(true);
    trapdoor.set_value(false);

    intake.move_voltage(12000);
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.move_to_point(26, 39.2, false, false, .7);
    match_loader.set_value(true);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .2);
    pros::delay(200);
    intake.move_voltage(6000);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-60, 127, .7);
    match_loader.set_value(false);
    wing.set_value(false);
    intake.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(5, 38, true, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-3, 42, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 60);
    drive.move_to_point(-3, 42, false, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-5, 90, .3);

    intake.move_voltage(-9000);
    pros::delay(500);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(10, 127, .6);

    wing.set_value(true);
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(29, 23, false, true, .7, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.turn_to_point(28, 40, false, .7, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(20, 90, 1.3);
}

// Misc paths
void Eclipse::Autonomous_Paths::skills(){
    util.set_robot_position(0, 0, -90);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    // first corner
    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(28.6, 5, false, true, 1.1, false);
    match_loader.set_value(true);
    pros::delay(150);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(30, -14, false, .7, false);
    pros::delay(250);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(16, 75, .7);
    pros::delay(750);
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(5, 90, .5);
    pros::delay(750);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(31, 26, true, .3, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(29, 5, false, true, 1.1, false);
    match_loader.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(19, 90, .7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(3, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(82, 127, 1.7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-12, 90, .5);

    // second corner
    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(5, 90, .7);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.turn_to_point(33, 54, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(32, 71, false, true, .7, false);

    trapdoor.set_value(true);
    match_loader.set_value(true);
    indexer.move_voltage(12000);
    intake.move_voltage(12000);
    pros::delay(1750);
    t_pid.set_t_constants(8, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(33, 130, false, .5, false);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 100, .6);

    trapdoor.set_value(false);
    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(33, 130, false, .5, false);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 75, .8);
    pros::delay(500);
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(5, 90, .5);
    pros::delay(1750);
    // t_pid.set_t_constants(5.5, 0, 11, 100);
    // t_pid.translation_pid(5, 90, .5);
    // pros::delay(700);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(32, 54, true, .3, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(32, 72, false, true, .7, false);

    trapdoor.set_value(true);
    indexer.move_voltage(12000);
    pros::delay(1500);
    t_pid.set_t_constants(10, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(-54, 98, false, .7, false);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-53, 104, false, false, 1.7, false);

    // third corner
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-56, 116, false, .7, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(13, 60, .8);
    pros::delay(1000);
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(5, 90, .5);
    pros::delay(750);
    // t_pid.set_t_constants(5.5, 0, 11, 100);
    // t_pid.translation_pid(5, 90, .5);
    // pros::delay(750);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-58, 105, false, true, .7, false);

    match_loader.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(20, 90, .7);
    pros::delay(150);

    intake.move_voltage(0);
    indexer.move_voltage(0);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-177, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(85, 127, 1.9);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .5);

    // fourth corner
    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(180, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-63, 35, false, true, .7, false);

    match_loader.set_value(true);
    trapdoor.set_value(true);
    indexer.move_voltage(12000);
    intake.move_voltage(12000);
    pros::delay(2000);
    t_pid.set_t_constants(10, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-64, -14, false, .7, false);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(18, 100, .6);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-65, -14, false, .7, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(18, 75, .8);
    pros::delay(1700);
    // t_pid.set_t_constants(5.5, 0, 11, 100);
    // t_pid.translation_pid(5, 90, .5);
    // pros::delay(1050);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(-64, 35, true, .4, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 85, 90);
    drive.move_to_point(-64, 35, false, true, .9), false;
    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(2000);
    t_pid.set_t_constants(10, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    // mid goal
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.turn_to_point(-36, 38, false, .7, false);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 50, 50);
    drive.move_to_point(-36, 38, false, false, .8, false);
    match_loader.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-5, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-30, 48, true, .9, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-30, 48, false, true, .9, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-7, 90, .4);

    indexer.move_voltage(-7500);
    pros::delay(1250);
    indexer.move_voltage(0);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(-20, 34, true, false, .9, false);
    intake.move_voltage(6000);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 65, 65);
    drive.move_to_point(6, 34, true, false, 1.5, false);
    match_loader.set_value(true);
    pros::delay(150);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-5, 50, false, .9, false);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-5, 51, false, false, .9, false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-5, 90, .2);
    intake.move_voltage(-7500);
    pros::delay(1000);

    // go park
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(5, 15, false, true, .9, false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.move_to_point(7, -5, true, false, .9, false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-94, 90, .7);

    match_loader.set_value(true);
    pros::delay(250);
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(25, 127, 2.5);
    match_loader.set_value(false);
    pros::delay(500);
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(16.7, 127, 1.5);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-15, 127, .4);
}

void Eclipse::Autonomous_Paths::test(){
    // util.set_robot_position(0, 0, -14);

    // intake.move_voltage(12000);
    // indexer.move_voltage(12000);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    // drive.move_to_point(-3, 15, false, false, .7);
    // match_loader.set_value(true);
    // pros::delay(50);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    // drive.move_to_point(-6, 24, false, false, .5);
    // // match_loader.set_value(false);

    // // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    // // drive.move_to_point(-22, 33, false, false, .5);

    // // drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    // // drive.turn_to_point(-28, 40, false, .7);

    // // t_pid.set_t_constants(5.5, 0, 11, 100);
    // // t_pid.translation_pid(12, 45, .5);

    // // match_loader.set_value(true);
    // // pros::delay(100);

    // // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // // drive.move_to_point(-5, 30, false, true, 1.2);

    // // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // // drive.turn_to_point(6, 39, true, .7);

    // // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 60);
    // // drive.move_to_point(6, 39, false, true, .7);

    // // indexer.move_voltage(-11000);
    // // pros::delay(750);
    // // indexer.move_voltage(12000);
    // // match_loader.set_value(true);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.move_to_point(-31, 5, false, false, .9);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.turn_to_point(-29, -15, false, .4);
    // r_pid.set_r_constants(2, 0, 11);
    // r_pid.rotation_pid(-180, 90, .7);

    // t_pid.set_t_constants(5.5, 0, 11, 600);
    // t_pid.translation_pid(15, 90, .7);
    // // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // // drive.move_to_point(-33, -7, false, false, .5);
    // pros::delay(350);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    // drive.turn_to_point(-30, 30, true, .4);

    // drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    // drive.move_to_point(-31, 30, false, true, 1.2);

    // trapdoor.set_value(true);
    // pros::delay(1500);
    // t_pid.set_t_constants(5.5, 0, 11, 600);
    // t_pid.translation_pid(-10, 90, .2);
    // indexer.move_voltage(0);

    // t_pid.set_t_constants(9, 0, 11, 600);
    // t_pid.translation_pid(3, 90, .2);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-21, 12, true, false, .7);
    // trapdoor.set_value(false);
    // wing.set_value(true);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.turn_to_point(-20, 34, true, .5);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    // drive.move_to_point(-20, 37, false, true, .9);
    // match_loader.set_value(false);
}

// 9 ball right
// util.set_robot_position(0, 0, 14);
// intake.move_voltage(12000);
// indexer.move_voltage(12000);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
// drive.move_to_point(4, 15, false, false, .6);
// match_loader.set_value(true);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
// drive.move_to_point(6, 24, false, false, .5);
// match_loader.set_value(false);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
// drive.move_to_point(24, 32, false, false, .5);

// drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
// drive.turn_to_point(28, 40, false, .7);

// t_pid.set_t_constants(5.5, 0, 11, 100);
// t_pid.translation_pid(10, 45, .5);

// match_loader.set_value(true);
// pros::delay(200);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 60, 60);
// drive.move_to_point(10, 5, false, true, 1.2);
// pros::delay(100);

// drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
// drive.turn_to_point(30, 2, false, .4);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
// drive.move_to_point(30, 2, false, false, 1.2);
// pros::delay(100);

// drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
// drive.turn_to_point(32, -10, false, .7);

// t_pid.set_t_constants(5.5, 0, 11, 100);
// t_pid.translation_pid(14, 90, .7);
// // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
// // drive.move_to_point(-33, -7, false, false, .5);
// pros::delay(450);

// drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
// drive.turn_to_point(31, 30, true, .4);

// drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
// drive.move_to_point(31, 25, false, true, .9);

// trapdoor.set_value(true);
// match_loader.set_value(false);
// pros::delay(2000);
// indexer.move_voltage(0);

// t_pid.set_t_constants(9, 0, 11, 600);
// t_pid.translation_pid(3, 90, .2);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
// drive.move_to_point(43, 12, true, false, .7);
// trapdoor.set_value(false);

// drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
// drive.turn_to_point(43, 34, true, .5);

// drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
// drive.move_to_point(42, 37, false, true, .9);
// match_loader.set_value(false);