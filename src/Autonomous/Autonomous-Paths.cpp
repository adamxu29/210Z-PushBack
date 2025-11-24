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
    // indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    indexer.move_voltage(12000);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(-6, 24, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(6, 36, true, .7);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(6, 36, false, true, .5);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    // drive.turn_to_point(9, 28, false, .4);

    indexer.move_voltage(-11000);
    pros::delay(850);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-31, 3, false, false, 1.3);

    drive.set_constants(5.5, 0, 11, 1.5, 0, 11, 90, 90);
    drive.turn_to_point(-35, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 100, .6);
    pros::delay(350);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-31, 30, true, .4);

    drive.set_constants(5.5, 0, 15, 2, 0, 0, 75, 90);
    drive.move_to_point(-31, 25, false, true, .9);

    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(750);
    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(30, 20, false, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 127);
    drive.move_to_point(27, 20, false, false, 1.7);
    match_loader.set_value(true);
    indexer.move_voltage(12000);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    // drive.move_to_point(36, 23, false, false, .5);
    

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(60, 0, false, .7);
    // match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(58, -3, false, false, .9);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 127, 127);
    drive.turn_to_point(59, -12, false, .4);
    
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(19, 90, .7);
    pros::delay(350);

    drive.set_constants(5.5, 0, 15, 2, 0, 0, 90, 127);
    drive.move_to_point(59, 27, false, true, .8);
    trapdoor.set_value(true);
    match_loader.set_value(false);
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
    drive.move_to_point(-25, 7, true, false, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(-28, 30, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(-28, 24, false, true, .9);

    match_loader.set_value(true);
    trapdoor.set_value(true);
    pros::delay(1500);
    indexer.move_voltage(0);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    // drive.move_to_point(-31, 5, false, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 90, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-29, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(15, 90, .7);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-33, -7, false, false, .5);
    pros::delay(400);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-5, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.turn_to_point(5, 34, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(5, 34, false, true, 1.2);
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
    drive.move_to_point(6, 24, false, false, .5);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(24, 32, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(28, 40, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(10, 45, .5);

    match_loader.set_value(true);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 60, 60);
    drive.move_to_point(10, 5, false, true, 1.2);
    pros::delay(100);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(30, 2, false, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(30, 2, false, false, 1.2);
    pros::delay(100);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(32, -10, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(14, 90, .7);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-33, -7, false, false, .5);
    pros::delay(450);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(31, 30, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(31, 25, false, true, .9);

    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(2000);
    indexer.move_voltage(0);

    t_pid.set_t_constants(9, 0, 11, 600);
    t_pid.translation_pid(3, 90, .2);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(43, 12, true, false, .7);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(43, 34, true, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(42, 37, false, true, .9);
    match_loader.set_value(false);
}

void Eclipse::Autonomous_Paths::left_elims()
{
    util.set_robot_position(0, 0, 90);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(-29, 5, false, true, 1.1);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-31, -14, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 90, .5);
    pros::delay(400);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(-31, 26, true, .3);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.move_to_point(-31, 26, false, true, .9);

    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(1500);
    
    
    // util.set_robot_position(0, 0, -14);

    // intake.move_voltage(12000);
    // // indexer.move_voltage(12000);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    // drive.move_to_point(-3, 15, false, false, .7);
    // match_loader.set_value(true);
    // pros::delay(50);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    // drive.move_to_point(-6, 24, false, false, .5);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.move_to_point(-33, 13, true, false, .9);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.turn_to_point(-33, 35, true, .5);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.move_to_point(-33, 35, false, true, .5);

    // indexer.move_voltage(12000);
    // trapdoor.set_value(true);
    // pros::delay(900);
    // t_pid.set_t_constants(5.5, 0, 11, 600);
    // t_pid.translation_pid(-10, 90, .2);
    // indexer.move_voltage(0);

    // t_pid.set_t_constants(9, 0, 11, 600);
    // t_pid.translation_pid(3, 90, .2);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-19, 12, true, false, .7);
    // trapdoor.set_value(false);
    // wing.set_value(true);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    // drive.turn_to_point(-19, 34, true, .5);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    // drive.move_to_point(-19, 37, false, true, .9);
    // match_loader.set_value(false);
}

void Eclipse::Autonomous_Paths::right_elims(){
    util.set_robot_position(0, 0, 14);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(4, 15, false, false, .6);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(6, 24, false, false, .5);
    // match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(32, 5, true, false, 1.2);
    match_loader.set_value(true);
    pros::delay(250);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(33, -10, false, .4);
    pros::delay(250);
    indexer.move_voltage(0);


    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(14, 75, .7);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-33, -7, false, false, .5);
    pros::delay(450);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 60, 90);
    drive.turn_to_point(32, 30, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(32, 24, false, true, .9);

    indexer.move_voltage(12000);
    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(1700);
    // indexer.move_voltage(0);

    // c_pid.set_c_constants(5, 0, 30);
    // c_pid.curve_pid(-7, 90, 1.5, -.13, false);
    // wing.set_value(true);
    // trapdoor.set_value(false);

    // drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    // drive.move_to_point(26, 39, false, false, .7);

    // drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    // drive.turn_to_point(28, 45, false, .5);

    // match_loader.set_value(true);
    // pros::delay(200);

    // r_pid.set_r_constants(2, 0, 11);
    // r_pid.rotation_pid(-60, 127, .7);
    // match_loader.set_value(false);
}

// Misc paths
void Eclipse::Autonomous_Paths::skills(){
    util.set_robot_position(0, 0, -90);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);
    match_loader.set_value(true);

    // first corner
    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(29, 5, false, true, 1.1);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(30, -14, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 100, .5);
    pros::delay(1250);
    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(5, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(31, 26, true, .3);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 75, 90);
    drive.move_to_point(29, 5, false, true, 1.1);
    match_loader.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(19, 90, .7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(2, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(82, 127, 1.7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .5);

    // second corner
    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(31, 54, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(31, 71, false, true, .7);

    trapdoor.set_value(true);
    match_loader.set_value(true);
    indexer.move_voltage(12000);
    intake.move_voltage(12000);
    pros::delay(1500);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 100, .6);

    trapdoor.set_value(false);
    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(32, 130, false, .5);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(15, 100, .6);
    
    pros::delay(1700);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(31, 54, true, .3);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(31, 72, false, true, .7);

    trapdoor.set_value(true);
    indexer.move_voltage(12000);
    pros::delay(1400);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(5, 90, .4);

    trapdoor.set_value(false);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 127, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(-54, 98, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-54, 98, false, false, 1.7);

    // third corner
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-56, 116, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(18, 100, .6);

    pros::delay(2000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-58, 105, false, true, .7);

    match_loader.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(18, 90, .7);

    intake.move_voltage(0);
    indexer.move_voltage(0);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-178, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(80, 127, 1.7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-12, 90, .5);

    // fourth corner
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-62, 45, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-62, 35, false, true, .7);

    match_loader.set_value(true);
    trapdoor.set_value(true);
    indexer.move_voltage(12000);
    intake.move_voltage(12000);
    pros::delay(2000);

    t_pid.set_t_constants(5.5, 0, 11, 100);
    t_pid.translation_pid(17, 100, .6);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-64, -14, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 90, .5);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .5);

    pros::delay(1700);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-64, 35, false, true, .7);
    trapdoor.set_value(true);
    match_loader.set_value(false);
    pros::delay(2000);

    // mid goal
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.turn_to_point(-38, 36, false, .9);
    trapdoor.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-40, 32, false, false, .9);
    match_loader.set_value(true);
    pros::delay(150);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-24, 48, true, .9);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-24, 48, false, true, .9);
    indexer.move_voltage(-6000);
    pros::delay(1500);
    indexer.move_voltage(0);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(-20, 35, true, false, .9);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-3, 32, true, false, .9);
    match_loader.set_value(true);
    pros::delay(150);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-7, 47, false, .9);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 75, 75);
    drive.move_to_point(-7, 47, false, false, .9);
    intake.move_voltage(-12000);
    pros::delay(2000);

    // go park
    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(5, 15, false, true, .9);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 127);
    drive.move_to_point(5, -5, true, false, .9);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    match_loader.set_value(true);
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(40, 90, 1.5);
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
