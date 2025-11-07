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
    util.set_robot_position(0, 0);
    imu1.set_rotation(-90);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-27, 5, false, false, 1.1);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(-29, -14, false, .4);

    match_loader.set_value(true);
    pros::delay(150);
    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-29, -8, false, false, .9);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(5, 90, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    match_loader.set_value(false);
    intake.move_voltage(12000);
    indexer.move_voltage(0);
    trapdoor.set_value(true);
    pros::delay(100);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(-29, 25, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-29, 25, false, true, .7);

    indexer.move_voltage(12000);

    pros::delay(800);
    indexer.move_voltage(0);

    t_pid.set_t_constants(9, 0, 11, 600);
    t_pid.translation_pid(10, 90, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-21, 14, false, true, .9);

    trapdoor.set_value(false);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-9, 25, false, false, 1.7);

    pros::delay(250);

    // t_pid.set_t_constants(5.5, 0, 11, 600);
    // t_pid.translation_pid(7, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(0, 36, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(0, 36, false, true, .7);

    indexer.move_voltage(-12000);
    intake.move_voltage(-11000);
    intake.move_voltage(0);
    pros::delay(650);
    indexer.move_voltage(0);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(10, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(18, 25, true, false, .7);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(36, 22, false, false, 1.2);

    pros::delay(250);

    // t_pid.set_t_constants(5.5, 0, 11, 600);
    // t_pid.translation_pid(5, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 80, 90);
    drive.move_to_point(61, 7, true, false, .9);

    indexer.move_voltage(0);
    trapdoor.set_value(true);
    pros::delay(100);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(61, 33, false, true, .9);

    indexer.move_voltage(12000);
}

void Eclipse::Autonomous_Paths::left_half_awp(){

}

void Eclipse::Autonomous_Paths::right_half_awp(){
}

void Eclipse::Autonomous_Paths::left_elims(){
    util.set_robot_position(0, 0);
    imu1.set_rotation(-17);

    intake.move_voltage(12000);
    // indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    pros::delay(50);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(-7, 24, false, false, .9);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(-24, 30.5, false, false, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.turn_to_point(-30, 40, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 30);
    drive.move_to_point(-28, 37, false, false, 1.5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 30);
    drive.move_to_point(-30, 40, false, false, 2);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 75);
    drive.move_to_point(-5, 30, false, true, .75);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(5, 38, true, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 60);
    drive.move_to_point(4, 39, false, true, .7);

    indexer.move_voltage(-12000);
    pros::delay(500);
    indexer.move_voltage(0);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-32, 5, false, false, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-33, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(12, 90, .5);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-33, -7, false, false, .5);
    pros::delay(500);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-32, 30, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.move_to_point(-32, 24, false, true, .7);

    pros::delay(500);

    t_pid.set_t_constants(9, 0, 11, 600);
    t_pid.translation_pid(2, 90, .2);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 127, 75);
    drive.move_to_point(-22, 19, true, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-23.4, 34, true, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 127, 75);
    drive.move_to_point(-23.5, 34, true, true, .7);
}

void Eclipse::Autonomous_Paths::right_elims(){
    
}

// Misc paths
void Eclipse::Autonomous_Paths::skills(){
    util.set_robot_position(0, 0);
    imu1.set_rotation(90);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(26.4, 5, false, false, 1.1);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(32, -14, false, .4);

    match_loader.set_value(true);
    pros::delay(150);
    intake.move_voltage(12000);
    indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 80, 90);
    drive.move_to_point(29, -7, false, false, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .5);

    pros::delay(1300);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-18, 90, .7);

    indexer.move_voltage(0);
    match_loader.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 90, .7);

    intake.move_voltage(0);
    indexer.move_voltage(0);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(0, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(82, 127, 1.7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(32, 54, true, .4);

    trapdoor.set_value(true);
    pros::delay(250);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(32, 71, false, true, .7);

    indexer.move_voltage(12000);
    pros::delay(200);
    intake.move_voltage(12000);
    pros::delay(1300);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(32, 120, false, .3);

    trapdoor.set_value(false);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 80, 90);
    drive.move_to_point(33, 108, false, false, 1.2);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .5);

    pros::delay(1300);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    indexer.move_voltage(0);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.turn_to_point(32, 54, true, .3);

    trapdoor.set_value(true);
    pros::delay(100);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(32, 72, false, true, .7);

    indexer.move_voltage(12000);

    pros::delay(1400);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(10, 90, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-15, 127, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(7, 90, .4);

    trapdoor.set_value(false);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-58, 90, false, false, 2.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-62, 120, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.move_to_point(-62, 111, false, false, 1.2);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .5);

    pros::delay(1400);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    match_loader.set_value(false);
    
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 80, 90);
    // drive.turn_to_point(-62, 75, true, .4);

    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 80, 90);
    // drive.move_to_point(-62,  73, false, true, .9);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(18, 90, .7);

    intake.move_voltage(0);
    indexer.move_voltage(0);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-178, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(82, 127, 1.7);

    r_pid.set_r_constants(2, 0, 11);
    r_pid.rotation_pid(-90, 90, .7);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 90);
    drive.turn_to_point(-62, 45, true, .7);

    trapdoor.set_value(true);
    pros::delay(250);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 90);
    drive.move_to_point(-67, 31, false, true, .7);

    indexer.move_voltage(12000);
    pros::delay(200);
    intake.move_voltage(12000);
    pros::delay(1300);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 90, .5);

    r_pid.set_r_constants(2, 0, 0);
    r_pid.rotation_pid(-180, 90, .3);

    match_loader.set_value(true);
    pros::delay(150);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(17, 90, .5);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(6, 90, .5);

    pros::delay(1300);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    indexer.move_voltage(0);

    r_pid.set_r_constants(2, 0, 0);
    r_pid.rotation_pid(-180, 90, .3);

    trapdoor.set_value(true);
    pros::delay(250);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-17, 90, .7);

    indexer.move_voltage(12000);
    pros::delay(1500);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(15, 127, 1);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .4);

    indexer.move_voltage(0);
    intake.move_voltage(0);
    match_loader.set_value(false);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-43, -11, false, false, 1.2);

    r_pid.set_r_constants(2, 0, 0);
    r_pid.rotation_pid(87, 90, .7);

    intake.move_voltage(-12000);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(30, 127, 1.2);
}

void Eclipse::Autonomous_Paths::test(){
    util.set_robot_position(0, 0);
    imu1.set_rotation(-17);

    intake.move_voltage(12000);
    // indexer.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(-3, 15, false, false, .7);
    pros::delay(50);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 45, 45);
    drive.move_to_point(-6, 24, false, false, .9);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(4, 36, true, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 60);
    drive.move_to_point(4, 36, false, true, .7);

    indexer.move_voltage(-12000);
    pros::delay(500);
    indexer.move_voltage(0);
    match_loader.set_value(true);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    drive.move_to_point(-31, 5, false, false, 1.2);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-32, -7, false, .4);

    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(12, 90, .5);
    // drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 90);
    // drive.move_to_point(-33, -7, false, false, .5);
    pros::delay(500);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(-31, 30, true, .4);

    drive.set_constants(5.5, 0, 11, 2, 0, 0, 90, 90);
    drive.move_to_point(-31, 24, false, true, .7);

    match_loader.set_value(false);
    pros::delay(500);

    t_pid.set_t_constants(9, 0, 11, 600);
    t_pid.translation_pid(5, 90, .5);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 90, 60);
    drive.turn_to_point(43, 18, false, .7);
    intake.move_voltage(12000);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 127);
    drive.move_to_point(30, 21, false, false, 3);
    pros::delay(50);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 75, 75);
    drive.move_to_point(36, 21, false, false, .9);
    match_loader.set_value(true);
    pros::delay(200);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 127, 127);
    drive.turn_to_point(57, 0, false, .7);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 127, 127);
    drive.move_to_point(57, 0, false, false, .9);

    drive.set_constants(5.5, 0, 11, 2, 0, 11, 127, 127);
    drive.turn_to_point(57, -12, false, .4);
    
    t_pid.set_t_constants(5.5, 0, 11, 600);
    t_pid.translation_pid(12, 90, .5);
    pros::delay(500);

    drive.set_constants(5.5, 0, 11, 2, 0, 8, 90, 127);
    drive.move_to_point(59, 23, false, true, .9);
}