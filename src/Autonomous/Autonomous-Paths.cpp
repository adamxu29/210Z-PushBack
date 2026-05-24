#include "main.h"
#include "pros/motors.h"
#include "map"

/*
Odometry coord finder: //https://path.jerryio.com/
*/

using namespace Eclipse;

// Translation PID constants: 3, 0, 30, 100
// Rotation PID constants: 5, 0, 35/28)
// Curve PID constants: 5, 0, 30


void Eclipse::Autonomous_Paths::solo_awp(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    t_pid.set_t_constants(5, 0, 30, 100);
    t_pid.translation_pid(6, 127, .5, false);

    drive.set_constants(3, 0, 40, 5, 0, 28, 127, 90);
    drive.move_to_point(42, -45, false, true, 1.0);

    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(-41, 100, 1.1, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.4, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(14, 90, .8);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 110);
    drive.move_to_point(48, -22, false, true, .85, false);
    
    match_loader.set_value(false);
    scoring.score_until_opp_block(scoring.RunMode::Scoring, 1);

    odom.distance_sensor_reset(2, true, Odom::Front);
    odom.distance_sensor_reset(2, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Top);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(17, -25, false, .7, false);

    odom.distance_sensor_reset(1, true, Odom::Back);
    odom.distance_sensor_reset(1, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 110);
    drive.move_to_point(17, -22 , false, false, .5);
    trapdoor.set_value(true);
    odom.distance_sensor_reset(1, false, Odom::Left);

    drive.set_constants(3, 0, 35, 5, 0, 28, 90, 90);
    drive.move_to_point(-17, -22, false, false, .8);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(-48, -47, false, .3);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(-48, -47, false, false, .85, false);

    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.3, false);

    pros::delay(100);

    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "score: reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Top, 0, 0);
    drive.set_constants(3, 0, 30, 5, 0, 35, 110, 90);
    drive.move_to_point(-48, -22, false, true, .55, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Top);
    trapdoor.set_value(true);
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    trapdoor.set_value(true);
    pros::delay(800);

    match_loader.set_value(true);

    drive.set_constants(3, 0, 30, 5, 0, 0, 90, 90);
    drive.turn_to_point(-45, -66, false, .3);

    indexer.move_voltage(12000);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(31, 70, 1.3, false);
    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad, 600, 0);

    pros::delay(100);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 90, .5, false);
    
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    back_wing.set_value(false);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-10, -10, true, .5);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 90);
    drive.move_to_point(-10, -10, false, true, 1.0);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 90, .2, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 360);
    pros::delay(1200);

    // can cut wing part

    // pros::delay(600);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-37, 14, false, false, .8, false);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.turn_to_point(-37, 37, true, .7);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-37, 37, false, true, .8, false);
}

// 4+3
void Eclipse::Autonomous_Paths::left_half_awp(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    drive.set_constants(3, 0, 40, 5, 0, 28, 127, 90);
    drive.move_to_point(-43, -45, false, true, 0.7);

    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(-41, 100, 1.1, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.4, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(14, 90, .8);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 110);
    drive.move_to_point(-48, -22, false, true, .85, false);
    
    match_loader.set_value(false);
    scoring.score_until_opp_block(scoring.RunMode::Scoring, 1);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(-20, -20, false, .6);
    trapdoor.set_value(true);

    odom.distance_sensor_reset(1, true, Odom::Back);
    odom.distance_sensor_reset(1, true, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 110, 110);
    drive.move_to_point(-20, -20, false, false, .5);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(9, -9, true, true, 1.2, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 460);
    pros::delay(800);
    scoring.set_scoring_mode(scoring.ScoringMode::None);

    back_wing.set_value(true);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 90, .5);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-7, 90, .7);



    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    // drive.turn_to_point(38, -9, false, .5);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(38, -9, false, false, .9, false);

    // scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    // pros::delay(250);
    // odom.distance_sensor_reset(1, true, Odom::Front);
    // std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;


    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-35, -25, false, true, 1.2);




    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    // drive.turn_to_point(-35, 0, true, .5);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 60, 90);
    // drive.move_to_point(-35, -9, false, true, .9, false);
}

void Eclipse::Autonomous_Paths::right_half_awp(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    drive.set_constants(3, 0, 40, 5, 0, 28, 127, 90);
    drive.move_to_point(42, -45, false, true, .6);

    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(-41, 100, 1.1, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.4, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(14, 90, .8);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(250);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 110);
    drive.move_to_point(48, -22, false, true, .85, false);
    
    match_loader.set_value(false);
    scoring.score_until_opp_block(scoring.RunMode::Scoring, 1);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Top, 600, 0);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(17, -25, false, .6, false);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600 ,0);

    pros::delay(100);

    odom.distance_sensor_reset(2, true, Odom::Back);
    odom.distance_sensor_reset(2, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(20, -20, false, false, .3);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 600);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(10, -8, true, false, 0.8);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(5, 90, .4);

    scoring.set_scoring_mode(scoring.ScoringMode::Bottom, 345, 600);
    pros::delay(1200);
    scoring.set_scoring_mode(scoring.ScoringMode::Outtake);

    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(-7, 90, .7);

    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    // drive.turn_to_point(46, -5, false, .7);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 60, 90);
    // drive.move_to_point(46, -5, false, false, .9, false);

    // scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    // pros::delay(400);
    // odom.distance_sensor_reset(1, true, Odom::Front);
    // std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    front_wing.set_value(true);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(34, -30, false, true, 0.9, false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(34, 0, false, .5);

    drive.set_constants(3, 0, 30, 5, 0, 28, 60, 90);
    drive.move_to_point(34, -9, false, false, .9, false);
}

// 9 split
void Eclipse::Autonomous_Paths::right_9(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(22, -15, false, false, .8);

    drive.set_constants(3, 0, 30, 5, 0, 28, 85, 90);
    drive.move_to_point(32, -8, false, false, .4);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 75);
    drive.move_to_point(49, -9, true, false, .8, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    pros::delay(250);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    drive.move_to_point(38, -38, true, true, 1.0);

    drive.set_constants(3, 0, 30, 5, 0, 35, 127, 127);
    drive.move_to_point(56, -48, true, true, 1.0 ,false);

    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.5, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(19, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(700);

    drive.set_constants(3, 0, 30, 5, 0, 35, 110, 90);
    drive.move_to_point(48, -20, false, true, .8, false);
    
    match_loader.set_value(false);
    scoring.set_scoring_mode(scoring.ScoringMode::Top, -600, 600);
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(300); // tune delay based on split
    scoring.set_scoring_mode(scoring.ScoringMode::Top, 300, 600);
    pros::delay(350);
    // 400 ~ 6 blocks
    // 200 ~ 4 blocks

    scoring.set_scoring_mode(scoring.ScoringMode::Outtake, 0, 600);
    match_loader.set_value(false);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(17, -25, false, .6, false);

    pros::delay(100);
    trapdoor.set_value(false);

    odom.distance_sensor_reset(2, false, Odom::Back);
    odom.distance_sensor_reset(2, true, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(9, -11, true, false, 1.1, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(10, 90, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Bottom, 400, 600);
    pros::delay(1500);
    scoring.set_scoring_mode(scoring.ScoringMode::Outtake);
}

void Eclipse::Autonomous_Paths::left_9(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(-22, -15, false, false, .8);

    drive.set_constants(3, 0, 30, 5, 0, 28, 85, 90);
    drive.move_to_point(-32, -11, false, false, .4);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 75);
    drive.move_to_point(-48, -10 , true, false, .8, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    pros::delay(250);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 110);
    drive.move_to_point(-35, -37, true, true, 1.0);

    drive.set_constants(3, 0, 30, 5, 0, 35, 110, 110);
    drive.move_to_point(-50, -45, true, true, 1.0 ,false);

    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.5, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(18, 70, .7);
    
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(700);
    // scoring.set_scoring_mode(scoring.ScoringMode::Top, 0, 0);

    drive.set_constants(3, 0, 30, 5, 0, 35, 100, 90);
    drive.move_to_point(-47, -20, false, true, .74, false);
    
    scoring.set_scoring_mode(scoring.ScoringMode::Top, -600, 600);
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(300); // tune delay based on split
    scoring.set_scoring_mode(scoring.ScoringMode::Top, 0, 600);
    pros::delay(350);
    // 400 ~ 6 blocks
    // 200 ~ 4 blocks
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 0, -400);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(15, 90, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::None);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.turn_to_point(-10, -10, true, .4, false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 105, 90);
    drive.move_to_point(-10, -10, false, true, 1.2);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 90, .3, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 400);
    pros::delay(1000);
    trapdoor.set_value(true);
    pros::delay(500);
    scoring.set_scoring_mode(scoring.ScoringMode::None);
    
    back_wing.set_value(true);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 90, .5);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-7, 90, .7);
}
// 7 wing
void Eclipse::Autonomous_Paths::right_7(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(22, -22, false, false, .55);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(48, -48, true, false, 1.6);

    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.4, false);
    
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(19, 75, 0.9, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    // pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 110, 110);
    drive.move_to_point(48, -22, false, true, .8, false);
    
    match_loader.set_value(false);
    scoring.score_until_opp_block(scoring.RunMode::Scoring, 1.2);

    odom.distance_sensor_reset(5, true, Eclipse::Odom::Front);
    odom.distance_sensor_reset(5, false, Eclipse::Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // wing for right side (not tuned)
    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    drive.move_to_point(51, -41, false, false, .5);

    back_wing.set_value(true);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    drive.move_to_point(56.5, -20, true, true, .8);

    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(odom.get_robot_x(), -7, false, true, .6, false);

    trapdoor.set_value(true);
}

void Eclipse::Autonomous_Paths::left_7(){
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Back);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-21, -22, false, false, .55);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(-49, -48, true, false, 1.7);

    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(180, 110, 0.4, false);
    
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(19, 75, 1.2, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 110, 110);
    drive.move_to_point(-48, -22, false, true, .8, false);
    
    match_loader.set_value(false);
    scoring.score_until_opp_block(scoring.RunMode::Scoring, 2);

    odom.distance_sensor_reset(5, true, Eclipse::Odom::Front);
    odom.distance_sensor_reset(5, false, Eclipse::Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // wing for right side (not tuned)
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(-43, -40, false, false, .44);

    back_wing.set_value(true);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    drive.move_to_point(-41, -20, true, true, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 70, 90);
    drive.move_to_point(odom.get_robot_x(), -7, false, true, .6, false);
    trapdoor.set_value(true);
}

void Eclipse::Autonomous_Paths::skills(){
    // start in park zone
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    odom_lift.set_value(false);

    // clear park
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(5, 75, .4, false);

    pros::delay(300);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-4, 60, .3);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(10, 60, .5, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-4, 60, .3);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(10, 60, .8, true);
    odom_lift.set_value(false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-24, 75, 1.5, false);

    // reset and intake 7th block
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 110, 110);
    drive.move_to_point(-7, -14, false, true, .7, false);
    match_loader.set_value(false);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-21, -25, false, .5, false);
    trapdoor.set_value(false);

    scoring.set_color(scoring.Color::Red);
    left_drive.move_voltage(3000);
	right_drive.move_voltage(3000);
	scoring.score_until_opp_block(scoring.RunMode::Intake, 1, 290, 0);
    left_drive.move_voltage(0);
	right_drive.move_voltage(0);
    scoring.set_color(scoring.Color::None);

    // dirve to mid goal
    scoring.set_scoring_mode(scoring.ScoringMode::Outtake, 300, 0);
    pros::delay(50);
    scoring.set_scoring_mode(scoring.ScoringMode::None);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-7, -11, true, .4);

    mid_goal.set_value(true);

    drive.set_constants(3, 0, 30, 5, 0, 28, 110, 110);
    drive.move_to_point(-7, -11, false, true, 1.3, false);

    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(-15, 90, .9);

    std::cout << "mid goal " << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // score mid goal
    std::cout << "i: " << pros::millis() << std::endl;
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 460);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(1, 70, .3, false);
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 230);
    pros::delay(1500);
    while(true){
        if((indexer.get_current_draw() < 450) && (indexer.get_voltage() > 4900)){
            break;
        }
        pros::delay(10);
    }
    std::cout << "c: " << indexer.get_current_draw() << "v: " << indexer.get_voltage() << std::endl;
    std::cout << "f: " << pros::millis() << std::endl;
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 115);
    pros::delay(500);

    // match_loader first corner
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(10, 90, .4);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    mid_goal.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-43, -50, false, false, .9, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-41, -66, false, .6, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(13, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(400);

    r_pid.set_r_constants(3, 0, 30);
    r_pid.rotation_pid(175, 127, .5);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-60, -29, true, true, 1.0);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-60, 28, false, true, 1.3, false);
    
    left_drive.move_voltage(0);
    right_drive.move_voltage(0);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    std::cout << "jew reset";
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-50, 43, true, .5);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-50, 43, true, true, .9, false);

    r_pid.set_r_constants(3, 0, 35);
    r_pid.rotation_pid(4, 90, .5, false);

    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-16, 90, .8, false);

    // score 1
    scoring.set_scoring_mode(scoring.ScoringMode::Top);
    pros::delay(1250);
    match_loader.set_value(true);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-49, 60, false, 0.3);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    // match load second corner
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(30, 70, .9);
    trapdoor.set_value(false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(800);

    // score 12-15 blocks
    drive.set_constants(3, 0, 30, 5, 0, 8, 100, 127);
    drive.move_to_point(-48, 22, false, true, 1.0, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(1400);

    match_loader.set_value(false);

    // drive to park zone
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-5, 46, false, .7);

    std::cout << "park zone: ";
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 75);
    drive.move_to_point(-5, 46, false, false, 1.3, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    r_pid.set_r_constants(5, 0, 35);
    r_pid.rotation_pid(0, 90, .7);
    
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left); // change to right
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;







    // 2nd half
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 40, .4);
    odom_lift.set_value(true);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(8, 60, .5, false);
    trapdoor.set_value(false);

    pros::delay(1000);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 75, .7);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 60, .3, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 75, .5, false);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 60, .5);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(10, 60, .7, true);
    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad, 600, 0);
    odom_lift.set_value(false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-17, 90, .9, false);

    // reset and intake 7th block
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    // intake 7th block
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-5, 21, false, true, 1.4, false);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 75);
    drive.turn_to_point(-17, 20.5, false, .6, false);

    scoring.set_color(scoring.Color::Blue);
    left_drive.move_voltage(3000);
	right_drive.move_voltage(3000);
	scoring.score_until_opp_block(scoring.RunMode::Intake, 1, 290, 0);
    left_drive.move_voltage(0);
	right_drive.move_voltage(0);
    scoring.set_color(scoring.Color::None);

    scoring.set_scoring_mode(scoring.ScoringMode::None);
    
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(0, 0, false, .6, false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(9, 12, false, false, .9);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(8, 30, .4);

    std::cout << "mid goal " << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // score lower goal
    std::cout << "i: " << pros::millis() << std::endl;
    intake_lift.set_value(true);
    scoring.set_scoring_mode(scoring.ScoringMode::Bottom, 600, 600);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-2.5, 70, .4, false);
    scoring.set_scoring_mode(scoring.ScoringMode::Bottom, 200, 400);
    pros::delay(3000);
    // while(true){
    //     if((intake.get_current_draw() < 450) && (intake.get_voltage() < -4350)){
    //         break;
    //     }
    //     pros::delay(100);
    // }
    pros::delay(500);
    std::cout << "c: " << intake.get_current_draw() << "v: " << intake.get_voltage() << std::endl;
    std::cout << "f: " << pros::millis() << std::endl;
    




    // match load third corner
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 75, .7);
    
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(15, 23, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 90);
    drive.move_to_point(15, 23, false, false, .9);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 46, true, false, 1.7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(48, 64, false, .7);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(17, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(700);

    r_pid.set_r_constants(3, 0, 30);
    r_pid.rotation_pid(-5, 127, .5);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(62, 28, true, .6);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(62, 28, false, true, .9, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(62, -29, false, true, 1.5, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;


    std::cout << "last corner ";
    // match load fourth corner
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(49, -45, true, .6);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(49, -45, false, true, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad, 600, 0);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(49, -66, false, .7, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(15, 60, .7);

    pros::delay(700);

    r_pid.set_r_constants(5, 0, 28);
    r_pid.rotation_pid(170, 90, .4);

    r_pid.set_r_constants(5, 0, 28);
    r_pid.rotation_pid(190, 90, .4);

    r_pid.set_r_constants(5, 0, 28);
    r_pid.rotation_pid(180, 90, .4);

    t_pid.set_t_constants(10, 0, 30, 100);
    t_pid.translation_pid(10, 90, .3);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;


    // score
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(48, -23, false, true, 1.2);

    scoring.set_scoring_mode(scoring.ScoringMode::Top, 0, 600);
    pros::delay(50);
    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    odom.distance_sensor_reset(10, true, Odom::Front);
    odom.distance_sensor_reset(10, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(750);

    scoring.set_scoring_mode(scoring.ScoringMode::Outtake);
    pros::delay(100);
    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(5, 90, .4);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-7, 90, .4);

    pros::delay(1750);

    match_loader.set_value(false);

    // go park
    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    drive.move_to_point(35, -53, false, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 35, 127, 127);
    drive.move_to_point(10, -61, true, false, 1);

    odom_lift.set_value(true);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(13, 127, .9);
}

void Eclipse::Autonomous_Paths::test(){
    // t_pid.set_t_constants(3, 0, 30, 100);
    // t_pid.translation_pid(5, 75, .7);
    
}
