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
    odom.set_robot_position(16, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 5, false, true, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(46, -20, false, .7, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(12, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 23, false, true, .8, false);
    
    match_loader.set_value(false);
    // pros::delay(500);
    util.score_until_opp_block(1);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(20, 27, false, .9);

    trapdoor.set_value(true);
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(20, 22, false, false, .7);
    trapdoor.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-17, 22, false, false, 1.2);

    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(-43, 7, true, false, 1.2);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-43, 30, true, .9);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.move_to_point(-43, 20, false, true, .7);

    scoring.set_scoring_mode(scoring.ScoringMode::Top);
    intake.move_voltage(12000);
    indexer.move_voltage(12000);
    r_pid.set_r_constants(10, 0, 30);
    r_pid.rotation_pid(180, 90, .4);
    pros::delay(800);
    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-45, -20, false, .7);

    indexer.move_voltage(12000);
    t_pid.set_t_constants(3, 0, 35, 100);
    t_pid.translation_pid(29, 75, 1.2);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(100);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 90, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-10, 37, true, .7);
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-10, 39, false, true, 1.2, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 90, .4);

    scoring.set_scoring_mode(scoring.ScoringMode::Middle);

    // can cut wing part

    // pros::delay(600);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-37, 14, false, false, .8, false);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.turn_to_point(-37, 37, true, .7);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-37, 37, false, true, .8, false);
}

void Eclipse::Autonomous_Paths::anti_solo_awp(){
    
}

void Eclipse::Autonomous_Paths::left_half_awp(){
    
}

void Eclipse::Autonomous_Paths::right_half_awp(){
   
}

void Eclipse::Autonomous_Paths::left_7(){   

}

void Eclipse::Autonomous_Paths::right_9(){
    odom.set_robot_position(-2, 14);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(23, 25, false, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 75);
    drive.move_to_point(48, 45, false, false, .6, false);

    match_loader.set_value(true);
    pros::delay(300);
    // // drive.set_constants(3, 0, 30, 5, 0, 28, 127, 127);
    // // drive.move_to_point(50, 47, true, false, .3, false);
    // match_loader.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(40, 17, false, true, .7);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 60);
    drive.turn_to_point(48, 0, true, .4);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 0, false, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, -9, true, true, .7);

    pros::delay(700);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 20, false, true, .7);

    util.score_until_opp_block(2);
    trapdoor.set_value(false);

    t_pid.set_t_constants(10, 0, 11, 600);
    t_pid.translation_pid(-10, 90, .3);

    // wing for right side (not tuned)
    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    // drive.move_to_point(60, 10, true, false, .6);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(60, 30, true, true, .9);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.turn_to_point(0, 0, false, .3, false);

    // c_pid.set_c_constants(3, 0, 30);
    // c_pid.curve_pid(-180, 90, .5, 0.2, true);
}

// Misc paths
void Eclipse::Autonomous_Paths::skills(){
    // start in park zone
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    // clear park
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(6, 75, .7);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 90, .5);
    pros::delay(300);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(12, 75, .7);
    match_loader.set_value(true);
    pros::delay(300);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-17, 90, .7);
    odom_lift.set_value(false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(3, 60, .2);

    // reset and intake 7th block
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-3, 27, false, true, .9);
    match_loader.set_value(false);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-17, 29, false, .7);
    trapdoor.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 45, 30);
    drive.move_to_point(-17, 29, false, false, .9, false);

    // dirve to mid goal

    scoring.set_scoring_mode(scoring.ScoringMode::Outtake);
    pros::delay(150);
    scoring.set_scoring_mode(scoring.ScoringMode::None);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(0, 46, true, .7);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);
    mid_goal.set_value(true);
    
    drive.set_constants(3, 0, 30, 5, 0, 28, 60, 60);
    drive.move_to_point(-12, 33.67, true, true, .7, false);

    t_pid.set_t_constants(3, 0, 30, 1);
    t_pid.translation_pid(-7, 70, .4);

    std::cout << "mid goal " << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // score mid goal
    std::cout << "i: " << pros::millis() << std::endl;
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 430);
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(2, 70, .3, false);
    scoring.set_scoring_mode(scoring.ScoringMode::Middle, 600, 202.5);
    pros::delay(1000);
    while(true){
        if((indexer.get_current_draw() < 500) && (indexer.get_voltage() > 4400)){
            break;
        }
        pros::delay(10);
    }
    pros::delay(100);
    std::cout << "c: " << indexer.get_current_draw() << "v: " << indexer.get_voltage() << std::endl;
    std::cout << "f: " << pros::millis() << std::endl;

    // match_loader first corner
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-21, 25, false, false, .9, false);

    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-48, -4, false, false, 1, false);
    
    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-48, -20, false, .7, false);

    t_pid.set_t_constants(3, 0, 35, 100);
    t_pid.translation_pid(9.5, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(700);

    r_pid.set_r_constants(3, 0, 30);
    r_pid.rotation_pid(175, 90, .4);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-60, 17, true, true, 1.2);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-60, 74, false, true, 1.7, false);
    
    left_drive.move_voltage(0);
    right_drive.move_voltage(0);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    std::cout << "jew reset";
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // match load second corner
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(-49, 93, true, .7);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-49.5, 93, false, true, .9, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad, 600, 0);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-49, 110, false, .7, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(13.5, 60, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(500);

    r_pid.set_r_constants(10, 0, 0);
    r_pid.rotation_pid(5, 127, .5);

    r_pid.set_r_constants(10, 0, 30);
    r_pid.rotation_pid(-5, 127, .5);

    r_pid.set_r_constants(10, 0, 30);
    r_pid.rotation_pid(0, 127, .5);

    t_pid.set_t_constants(10, 0, 30, 100);
    t_pid.translation_pid(10, 90, .4);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    // score 12-15 blocks
    drive.set_constants(3, 0, 30, 5, 0, 8, 90, 127);
    drive.move_to_point(-48, 70, true, true, 1.2, false);

    trapdoor.set_value(true);
    scoring.set_scoring_mode(scoring.ScoringMode::Top, 0, 600);
    pros::delay(200);
    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(500);

    scoring.set_scoring_mode(scoring.ScoringMode::Outtake);
    pros::delay(200);
    scoring.set_scoring_mode(scoring.ScoringMode::Top);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(3, 90, .4);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-7, 90, .4);

    pros::delay(1500);

    match_loader.set_value(false);

    t_pid.set_t_constants(10, 0, 30, 100);
    t_pid.translation_pid(-10, 90, .2);

    // drive to park zone
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(-2, 92, false, .7);

    std::cout << "park zone: ";
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 75);
    drive.move_to_point(-2, 92, false, false, 1.5, false);

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
    t_pid.translation_pid(12, 90, .6);
    trapdoor.set_value(false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 90, .5);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(7, 75, .7);
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-5, 90, .5);
    pros::delay(300);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(12, 75, .7);
    match_loader.set_value(true);
    pros::delay(300);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-17, 90, .7);
    odom_lift.set_value(false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(3, 60, .2);

    scoring.set_scoring_mode(scoring.ScoringMode::Intake);
    odom_lift.set_value(false);
    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    // intake 7th block
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-5, 67, false, true, 1.4, false);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 75);
    drive.turn_to_point(-17, 66.5, false, .9);

    drive.set_constants(3, 0, 30, 5, 0, 28, 45, 30);
    drive.move_to_point(-17, 66.5, false, false, .8, false);

    pros::delay(100);
    scoring.set_scoring_mode(scoring.ScoringMode::None);
    
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(0, 46, false, .7, false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(-9, 58, false, false, .9);

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
    pros::delay(1000);
    while(true){
        if((intake.get_current_draw() < 320) && (intake.get_voltage() < -4350)){
            break;
        }
        pros::delay(100);
    }
    pros::delay(500);
    std::cout << "c: " << intake.get_current_draw() << "v: " << intake.get_voltage() << std::endl;
    std::cout << "f: " << pros::millis() << std::endl;
    




    // match load third corner
    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(-10, 75, .7);
    
    scoring.set_scoring_mode(scoring.ScoringMode::Intake);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(9, 69, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 28, 127, 90);
    drive.move_to_point(9, 69, false, false, .9);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 92, true, false, 1.7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(48, 110, false, .7);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(17, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    pros::delay(700);

    r_pid.set_r_constants(3, 0, 30);
    r_pid.rotation_pid(-5, 127, .5);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(62, 74, true, .6);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    drive.move_to_point(62, 74, false, true, .9, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    scoring.set_scoring_mode(scoring.ScoringMode::Intake, 600, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(62, 17, false, true, 1.5, false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Right);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;


    std::cout << "last corner ";
    // match load fourth corner
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.turn_to_point(49, 3, true, .6);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(49, 3, false, true, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad, 600, 0);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(49, -20, false, .7, false);

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
    drive.move_to_point(48, 23, false, true, 1.2);

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
    drive.move_to_point(35, -7, false, false, .7);

    drive.set_constants(3, 0, 30, 5, 0, 35, 127, 127);
    drive.move_to_point(10, -15, true, false, 1);

    odom_lift.set_value(true);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(13, 127, .9);
}

// 4+3 right
void Eclipse::Autonomous_Paths::test(){
    odom_lift.set_value(false);
    odom.set_robot_position(16, 0);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 5, false, true, .7, false);

    scoring.set_scoring_mode(scoring.ScoringMode::MatchLoad);
    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(46, -20, false, .7, false);

    t_pid.set_t_constants(3, 0, 30, 100);
    t_pid.translation_pid(12, 75, .7);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
    pros::delay(400);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(48, 23, false, true, .8, false);
    
    trapdoor.set_value(true);
    match_loader.set_value(false);
    // pros::delay(500);
    util.score_until_opp_block(1.75);
    // trapdoor.set_value(false);

    odom.distance_sensor_reset(5, true, Odom::Front);
    odom.distance_sensor_reset(5, false, Odom::Left);
    std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    drive.turn_to_point(20, 26, false, .9);

    trapdoor.set_value(true);
    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(20, 26, false, false, .7);
    indexer.move_voltage(0);
    intake.move_voltage(10000);
    trapdoor.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(10, 39, true, false, 1.2, false);
    

    intake_lift.set_value(true);
    intake.move_voltage(-10000);
    pros::delay(1500);
    intake.move_voltage(0);
    intake_lift.set_value(false);

    drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    drive.move_to_point(64, 6, false, true, 1.9, false);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(62, 38, true, .9);

    intake.move_voltage(12000);
    indexer.move_voltage(12000);
    drive.set_constants(3, 0, 30, 5, 0, 28, 75, 90);
    drive.move_to_point(62, 36.7, false, true, 1.2, false);

    drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    drive.turn_to_point(0, 0, false, .9, true);
    // match_loader.set_value(true);

    // odom.distance_sensor_reset(5, false, Odom::Right);
    // std::cout << "reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;


    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(50, 37, true, false, .3);

    // match_loader.set_value(false);

    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    // drive.turn_to_point(0, 93, false, .9);

    // wing.set_value(true);

    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 90);
    // drive.turn_to_point(0, 24, false, .9);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-11, 24, false, false, 1.1);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-40, 40, false, false, 1.1, false);
    // indexer.move_voltage(0);

    // pros::delay(250);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-17, 27, true, true, .8);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.turn_to_point(0, 46, true, .7);

    // intake.move_voltage(12000);
    // mid_goal.set_value(true);
    
    // drive.set_constants(3, 0, 30, 5, 0, 28, 60, 60);
    // drive.move_to_point(-8, 34, true, true, .9, false);

    // indexer.move_voltage(12000);

    // match_loader.set_value(false);
    // indexer.move_voltage(0);

    // drive.set_constants(3, 0, 30, 5, 0, 35, 90, 127);
    // drive.turn_to_point(-17, 21, false, .9);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 90);
    // drive.move_to_point(-17, 21, false, false, 1.2);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 90, 127);
    // drive.turn_to_point(0, 46, true, .9);
    // mid_goal.set_value(true);

    // drive.set_constants(3, 0, 30, 5, 0, 28, 60, 60);
    // drive.move_to_point(-10, 35, true, true, .9, false);

    // indexer.move_voltage(9000);
}
