#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive({-20, -18, 19});
pros::Motor_Group right_drive({11, 13, -12});

pros::Motor intake(3);
pros::Motor pusher(10);

pros::Rotation vertical_tracking_wheel(14);
pros::Rotation horizontal_tracking_wheel(5, true);

pros::ADIDigitalOut scoring_adjuster('h');
pros::ADIDigitalOut match_loader('g');
pros::ADIDigitalOut descore('f');

pros::IMU imu1(16);
pros::IMU imu2(2);

pros::Optical color_sensor(9);

// pros::Distance front_sensor(5);
// pros::Distance left_sensor(6);
pros::Distance right_sensor(1);
pros::Distance back_sensor(17);

Utility util;
OPControl driver;
PID_Tuner tuner;

Translation_PID t_pid;
Rotation_PID r_pid;
Curve_PID c_pid;
PID m_pid;
Odom odom;
Drive drive;
// MonteCarloLocalization mcl;

Autonomous_Paths auton;
Autonomous_Paths::Red red;
Autonomous_Paths::Blue blue;

GUI gui;
