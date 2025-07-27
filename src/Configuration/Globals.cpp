#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive({15, -12, -11});
pros::Motor_Group right_drive({-14, 16, 13,});

pros::Motor intake(19);
pros::Motor pusher(20);

pros::Rotation vertical_tracking_wheel(3);
pros::Rotation horizontal_tracking_wheel(10, true);

pros::ADIDigitalOut scoring_adjuster('h');
pros::ADIDigitalOut match_loader('g');

pros::IMU imu1(1);
pros::IMU imu2(2);

pros::Distance pusher_reset_sensor(21);

// pros::Distance front_sensor(5);
// pros::Distance left_sensor(6);
pros::Distance right_sensor(18);
pros::Distance back_sensor(7);

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
