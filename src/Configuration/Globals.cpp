#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive({-20, -19, 18});
pros::Motor_Group right_drive({11, -12, 13});

pros::Motor intake(3);
pros::Motor indexer(10);

pros::Rotation vertical_tracking_wheel(14);
pros::Rotation horizontal_tracking_wheel(5, true);

pros::ADIDigitalOut match_loader('a');
pros::ADIDigitalOut park('h');
pros::ADIDigitalOut trapdoor('g');

pros::IMU imu1(16);
pros::IMU imu2(2);

pros::Optical color(9);

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

GUI gui;
