#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive({3, -2, -12});
pros::Motor_Group right_drive({-6, 8, 19});

pros::Motor intake(9);
pros::Motor indexer(5);

pros::Rotation vertical_tracking_wheel(4, true);
pros::Rotation horizontal_tracking_wheel(14, true);

pros::ADIDigitalOut match_loader('c');
pros::ADIDigitalOut park('b');
pros::ADIDigitalOut trapdoor('a');
pros::ADIDigitalOut wing('d');

pros::IMU imu1(20);
pros::IMU imu2(21);

pros::Optical color(1);

pros::Distance park_sensor(11);
// pros::Distance front_sensor(5);
// pros::Distance left_sensor(6);
pros::Distance right_sensor(6);
pros::Distance back_sensor(13);

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
