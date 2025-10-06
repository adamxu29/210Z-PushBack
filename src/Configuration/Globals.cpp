#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive({2, -1, -12});
pros::Motor_Group right_drive({-9, 10, 19});

pros::Motor intake(6);
pros::Motor indexer(3);

pros::Rotation vertical_tracking_wheel(14);
pros::Rotation horizontal_tracking_wheel(5, true);

pros::ADIDigitalOut match_loader('c');
pros::ADIDigitalOut park('a');
pros::ADIDigitalOut trapdoor('b');

pros::IMU imu1(20);
pros::IMU imu2(21);

pros::Optical color(4);

// pros::Distance front_sensor(5);
// pros::Distance left_sensor(6);
pros::Distance right_sensor(7);
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
