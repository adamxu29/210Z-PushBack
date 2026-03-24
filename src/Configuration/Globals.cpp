#include "main.h"

using namespace Eclipse;
// use AssetConfig, this should be for backup

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor front_left(-1, pros::E_MOTOR_GEARSET_06);
pros::Motor mid_left(2, pros::E_MOTOR_GEARSET_06);
pros::Motor back_left(-3, pros::E_MOTOR_GEARSET_06);
pros::Motor front_right(8, pros::E_MOTOR_GEARSET_06);
pros::Motor mid_right(-9, pros::E_MOTOR_GEARSET_06);
pros::Motor back_right(10, pros::E_MOTOR_GEARSET_06);

pros::Motor_Group left_drive({front_left, mid_left, back_left});
pros::Motor_Group right_drive({front_right, mid_right, back_right});

pros::Motor intake(7);
pros::Motor indexer(4);

pros::Rotation vertical_tracking_wheel(12, true);
pros::Rotation horizontal_tracking_wheel(5, false);

pros::ADIDigitalOut match_loader('h');
pros::ADIDigitalOut mid_goal('d');
pros::ADIDigitalOut trapdoor('g');
pros::ADIDigitalOut wing('b');
pros::ADIDigitalOut intake_lift('a');
pros::ADIDigitalOut odom_lift('c');

pros::IMU imu1(17);
// pros::IMU imu2(19);

pros::Optical color(18);

pros::Distance front_sensor(6);
pros::Distance left_sensor(11);
pros::Distance right_sensor(21);
pros::Distance back_sensor(20);

Utility util;
OPControl driver;
Intake scoring;
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
pros::Task* update_telemetry = nullptr;
// pros::Task* run_intake = nullptr;