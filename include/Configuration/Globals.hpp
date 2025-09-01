#include "main.h"

// controller
extern pros::Controller controller;

extern pros::Motor_Group left_drive;
extern pros::Motor_Group right_drive;

extern pros::Motor intake;
extern pros::Motor pusher;

extern pros::Rotation vertical_tracking_wheel;
extern pros::Rotation horizontal_tracking_wheel;

extern pros::ADIDigitalOut scoring_adjuster;
extern pros::ADIDigitalOut match_loader;
extern pros::ADIDigitalOut descore;

extern pros::IMU imu1;
extern pros::IMU imu2;

extern pros::Optical color_sensor;
extern pros::Distance front_sensor;
extern pros::Distance left_sensor;
extern pros::Distance right_sensor;
extern pros::Distance back_sensor;

extern Eclipse::Utility util;
extern Eclipse::OPControl driver;
extern Eclipse::PID_Tuner tuner;

extern Eclipse::Translation_PID t_pid;
extern Eclipse::Rotation_PID r_pid;
extern Eclipse::Curve_PID c_pid;
extern Eclipse::PID m_pid;
extern Eclipse::Odom odom;
extern Eclipse::Drive drive;

extern Eclipse::Autonomous_Paths auton;
extern Eclipse::Autonomous_Paths::Red red;
extern Eclipse::Autonomous_Paths::Blue blue;

extern Eclipse::GUI gui;
