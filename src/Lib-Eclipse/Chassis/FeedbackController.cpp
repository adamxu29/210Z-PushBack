#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

using namespace Eclipse;

void Eclipse::FeedbackControl::set_constants(const double t_kp, const double r_kp, const double f_tt, const double t){ // Set constants
    mtp.t_kp = t_kp;
    mtp.r_kp = r_kp;
    mtp.target_final_tol = f_tt;
    mtp.target_tol = t;
  }
  
void Eclipse::FeedbackControl::reset_mtp_constants(){ // Reset values
    mtp.distance = 0;
    mtp.alpha = 0;
    mtp.t_error = 0;
    mtp.beta = 0;
    mtp.iterator = 0;
  }
  
  Eclipse::FeedbackControl::FeedbackControl(){
    mtp.t_local_kp = 13;
    mtp.t_local_derivative          = 0;
    mtp.t_local_integral            = 0;
    mtp.t_local_tolerance           = 3;
    mtp.t_local_error               = 0;
    mtp.t_local_previouserror       = 0;
    mtp.t_local_multiplier          = 3000;
    mtp.t_local_averageposition     = 0;
    mtp.t_local_averageHeading      = 0;
    mtp.t_local_FailSafeCounter     = 0;
    mtp.t_local_threshholdcounter   = 0;
  }
  
void Eclipse::FeedbackControl::set_mtp_constants(double lkp, double lkd, double akp, double akd, double max_linear_speed, double max_angular_speed) {
    mtp.lkp = lkp;
    mtp.lkd = lkd;
    mtp.akp = akp;
    mtp.akd = akd;
    mtp.mtp_max_linear_speed = max_linear_speed;
    mtp.mtp_max_angular_speed = max_angular_speed;
   }

   void Eclipse::FeedbackControl::TurnToPoint(const int targetX, const int targetY){
    int ct = 0;
    double prev_linear_error = 0;
    double prev_angular_error = 0;
    double threshold = 3;
    while (true){
      odom.update_position();
  
      double angular_error = util.getAngleError(targetX, targetY, false);
      double angular_derivative = angular_error - prev_angular_error;
      double ang_speed = (angular_error * mtp.akp) + (angular_derivative * mtp.akd);
      if ((ang_speed * (12000.0 / 127)) > mtp.mtp_max_angular_speed * (12000.0 / 127)) { ang_speed = mtp.mtp_max_angular_speed; }
      else if (ang_speed * (12000.0 / 127) < -mtp.mtp_max_angular_speed * 12000.0 / 127){ ang_speed = -mtp.mtp_max_angular_speed; }
        
      left_drive.move_voltage((-ang_speed) * (12000.0 / 127));
      right_drive.move_voltage((ang_speed) * (12000.0 / 127));
      ct++;
    
      if (fabs(angular_error * (180 / M_PI)) < threshold){
        left_drive.move_voltage(0);
        right_drive.move_voltage(0);
        ct = 0;
        break;
      }
  
      pros::delay(10);
    }
  
}


void Eclipse::FeedbackControl::move_to_point(double target_x, double target_y, bool backwards, bool init_delay, double timer){
    int ct = 0;
    double prev_linear_error = 0;
    double prev_angular_error = 0;
    double threshold = 3;
  
    bool settling = false;
  
    int overRideTimer = 0;
    int distanceThreshold = 1;
    double prev_x = 0;
    double prev_y = 0;
  
    double prev_angular_speed = 0;
    double global_angular_speed = 0;
    double global_linear_velocity = 0;
    double global_linear_acceleration = 0;
    double prev_global_linear_velocity = 0;
  
    while (true){
      odom.update_position();
  
      double angular_error = util.getAngleError(target_x, target_y, false) * (180 / M_PI);
      double linear_error = util.getDistanceError(target_x, target_y);
  
      if (backwards == true){
        angular_error = util.getAngleError(target_x, target_y, true) * (180 / M_PI);
        linear_error = -linear_error;
      }
  
      if (fabs(linear_error) < 7.5 && settling == false) {
        mtp.mtp_max_angular_speed = fmin(fabs(prev_angular_speed), 10);
        settling = true;
      }
  
      double linear_derivative = linear_error - prev_linear_error;
      double angular_derivative = angular_error - prev_angular_error;
  
      double lin_speed = (linear_error * mtp.lkp) + (linear_derivative * mtp.lkd);
      double ang_speed = (angular_error * mtp.akp) + (angular_derivative * mtp.akd);
  
      double ang_lin_adjustment_factor = angular_error * (M_PI / 180);
      lin_speed *= std::cos(ang_lin_adjustment_factor);
  
      if ((lin_speed * (12000.0 / 127)) > mtp.mtp_max_linear_speed * (12000.0 / 127)) {
        lin_speed = mtp.mtp_max_linear_speed;
      }
      else if (lin_speed * (12000.0 / 127) < -mtp.mtp_max_linear_speed * 12000.0 / 127){
        lin_speed = -mtp.mtp_max_linear_speed;
      }
      if ((ang_speed * (12000.0 / 127)) > mtp.mtp_max_angular_speed * (12000.0 / 127)) {
        ang_speed = mtp.mtp_max_angular_speed; 
      }
      else if (ang_speed * (12000.0 / 127) < -mtp.mtp_max_angular_speed * 12000.0 / 127){
        ang_speed = -mtp.mtp_max_angular_speed; 
      }
  
      if (ct < 20 && init_delay) {
        lin_speed = 0;  
      }
  
      if (fabs(linear_error) < 8) {
        ang_speed = 0;
      }
  
      float leftPower = lin_speed - ang_speed;
      float rightPower = lin_speed + ang_speed;
      
      left_drive.move_voltage(leftPower * (12000.0 / 127));
      right_drive.move_voltage(rightPower * (12000.0 / 127));
      ct++;
    
      if (fabs(linear_error) < threshold){
        ang_speed = 0;
        left_drive.move_voltage(0);
        right_drive.move_voltage(0);
        ct = 0;
        break;
      }
  
      if (ct > (timer * 100)) { 
        left_drive.move_voltage(0);
        right_drive.move_voltage(0);
        ct = 0;
         break;
      }  
  
      prev_linear_error = linear_error;
      prev_angular_error = angular_error;
      prev_angular_speed = ang_speed;
      prev_global_linear_velocity = global_linear_velocity;
  
      pros::delay(10);
    }
}