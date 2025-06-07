namespace Eclipse
{
    class FeedbackControl
    {
        public:
            double target_tol; double target_final_tol; double t_kp; double r_kp; double iterator;
            double distance; double alpha; double beta; double t_error; double r_error;
            double previous_drive_error; double previous_turn_error;
            double t_local_kp = 5; double t_local_ki = 0.003; double t_local_kd = 35;
            double t_local_derivative;  double t_local_integral;  double t_local_tolerance; double t_local_error; double t_local_previouserror; double t_local_multiplier;
            double t_local_averageposition; double t_local_averageHeading; double t_local_FailSafeCounter; int t_local_threshholdcounter;

            double lkp = 0;
            double lkd = 0;
            double akp = 0;
            double akd = 0;
            double mtp_max_linear_speed = 0;
            double mtp_max_angular_speed = 0;

            void set_constants(const double t_kp, const double r_kp, const double f_tt, const double t);
            void reset_mtp_constants();
            FeedbackControl();
            void set_mtp_constants(double lkp, double lkd, double akp, double akd, double max_linear_speed, double max_angular_speed);
            void TurnToPoint(const int targetX, const int targetY);
            void move_to_point(double target_x, double target_y, bool backwards = false, bool init_delay = true, double timer = 0);
    };
}