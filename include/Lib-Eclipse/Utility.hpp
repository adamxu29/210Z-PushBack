namespace Eclipse
{
    class Utility
    {
        private:
            double wheel_diameter = 0;
            double gear_ratio = 0;
            double motor_cartridge = 0;
            double circumference = 0;
            double tpr = 0; // ticks per revolution
            // 1800 ticks/rev with 36:1 gears
            // 900 ticks/rev with 18:1 gears
            // 300 ticks/rev with 6:1 gears
            double tpi = 0; // ticks per inch
        public:
            // odometry
            void set_drive_constants(const double dt_wheel_diameter, const double dt_gear_ratio, const double dt_motor_cartridge);
            
            // PID helpers
            double get_angular_error(double x, double y, bool backwards);
            double get_lateral_error(double x, double y);
            bool is_line_settled(double target_x, double current_x, double target_y, double current_y, double target_heading);

            // sensors
            double get_position();
            void reset_position();

            // color sort
            int red_max = 360;
            int red_min = 330;
            int blue_max = 230;
            int blue_min = 200;
            int sort_delay = 0;

            bool sorting = true;
            bool detect_ball();
            bool detect_upper_ball();
            bool is_red();
            bool is_blue();
            void sort();
            void sort_red();
            void sort_blue();
            void run_sort();
            void score_until_opp_block(int time_out = 0);

            void set_tpi();
            double get_tpi(){ return tpi; }

            double get_drive_temp();
            double get_wall_stake_position();
            double get_heading();

            // misc
            bool is_reversed(int port);
            int sign(double num);
            double get_min_angle(float angle);
            double get_min_error(float angle1, float angle2);
    };
}