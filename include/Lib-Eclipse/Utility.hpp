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
            double tpi = 0; // ticks per inch
        public:
            // odometry
            double get_robot_x();
            double get_robot_y();
            void set_robot_position(double x, double y);
            void set_drive_constants(const double dt_wheel_diameter, const double dt_gear_ratio, const double dt_motor_cartridge);
            
            double get_angular_error(double x, double y, bool backwards);
            double get_lateral_error(double x, double y);
            // sensors
            double get_position();
            void reset_position();

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