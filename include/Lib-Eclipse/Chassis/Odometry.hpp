extern double robot_x;
extern double robot_y;
extern double robot_theta;

namespace Eclipse{
    class Odom{
        private:
            // odometry variables
            double x = 0;
            double y = 0;

            // reset variables
            double right_weightings = 0;
            double back_weightings = 0;

            double right_weighted_distance = 0;
            double back_weighted_distance = 0;

            const double mm_to_inches = 0.0393701;

            const double right_offset = 95.0;
            const double back_offset = 98.00;

            // retune values for skills
            const double max_x = 40.1124;
            const double max_y = 5.7999;
            const double min_x = -98.527;
            const double min_y = -133.19;

            std::pair<int, int> get_wall(double heading);
        public:

            float horizontal_wheel_diameter;
            float horizontal_wheel_offset;
            float vertical_wheel_diameter;
            float vertical_wheel_offset;

            void set_horizontal_tracker_specs(double diameter, double offset);
            void set_vertical_tracker_specs(double diameter, double offset);

            float get_horizontal_displacement();
            float get_vertical_displacement();
            float get_left_displacement();
            float get_right_displacement();

            void update_position();
            void update_position_single_vertical();

            void distance_sensor_reset(int readings, bool create_task = true);
    };
}