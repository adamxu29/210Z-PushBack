extern double robot_x;
extern double robot_y;
extern double robot_theta;

namespace Eclipse{
    class Odom{
        private:
            // odometry variables
            double x = 0;
            double y = 0;

            double prev_horizontal_displacement = 0;
            double prev_vertical_displacement = 0;
            double prev_heading = 0;

            // reset variables
            const double mm_to_inches = 0.0393701;

            const double front_offset = 54;
            const double left_offset = 46;
            const double right_offset = 52;

            // retune values for skills
            const double max_x = 65.4332062;
            const double max_y = 108.6344109;
            const double min_x = -64.566964;
            const double min_y = -14.960638;

            int get_wall(double heading);
        public:
            enum SensorIndex {
                Front = 0,
                Left = 1,
                Right = 2
            };

            float horizontal_wheel_diameter;
            float horizontal_wheel_offset;
            float vertical_wheel_diameter;
            float vertical_wheel_offset;

            double get_robot_x();
            double get_robot_y();
            void set_robot_position(double x, double y);
            void set_robot_heading(double heading);
            static void update_telemetry_fn(void* param);

            void set_horizontal_tracker_specs(double diameter, double offset);
            void set_vertical_tracker_specs(double diameter, double offset);

            float get_horizontal_displacement();
            float get_vertical_displacement();
            float get_left_displacement();
            float get_right_displacement();

            void update_position();
            void update_position_single_vertical();

            void distance_sensor_reset(int readings, bool create_task, SensorIndex sensor);
    };
}
