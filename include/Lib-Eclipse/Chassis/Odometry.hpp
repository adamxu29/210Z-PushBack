extern double robot_x;
extern double robot_y;
extern double robot_theta;

extern double ex;
extern double ey;

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

            // Distance from robot CENTER to each sensor, in inches (positive = toward the wall that sensor faces)
            const double front_offset = 7.903273329;  // TODO: measure center-to-front-sensor in inches
            const double left_offset = 4.686594488;   // TODO: measure center-to-left-sensor in inches
            const double right_offset = 4.686594488;  // TODO: measure center-to-right-sensor in inches
            const double back_offset = 6.194094488;   // TODO: measure center-to-back-sensor in inches

            // Half the field dimension: distance from field center to any wall (inches)
            const double field_half_size = 140.42 * 0.5;

            int get_wall(double heading);
        public:
            enum SensorIndex {
                Front = 0,
                Left = 1,
                Right = 2,
                Back = 3
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
