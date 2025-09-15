namespace Eclipse{
    class OPControl{
        private:
            bool match_loading = false;
            bool parking = false;
            float intake_speed = 127;
            float shooter_speed = 127;
        public:
            bool skills = false;
            bool color_sorting = false;

            void exponential_curve_accelerator();
            void drivetrain_control();
            void power_intake(float speed);
            void manual_pusher(float speed);
            void power_indexer(float speed);
            void pusher_control();

            void change_intake_speed();
            void change_shooter_speed();

            void activate_match_load();
            void activate_double_park();

            void driver_control();
    };
}