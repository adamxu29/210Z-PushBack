namespace Eclipse{
    class OPControl{
        private:
            bool shooter_raised = false;
            bool match_loading = false;
            bool scoring = false;
            int cycle_counter = 0;
        public:
            bool skills = false;
            bool color_sorting = false;
            float pusher_speed = 127;

            void exponential_curve_accelerator();
            void drivetrain_control();
            void power_intake(float speed);
            void manual_pusher(float speed);
            void power_pusher(float speed);
            void pusher_control();

            void change_pusher_speed();

            void raise_shooter();
            void activate_match_load();

            void driver_control();
    };
}