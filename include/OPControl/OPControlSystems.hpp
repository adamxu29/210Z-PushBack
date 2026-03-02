namespace Eclipse{
    class OPControl{
        private:
            bool match_loading = false;
            bool parking = false;
            bool trapdoor_down = false;
            bool intake_lift_active = true;
            bool wing_up = false;
            bool trapdoor_delay_active = false;
            int trapdoor_delay_counter = 0;
            bool intake_lift_delay_active = false;
            int intake_lift_delay_counter = 0;
            float intake_speed = 127;
            float shooter_speed = 127;
        public:
            bool skills = false;
            bool color_sorting = false;
            bool driver_disabled = false;

            void exponential_curve_accelerator();
            void drivetrain_control();
            void power_intake(float speed);
            void intake_lift_control();
            void power_indexer(float speed);

            int max_speed = 127;
            int slow_speed = 50;
            void change_intake_speed();

            void activate_match_load();
            void activate_double_park();
            void activate_trapdoor();
            void activate_wing();

            void driver_control(bool disabled);
    };
}
