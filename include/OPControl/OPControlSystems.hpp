namespace Eclipse{
    class OPControl{
        private:
            bool match_loading = false;
            bool parking = false;
            bool trapdoor_down = false;
            float intake_speed = 127;
            float shooter_speed = 127;
        public:
            bool skills = true;
            bool color_sorting = false;
            bool driver_disabled = false;

            void exponential_curve_accelerator();
            void drivetrain_control();
            void power_intake(float speed);
            void power_indexer(float speed);
            
            void change_shooter_speed();

            void activate_match_load();
            void activate_double_park();
            void activate_trapdoor();

            void driver_control(bool disabled);
    };
}