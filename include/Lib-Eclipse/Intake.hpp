#pragma once
#include "main.h"

namespace Eclipse {

    // ============================================================================
    // LookupTable — feedforward: velocity (RPM) → voltage (mV)
    // ============================================================================
    class LookupTable {
    public:
        LookupTable(std::initializer_list<std::pair<double, double>> data)
            : table(data) {}

        double get_value(double velocity) const;

    private:
        std::vector<std::pair<double, double>> table;
    };

    // ============================================================================
    // VelocityController
    // No internal task. compute() is called each tick from intake_task().
    // Uses Eclipse::PID for velocity error correction.
    // ============================================================================
    class VelocityController {
    public:
        VelocityController(LookupTable lut)
            : lut(lut) {}

        // Returns voltage. Caller passes actual_rpm from its own motor.
        int compute(double target_rpm, double actual_rpm);

        // Zero PID state — call on every mode change and jam entry/exit.
        void reset();

    private:
        LookupTable lut;
        PID         pid;
    };

    // ============================================================================
    // Intake — same structure as original
    // ============================================================================
    class Intake {
        public:
            enum class RunMode {
                Scoring,  // upper sensor — opens trapdoor, stops on opponent ball
                Intake    // lower sensor — runs intake, stops on opponent ball
            };

        private:
            bool trapdoor_delay_active = false;
            int  trapdoor_delay_counter = 0;

            bool intake_lift_delay_active = false;
            int  intake_lift_delay_counter = 0;

            int jam_current         = 2400;
            int jam_detection_time  = 500;
            int unjam_outtake_delay = 300;
            int unjam_intake_delay  = 300;
            int jam_start           = 0;
            int state_start         = 0;

            enum class AntiJamState {
                Idle,
                Outtaking,
                Intaking
            };

            AntiJamState jam_state = AntiJamState::Idle;

            // Two controllers with independent PID state — caller passes actual_rpm.
            static LookupTable make_lut() {
                return LookupTable({
                    {-600, -12000}, {-565, -11000}, {-520, -10000}, {-460, -9000}, {-400, -8000}, {-345, -7000},
                    {-288, -6000}, {-230, -5000}, {-175, -4000}, {-115, -3000}, {-50, -2000}, {0, 0}, {50, 2000},
                    {115, 3000}, {175, 4000}, {230, 5000}, {290, 6000}, {350, 7000}, {400, 8000}, {460, 9000},
                    {520, 10000}, {560, 11000}, {600, 12000}
                });
            }

            VelocityController intake_ctrl{make_lut()};
            VelocityController indexer_ctrl{make_lut()};

        public:
            int top_voltage    = 0;
            int bottom_voltage = 0;

            enum class ScoringMode {
                None,
                Intake,
                MatchLoad,
                Outtake,
                Top,
                Middle,
                Bottom
            };

            ScoringMode current_mode = ScoringMode::None;
            ScoringMode prev_mode = ScoringMode::None;

            bool antijam = true;
            int counter = 0;

            double target_intake_rpm  = 600.0;
            double target_indexer_rpm = 600.0;

            enum class Color {
                Red,
                Blue,
                None
            };

            Color selected_color = Color::None;

            void set_color(Color new_color);

            // Defaults to 0 so existing call sites without the argument still compile.
            void set_motor_voltage(ScoringMode mode);

            void set_scoring_mode(ScoringMode new_mode, double intake_rpm = 600.0, double indexer_rpm = 600.0);
            void score_until_opp_block(RunMode run_mode = RunMode::Scoring, int time_out = 0, double intake_rpm = 600.0, double indexer_rpm = 600.0);
            void intake_task();
    };

}