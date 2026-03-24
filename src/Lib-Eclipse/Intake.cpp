#include "main.h"

using namespace Eclipse;

// ============================================================================
// LookupTable::get_value
// Linear interpolation between the two surrounding entries.
// Clamps to the first/last voltage outside the table range.
// ============================================================================
double LookupTable::get_value(double velocity) const {
    if (table.empty())            return 0.0;
    if (table.size() == 1)        return table[0].second;
    if (velocity <= table.front().first) return table.front().second;
    if (velocity >= table.back().first)  return table.back().second;

    for (size_t i = 0; i < table.size() - 1; i++) {
        double v1 = table[i].first,     volt1 = table[i].second;
        double v2 = table[i+1].first,   volt2 = table[i+1].second;
        if (velocity >= v1 && velocity <= v2) {
            double ratio = (velocity - v1) / (v2 - v1);
            return volt1 + ratio * (volt2 - volt1);
        }
    }
    return 0.0;
}

// ============================================================================
// VelocityController::reset
// Wraps PID::reset_variables() so intake_task can reset on mode/jam changes.
// ============================================================================
void VelocityController::reset() {
    pid.reset_variables();
}

// ============================================================================
// VelocityController::compute
// Feedforward from LUT + PID correction using Eclipse::PID::compute().
//
// Eclipse::PID::compute(current, target) returns a value in the 0-127 range,
// so we convert to mV with * (12000.0 / 127.0) before adding to feedforward.
// ============================================================================
int VelocityController::compute(double target_rpm, double actual_rpm) {
    if (target_rpm == 0.0) {
        pid.reset_variables();
        return 0;
    }

    double ff = lut.get_value(target_rpm);
    double pid_out = pid.compute(actual_rpm, target_rpm) * (12000.0 / 127.0);

    return static_cast<int>(ff + pid_out);
}

// ============================================================================
// Intake::set_motor_voltage
// Same structure as original. Modes that benefit from velocity control
// (Intake, MatchLoad, Top) pass actual_rpm into the controller.
// Outtake/Bottom stay open-loop — PID adds no value for reversal.
// ============================================================================
void Intake::set_motor_voltage(ScoringMode mode) {
    switch (mode) {
        case ScoringMode::Intake: {
            indexer.set_current_limit(2100);
            intake.move_voltage(intake_ctrl.compute(target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(target_indexer_rpm, indexer.get_actual_velocity()));
            trapdoor.set_value(false);
            match_loader.set_value(false);
            intake_lift.set_value(false);
            break;
        }

        case ScoringMode::MatchLoad: {
            indexer.set_current_limit(2100);
            intake.move_voltage(intake_ctrl.compute(target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(target_indexer_rpm, indexer.get_actual_velocity()));
            trapdoor.set_value(false);
            match_loader.set_value(true);
            color.set_led_pwm(100);
            break;
        }

        case ScoringMode::Outtake:
            intake.move_voltage(intake_ctrl.compute(-target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(-target_indexer_rpm, indexer.get_actual_velocity()));
            break;

        case ScoringMode::Top: {
            intake.move_voltage(intake_ctrl.compute(target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(target_indexer_rpm, indexer.get_actual_velocity()));
            trapdoor.set_value(true);
            mid_goal.set_value(false);
            break;
        }

        case ScoringMode::Middle:
            intake.move_voltage(intake_ctrl.compute(target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(target_indexer_rpm, indexer.get_actual_velocity()));
            mid_goal.set_value(true);
            match_loader.set_value(false);
            break;

        case ScoringMode::Bottom:
            intake.move_voltage(intake_ctrl.compute(-target_intake_rpm, intake.get_actual_velocity()));
            indexer.move_voltage(indexer_ctrl.compute(-target_indexer_rpm, indexer.get_actual_velocity()));
            intake_lift.set_value(true);
            break;

        case ScoringMode::None:
        default:
            intake_ctrl.reset();
            indexer_ctrl.reset();
            indexer.set_current_limit(2500);
            intake.move_voltage(0);
            indexer.move_voltage(0);
            color.set_led_pwm(0);
            mid_goal.set_value(false);
            intake_lift.set_value(false);
            match_loader.set_value(false);
            break;
    }
}

// ============================================================================
// Intake::set_scoring_mode / set_color — unchanged
// ============================================================================
void Intake::set_scoring_mode(ScoringMode new_mode, double intake_rpm, double indexer_rpm) {
    target_intake_rpm  = intake_rpm;
    target_indexer_rpm = indexer_rpm;
    current_mode = new_mode;
}

void Intake::set_color(Color new_color) {
    selected_color = new_color;
}

// ============================================================================
// Intake::intake_task
// Structure is identical to the original. The only changes:
//   1. intake_ctrl/indexer_ctrl.reset() is called on mode change and jam entry/exit.
//   2. The MatchLoad color-sort block is unchanged.
// ============================================================================
void Intake::intake_task() {
    int    bottom_current = intake.get_current_draw();
    int    top_current    = indexer.get_current_draw();

    bool jamming = antijam && (current_mode != ScoringMode::None) &&
                    ((bottom_current > jam_current) || (top_current > jam_current));

    if (!antijam) {
        counter++;
        if (counter > 50) {
            antijam = true;
        }
    }

    switch (jam_state) {
        case AntiJamState::Idle:
            if (current_mode != prev_mode) {
                intake_ctrl.reset(); indexer_ctrl.reset();
                set_motor_voltage(current_mode);
                prev_mode = current_mode;
                antijam   = false;
                counter   = 0;
            }

            // --- Pneumatics timers (unchanged) ---
            if (current_mode == ScoringMode::Top) {
                if (!trapdoor_delay_active) {
                    trapdoor_delay_active  = true;
                    trapdoor_delay_counter = 0;
                    trapdoor.set_value(false);
                } else {
                    if (trapdoor_delay_counter >= 8) {
                        trapdoor.set_value(true);
                    } else {
                        trapdoor_delay_counter++;
                    }
                }
            } else if (current_mode == ScoringMode::Bottom) {
                if (!intake_lift_delay_active) {
                    intake_lift_delay_active  = true;
                    intake_lift_delay_counter = 0;
                    intake_lift.set_value(false);
                } else {
                    if (intake_lift_delay_counter >= 5) {
                        intake_lift.set_value(true);
                    } else {
                        intake_lift_delay_counter++;
                    }
                }
            } else if (current_mode == ScoringMode::MatchLoad) {
                // Color sort (unchanged)
                if ((scoring.selected_color != Color::None) && util.detect_upper_ball()) {
                    if (util.is_red()) {
                        trapdoor.set_value(scoring.selected_color == Color::Blue ? true : false);
                    } else if (util.is_blue()) {
                        trapdoor.set_value(scoring.selected_color == Color::Red ? true : false);
                    }
                }
            } else {
                trapdoor_delay_active    = false;
                intake_lift_delay_active = false;
            }

            // --- Normal velocity controller update each tick ---
            // Keeps velocity controller following the target while in Idle.
            // Only runs if mode hasn't changed this tick (prev_mode == current_mode).
            if (current_mode != ScoringMode::None && prev_mode == current_mode) {
                set_motor_voltage(current_mode);
            }

            // --- Jam detection (unchanged logic) ---
            if (jamming) {
                if (jam_start == 0) { jam_start = pros::millis(); }
                if ((pros::millis() - jam_start) >= jam_detection_time) {
                    if ((current_mode == ScoringMode::Intake) ||
                        (current_mode == ScoringMode::Top)    ||
                        (current_mode == ScoringMode::Middle)) {
                        intake_ctrl.reset(); indexer_ctrl.reset();
                        set_motor_voltage(ScoringMode::None);
                        intake.move_voltage(-6000);
                        indexer.move_voltage(-12000);
                        state_start = pros::millis();
                        jam_start   = 0;
                        jam_state   = AntiJamState::Outtaking;
                    } else if ((current_mode == ScoringMode::Outtake) ||
                                (current_mode == ScoringMode::Bottom)) {
                        intake_ctrl.reset(); indexer_ctrl.reset();
                        set_motor_voltage(ScoringMode::Intake);
                        state_start = pros::millis();
                        jam_start   = 0;
                        jam_state   = AntiJamState::Intaking;
                    }
                }
            } else {
                jam_start = 0;
            }
            break;

        case AntiJamState::Outtaking:
            if ((pros::millis() - state_start) >= unjam_outtake_delay) {
                intake_ctrl.reset(); indexer_ctrl.reset();
                set_motor_voltage(current_mode);
                jam_start = 0;
                jam_state = AntiJamState::Idle;
            }
            break;

        case AntiJamState::Intaking:
            if ((pros::millis() - state_start) >= unjam_intake_delay) {
                intake_ctrl.reset(); indexer_ctrl.reset();
                set_motor_voltage(current_mode);
                jam_start = 0;
                jam_state = AntiJamState::Idle;
            }
            break;
    }
}