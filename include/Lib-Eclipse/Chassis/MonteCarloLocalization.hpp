#pragma once
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#include "Eigen/Dense"
#include <vector>
#include <cmath>
#include <algorithm>

namespace Eclipse {

// ============================================================
// FIELD & OBSTACLE GEOMETRY (inches, origin at field centre)
// ============================================================
    constexpr float FIELD      = 142.42f;
    constexpr float HALF_FIELD = FIELD * 0.5f;
    constexpr float MAX_FIELD  =  HALF_FIELD;
    constexpr float MIN_FIELD  = -HALF_FIELD;

    constexpr float LOADER_X       = 47.0f;
    constexpr float LOADER_RADIUS  = 3.0f;
    constexpr float LOADER_PADDING = 0.5f;
    constexpr float LOADER_WIDTH   = LOADER_RADIUS * 2.0f + LOADER_PADDING * 2.0f;
    constexpr float LOADER_LENGTH  = LOADER_RADIUS * 2.0f + LOADER_PADDING;
    constexpr float LOADER_CY      = HALF_FIELD - LOADER_LENGTH * 0.5f;

    constexpr float GOAL_X         = 48.0f;
    constexpr float GOAL_Y         = 23.0f;
    constexpr float GOAL_PADDING   = 1.0f;
    constexpr float GOAL_WIDTH     = 6.0f + GOAL_PADDING * 2.0f;
    constexpr float GOAL_LENGTH    = 3.0f + GOAL_PADDING * 2.0f;

    constexpr float CENTER_PADDING = 1.0f;
    constexpr float CENTER_WIDTH   = 21.0f + CENTER_PADDING * 2.0f;
    constexpr float CENTER_LENGTH  = 21.0f + CENTER_PADDING * 2.0f;

// ============================================================
// MCL
// ============================================================
class MCL {
public:
    struct Estimate { float x, y, theta; };

    struct DistanceSensor {
        pros::Distance* sensor;
        float offset_x, offset_y;   // body-frame mount position (inches)
        float offset_theta;          // body-frame pointing direction (radians)
        float sigma;                 // Gaussian noise std-dev (inches) — tune per sensor
        float max_range;             // ignore readings beyond this (inches)

        float measured;              // last valid reading (inches)
        bool  healthy;               // true when last reading was valid & in range

        void  update();
        float likelihood(float px, float py, float p_theta) const;
    };

    std::vector<DistanceSensor> sensors;
    bool  enabled = false;

    // Minimum travel distance between sensor-update cycles.
    // Prevents thrashing when stationary; increase if convergence is too slow.
    float update_distance_threshold = 0.5f;  // inches

    MCL();
    void     init(float x, float y, float theta, float pos_variance);
    void     init_uniform();
    void     step(float dx, float dy, float heading, float pos_std);
    Estimate estimate() const;

    // Debug helpers — call from opcontrol button press
    void debug_sensors();
    void debug_likelihood(float x, float y, float theta_deg) const;
    void debug_print(int count = N) const;
    void simulate(float dx, float dy, float heading, float pos_std = 0.1f);

private:
    // Fast, minimal-state PRNG with Box-Muller Gaussian
    struct XorShift32 {
        uint32_t state;
        XorShift32(uint32_t seed = pros::micros()) : state(seed == 0 ? 0x12345678u : seed) {}

        inline uint32_t next_u32() {
            uint32_t x = state;
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            return (state = x);
        }
        inline float next_f32()                    { return (next_u32() >> 8) * (1.0f / (1u << 24)); }
        inline float range_f32(float lo, float hi) { return lo + (hi - lo) * next_f32(); }
        inline float gaussian(float std_dev) {
            float u1 = std::max(next_f32(), 1e-12f);
            float u2 = next_f32();
            return std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * (float)M_PI * u2) * std_dev;
        }
    };

    static constexpr int N = 300;

    // Structure-of-arrays layout for cache efficiency
    float particle_x[N], particle_y[N], particle_theta[N], particle_weights[N];
    float temp_x[N],      temp_y[N],     temp_theta[N],     temp_weights[N];
    float presample_x[N], presample_y[N], presample_theta[N], presample_weights[N];

    XorShift32 rng;
    float      distance_traveled = 0.0f;  // accumulated since last sensor update

    void predict(float dx, float dy, float heading, float pos_std);
    void update();
    void resample();
};

} // namespace Eclipse
