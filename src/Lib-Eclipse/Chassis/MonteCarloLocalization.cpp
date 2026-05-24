#include "main.h"
#include "Lib-Eclipse/Chassis/MonteCarloLocalization.hpp"

using namespace Eclipse;

// ============================================================
// RAYCASTER — slab method
// Returns distance along ray to nearest surface, or -1 if no hit.
// For the field boundary (robot inside): returns exit face distance.
// For obstacles (robot outside): returns entry face distance.
// ============================================================

struct AABB { float centre_x, centre_y, half_width, half_height; };

static float ray_vs_aabb(Eigen::Vector2f origin, Eigen::Vector2f dir, const AABB& box) {
    Eigen::Vector2f rel = origin - Eigen::Vector2f(box.centre_x, box.centre_y);

    float t_near = -std::numeric_limits<float>::infinity();
    float t_far  =  std::numeric_limits<float>::infinity();

    // x slab
    if (fabsf(dir.x()) > 1e-6f) {
        float t1 = (-box.half_width - rel.x()) / dir.x();
        float t2 = ( box.half_width - rel.x()) / dir.x();
        if (t1 > t2) std::swap(t1, t2);
        t_near = std::max(t_near, t1);
        t_far  = std::min(t_far,  t2);
    } else if (fabsf(rel.x()) > box.half_width) {
        return -1.0f;
    }

    // y slab
    if (fabsf(dir.y()) > 1e-6f) {
        float t1 = (-box.half_height - rel.y()) / dir.y();
        float t2 = ( box.half_height - rel.y()) / dir.y();
        if (t1 > t2) std::swap(t1, t2);
        t_near = std::max(t_near, t1);
        t_far  = std::min(t_far,  t2);
    } else if (fabsf(rel.y()) > box.half_height) {
        return -1.0f;
    }

    if (t_near > t_far || t_far < 0.0f) return -1.0f;

    return (t_near >= 0.0f) ? t_near : t_far;
}

// ============================================================
// OBSTACLE MAP
// ============================================================

static const AABB OBSTACLES[] = {
    { 0.0f,      0.0f,       HALF_FIELD,          HALF_FIELD          },  // field boundary
    { 0.0f,      0.0f,       CENTER_WIDTH * 0.5f, CENTER_LENGTH * 0.5f},  // centre barrier
    { -LOADER_X, -LOADER_CY, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},  // loader BL
    { -LOADER_X,  LOADER_CY, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},  // loader TL
    {  LOADER_X,  LOADER_CY, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},  // loader TR
    {  LOADER_X, -LOADER_CY, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},  // loader BR
    { -GOAL_X,   -GOAL_Y,    GOAL_WIDTH * 0.5f,   GOAL_LENGTH * 0.5f  },  // goal BL
    { -GOAL_X,    GOAL_Y,    GOAL_WIDTH * 0.5f,   GOAL_LENGTH * 0.5f  },  // goal TL
    {  GOAL_X,    GOAL_Y,    GOAL_WIDTH * 0.5f,   GOAL_LENGTH * 0.5f  },  // goal TR
    {  GOAL_X,   -GOAL_Y,    GOAL_WIDTH * 0.5f,   GOAL_LENGTH * 0.5f  },  // goal BR
};
static constexpr int NUM_OBSTACLES = sizeof(OBSTACLES) / sizeof(OBSTACLES[0]);

// ============================================================
// DISTANCE SENSOR
// ============================================================

void MCL::DistanceSensor::update() {
    int raw_mm = sensor->get();
    int conf   = sensor->get_confidence();

    if (raw_mm >= 9999 || conf < 1) { healthy = false; return; }

    float inches = raw_mm * 0.0393701f;
    if (inches > max_range) { healthy = false; return; }

    healthy  = true;
    measured = inches;
}

float MCL::DistanceSensor::likelihood(float px, float py, float p_theta) const {
    if (!healthy) return 1.0f;  // unhealthy sensor contributes no information

    // Body frame: +X = right, +Y = forward.
    // Rotation2Df assumes +X = forward, +Y = left (math convention), so we
    // must remap: forward_component = offset_y, left_component = -offset_x.
    Eigen::Vector2f sensor_pos =
        Eigen::Rotation2Df(p_theta) * Eigen::Vector2f(offset_y, -offset_x)
        + Eigen::Vector2f(px, py);

    // offset_theta uses CW-positive (compass) convention; math angles are CCW,
    // so subtract rather than add.
    float ray_theta = p_theta - offset_theta;
    Eigen::Vector2f ray_dir(cosf(ray_theta), sinf(ray_theta));

    // Find nearest obstacle along sensor ray
    float predicted = std::numeric_limits<float>::infinity();
    for (int k = 0; k < NUM_OBSTACLES; ++k) {
        float d = ray_vs_aabb(sensor_pos, ray_dir, OBSTACLES[k]);
        if (d >= 0.0f && d < predicted) predicted = d;
    }

    if (predicted == std::numeric_limits<float>::infinity()) return 0.0f;

    float e = measured - predicted;
    return expf(-0.5f * (e / sigma) * (e / sigma));
}

// ============================================================
// MCL
// ============================================================

MCL::MCL() : rng(pros::micros()) {
    std::fill(particle_x,        particle_x        + N, 0.0f);
    std::fill(particle_y,        particle_y        + N, 0.0f);
    std::fill(particle_theta,    particle_theta    + N, 0.0f);
    std::fill(particle_weights,  particle_weights  + N, 1.0f / N);
    std::fill(temp_x,            temp_x            + N, 0.0f);
    std::fill(temp_y,            temp_y            + N, 0.0f);
    std::fill(temp_theta,        temp_theta        + N, 0.0f);
    std::fill(temp_weights,      temp_weights      + N, 0.0f);
    std::fill(presample_x,       presample_x       + N, 0.0f);
    std::fill(presample_y,       presample_y       + N, 0.0f);
    std::fill(presample_theta,   presample_theta   + N, 0.0f);
    std::fill(presample_weights, presample_weights + N, 0.0f);
}

// Scatter particles uniformly across the whole field — use when position is unknown.
void MCL::init_uniform() {
    rng = XorShift32(pros::micros());
    for (int i = 0; i < N; i++) {
        particle_x[i]       = rng.range_f32(MIN_FIELD, MAX_FIELD);
        particle_y[i]       = rng.range_f32(MIN_FIELD, MAX_FIELD);
        particle_theta[i]   = rng.range_f32(0.0f, 360.0f);
        particle_weights[i] = 1.0f / N;
    }
    distance_traveled = update_distance_threshold;  // force update on first step
}

// Concentrate particles around a known starting pose ± pos_variance inches.
void MCL::init(float x, float y, float theta, float pos_variance) {
    rng = XorShift32(pros::micros());
    for (int i = 0; i < N; i++) {
        particle_x[i]       = std::clamp(x + rng.range_f32(-pos_variance, pos_variance), MIN_FIELD, MAX_FIELD);
        particle_y[i]       = std::clamp(y + rng.range_f32(-pos_variance, pos_variance), MIN_FIELD, MAX_FIELD);
        particle_theta[i]   = theta;
        particle_weights[i] = 1.0f / N;
    }
    distance_traveled = update_distance_threshold;  // force update on first step
}

// Call once per odometry tick with the delta since the last tick.
// heading is the raw compass heading from the IMU (degrees, 0=North, CW+).
// pos_std is the odometry noise std-dev for this tick (inches).
void MCL::step(float dx, float dy, float heading, float pos_std) {
    predict(dx, dy, heading, pos_std);

    distance_traveled += std::hypot(dx, dy);
    if (distance_traveled < update_distance_threshold) return;
    distance_traveled = 0.0f;

    update();
    resample();
}

// Propagate every particle by the measured odometry delta plus Gaussian noise.
// Heading is stamped directly from the IMU — particles share the same theta.
void MCL::predict(float dx, float dy, float heading, float pos_std) {
    for (int i = 0; i < N; i++) {
        particle_x[i]     += dx + rng.gaussian(pos_std);
        particle_y[i]     += dy + rng.gaussian(pos_std);
        particle_theta[i]  = heading;

        // Respawn out-of-bounds particles uniformly rather than clamping,
        // which would pile particles on the wall and corrupt the estimate.
        if (particle_x[i] < MIN_FIELD || particle_x[i] > MAX_FIELD ||
            particle_y[i] < MIN_FIELD || particle_y[i] > MAX_FIELD) {
            particle_x[i] = rng.range_f32(MIN_FIELD, MAX_FIELD);
            particle_y[i] = rng.range_f32(MIN_FIELD, MAX_FIELD);
        }
    }
}

// Weight every particle by the joint sensor likelihood.
void MCL::update() {
    bool any_healthy = false;
    for (auto& s : sensors) { s.update(); if (s.healthy) any_healthy = true; }
    if (!any_healthy) return;

    for (int i = 0; i < N; i++) {
        // compass → mathematical radians for Rotation2Df / trig
        float math_theta = (90.0f - particle_theta[i]) * (float)M_PI / 180.0f;
        float w = 1.0f;
        for (const auto& s : sensors) {
            w *= s.likelihood(particle_x[i], particle_y[i], math_theta);
            if (w == 0.0f) break;
        }
        particle_weights[i] = w;
    }

    // Two-pass normalisation: divide by max first to keep floats away from zero
    float max_w = *std::max_element(particle_weights, particle_weights + N);
    if (max_w <= 0.0f) {
        std::fill(particle_weights, particle_weights + N, 1.0f / N);
        return;
    }
    float sum = 0.0f;
    for (int i = 0; i < N; i++) { particle_weights[i] /= max_w; sum += particle_weights[i]; }
    float inv_sum = 1.0f / sum;
    for (int i = 0; i < N; i++) particle_weights[i] *= inv_sum;
}

// Systematic (low-variance) resampling.
void MCL::resample() {
    // Snapshot current particles so we can draw from them while overwriting
    for (int i = 0; i < N; i++) {
        presample_x[i]       = particle_x[i];
        presample_y[i]       = particle_y[i];
        presample_theta[i]   = particle_theta[i];
        presample_weights[i] = particle_weights[i];
    }

    const float inv_n  = 1.0f / N;
    const float offset = rng.next_f32() * inv_n;
    float cum = presample_weights[0];
    int   j   = 0;

    for (int i = 0; i < N; i++) {
        float pin = offset + i * inv_n;
        while (pin > cum && j < N - 1) { j++; cum += presample_weights[j]; }
        temp_x[i]      = presample_x[j];
        temp_y[i]      = presample_y[j];
        temp_theta[i]  = presample_theta[j];
        temp_weights[i] = inv_n;
    }

    for (int i = 0; i < N; i++) {
        particle_x[i]       = temp_x[i];
        particle_y[i]       = temp_y[i];
        particle_theta[i]   = temp_theta[i];
        particle_weights[i] = temp_weights[i];
    }
}

// Weighted mean position; circular mean for heading to handle 0/360 wrap.
MCL::Estimate MCL::estimate() const {
    float ex = 0.0f, ey = 0.0f;
    float sin_sum = 0.0f, cos_sum = 0.0f;
    for (int i = 0; i < N; i++) {
        float w = particle_weights[i];
        ex += w * particle_x[i];
        ey += w * particle_y[i];
        float rad = particle_theta[i] * (float)M_PI / 180.0f;
        sin_sum += w * sinf(rad);
        cos_sum += w * cosf(rad);
    }
    float et = atan2f(sin_sum, cos_sum) * 180.0f / (float)M_PI;
    if (et < 0.0f) et += 360.0f;
    return {ex, ey, et};
}

// ============================================================
// DEBUG HELPERS
// ============================================================

// Print raw sensor readings, confidence, and health.
// Call before enabling the filter to verify sensors are alive.
void MCL::debug_sensors() {
    std::cout << "=== sensor health ===" << std::endl;
    for (int i = 0; i < (int)sensors.size(); i++) {
        auto& s = sensors[i];
        int raw_mm = s.sensor->get();
        int conf   = s.sensor->get_confidence();
        s.update();
        std::cout << "[" << i << "] raw=" << raw_mm << "mm"
                  << "  conf=" << conf
                  << "  healthy=" << s.healthy
                  << "  measured=" << s.measured << "\""
                  << "  sigma=" << s.sigma << "\""
                  << "  max_range=" << s.max_range << "\"" << std::endl;
    }
}

// Print measured vs. raycaster-predicted distance for each sensor at a given pose.
// The key diagnostic: if measured ≠ predicted at the true pose, fix geometry offsets.
// theta_deg is compass heading (same convention as the IMU).
void MCL::debug_likelihood(float x, float y, float theta_deg) const {
    float math_theta = (90.0f - theta_deg) * (float)M_PI / 180.0f;
    std::cout << "=== likelihood at (" << x << ", " << y << ", " << theta_deg << "deg) ===" << std::endl;
    float joint = 1.0f;
    for (int i = 0; i < (int)sensors.size(); i++) {
        const auto& s = sensors[i];

        Eigen::Vector2f sensor_pos =
            Eigen::Rotation2Df(math_theta) * Eigen::Vector2f(s.offset_y, -s.offset_x)
            + Eigen::Vector2f(x, y);
        float ray_theta = math_theta - s.offset_theta;
        Eigen::Vector2f ray_dir(cosf(ray_theta), sinf(ray_theta));
        float predicted = std::numeric_limits<float>::infinity();
        for (int k = 0; k < NUM_OBSTACLES; ++k) {
            float d = ray_vs_aabb(sensor_pos, ray_dir, OBSTACLES[k]);
            if (d >= 0.0f && d < predicted) predicted = d;
        }

        // Convert math ray angle back to compass degrees for readability
        float ray_compass = 90.0f - ray_theta * 180.0f / (float)M_PI;
        if (ray_compass < 0.0f)   ray_compass += 360.0f;
        if (ray_compass >= 360.0f) ray_compass -= 360.0f;

        float l = s.likelihood(x, y, math_theta);
        float pred_print = (predicted == std::numeric_limits<float>::infinity()) ? -1.0f : predicted;
        std::cout << "[" << i << "]"
                  << "  pos=(" << sensor_pos.x() << ", " << sensor_pos.y() << ")\""
                  << "  heading=" << ray_compass << "deg"
                  << "  measured=" << s.measured << "\""
                  << "  predicted=" << pred_print << "\""
                  << "  error=" << (s.healthy ? s.measured - pred_print : 0.0f) << "\""
                  << "  likelihood=" << l
                  << "  healthy=" << s.healthy << std::endl;
        joint *= l;
    }
    std::cout << "joint=" << joint << std::endl;
}

void MCL::debug_print(int count) const {
    count = std::min(count, N);
    std::cout << "=== MCL particles (" << count << ") ===" << std::endl;
    for (int i = 0; i < count; i++) {
        std::cout << "[" << i << "] x=" << particle_x[i]
                  << " y=" << particle_y[i]
                  << " t=" << particle_theta[i]
                  << " w=" << particle_weights[i] << std::endl;
    }
}

void MCL::simulate(float dx, float dy, float heading, float pos_std) {
    predict(dx, dy, heading, pos_std);
    debug_print();
}