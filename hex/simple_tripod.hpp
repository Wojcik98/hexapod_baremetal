#ifndef __SIMPLE_TRIPOD_HPP
#define __SIMPLE_TRIPOD_HPP

extern "C" {
#include <stdint.h>
}

#include "gait.hpp"
#include "hexapod.hpp"
#include <Eigen/Dense>

struct SimpleTripodConfig {
    struct Tripod_ {
        Eigen::Vector3f front_trans;
        Eigen::Vector3f mid_trans;
        Eigen::Vector3f rear_trans;
    };

    uint16_t servo_freq;
    float gait_period;
    float base_height;
    float leg_height;
    Tripod_ left_tripod;  // fl, mr, rl
    Tripod_ right_tripod; // fr, ml, rr
};

/**
 * SimpleTripod is a tripod gait that moves only straight forward.
 */
class SimpleTripod : public Gait {
public:
    SimpleTripod(Hexapod &robot, SimpleTripodConfig &config);

    void set_velocity(float velocity) volatile;
    void get_next_joints(Legsf &angles) volatile override;

private:
    constexpr static float VEL_EPS = 0.01f;

    enum State {
        STOP,
        STARTING,
        MOVE,
        STOP_REQUESTED,
        STOPPING,
    };

    void stopped(Eigen::Vector3f &base, Eigen::Vector3f &left,
                 Eigen::Vector3f &right) volatile;
    void starting(Eigen::Vector3f &base, Eigen::Vector3f &left,
                  Eigen::Vector3f &right) volatile;
    void move(Eigen::Vector3f &base, Eigen::Vector3f &left,
              Eigen::Vector3f &right) volatile;
    void move_half(Eigen::Vector3f &base, Eigen::Vector3f &support,
                   Eigen::Vector3f &swing, float phase) volatile;
    void stopping(Eigen::Vector3f &base, Eigen::Vector3f &left,
                  Eigen::Vector3f &right) volatile;

    void calculate_joints(const Eigen::Vector3f &base,
                          const Eigen::Vector3f &left,
                          const Eigen::Vector3f &right, Legsf &out) volatile;

    SimpleTripodConfig &config;
    const uint16_t servo_freq;
    const uint16_t iters_per_period;
    const float gait_period;
    const float base_height;
    const float leg_height;

    uint16_t iter = 0;
    float phase = 0.0f;

    float velocity = 0.0f;
    float stride = 0.0f;

    State state = STOP;
};

#endif /* __SIMPLE_TRIPOD_HPP */
