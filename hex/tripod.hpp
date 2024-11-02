#ifndef __TRIPOD_HPP
#define __TRIPOD_HPP

#include <cstdint>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gait.hpp"
#include "hexapod.hpp"
#include "legs.hpp"
#include "pose.hpp"

struct TripodConfig {
    struct Tripod_ {
        Tripod_(const Eigen::Vector3f &front, const Eigen::Vector3f &mid,
                const Eigen::Vector3f &rear)
            : front_trans(Eigen::Translation3f(front)),
              mid_trans(Eigen::Translation3f(mid)),
              rear_trans(Eigen::Translation3f(rear)) {}

        T3D front_trans;
        T3D mid_trans;
        T3D rear_trans;
    };

    uint16_t servo_freq;
    float gait_period; // time to move one step, half of the actual gait period
    float base_height;
    float leg_height;
    Tripod_ left_tripod;  // fl, mr, rl
    Tripod_ right_tripod; // fr, ml, rr
};

/**
 * Tripod is a tripod gait that can move in any 2D direction.
 */
class Tripod : public Gait {
public:
    Tripod(Hexapod &robot, TripodConfig &config);

    void set_twist(const AtomicTwist2D &twist);
    void get_next_joints(Legsf &angles) override;

private:
    constexpr static float VEL_EPS = 0.01f;

    enum State {
        STOP,
        MOVE,
        STOP_REQUESTED,
    };

    void move();
    void calculate_next_step();

    void calculate_joints(Legsf &out);

    TripodConfig &config;
    const uint16_t servo_freq;
    const uint16_t iters_per_period;
    const float gait_period;
    const float base_height;
    const float leg_height;

    std::atomic_uint16_t iter;
    std::atomic_bool left_swing;

    Pose base;
    Pose left;
    Pose right;

    Twist2D twist;
    AtomicTwist2D next_twist;

    Pose base_target;
    Pose swing_target;
    Pose base_start;
    Pose swing_start;

    State state = STOP;
};

#endif /* __TRIPOD_HPP */
