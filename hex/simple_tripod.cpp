#include "simple_tripod.hpp"

// extern "C" {
// #include <stdint.h>
// }
#include <cmath>

#include "legs.hpp"

SimpleTripod::SimpleTripod(Hexapod &robot, SimpleTripodConfig &config)
    : Gait(robot), servo_freq(config.servo_freq),
      iters_per_period(servo_freq * config.gait_period),
      gait_period((float)iters_per_period / servo_freq),
      base_height(config.base_height), leg_height(config.leg_height),
      config(config) {}

void SimpleTripod::set_velocity(float velocity) volatile {
    if (abs(velocity) < VEL_EPS) {
        velocity = 0.0f;
        if (state != STOP) {
            state = STOP_REQUESTED;
        }
    } else if (state == STOP) {
        state = STARTING;
    }
    this->velocity = velocity;
    stride = velocity * gait_period;
}

void SimpleTripod::get_next_joints(Legsf &angles) volatile {
    Eigen::Vector3f base, left, right;

    if (state == STOP) {
        stopped(base, left, right);
    } else if (state == STARTING) {
        starting(base, left, right);

        if (phase >= 0.5f) {
            state = MOVE;
            iter = 0;
            phase = 0.0f;
        }
    } else if (state == MOVE || state == STOP_REQUESTED) {
        move(base, left, right);

        // wait for the period to end
        if (state == STOP_REQUESTED && iter == 0) {
            state = STOPPING;
        }
    } else if (state == STOPPING) {
        stopping(base, left, right);

        if (phase >= 0.5f) {
            state = STOP;
            iter = 0;
            phase = 0.0f;
        }
    }

    calculate_joints(base, left, right, angles);

    iter++;
    if (iter >= iters_per_period) {
        iter = 0;
    }
    phase = (float)iter / iters_per_period;
}

void SimpleTripod::calculate_joints(const Eigen::Vector3f &base,
                                    const Eigen::Vector3f &left,
                                    const Eigen::Vector3f &right,
                                    Legsf &out) volatile {
    // calculate end effectors' positions w.r.t. base
    LegsVec3f ee_base = {
        config.left_tripod.front_trans + left - base,
        config.right_tripod.mid_trans + right - base,
        config.left_tripod.rear_trans + left - base,
        config.right_tripod.front_trans + right - base,
        config.left_tripod.mid_trans + left - base,
        config.right_tripod.rear_trans + right - base,
    };

    // calculate joint angles
    robot.inverse_kinematics(ee_base, out);
}

/**
 * Stand still.
 */
void SimpleTripod::stopped(Eigen::Vector3f &base, Eigen::Vector3f &left,
                           Eigen::Vector3f &right) volatile {
    base(0) = 0.0f;
    base(1) = 0.0f;
    base(2) = base_height;

    left(0) = 0.0f;
    left(1) = 0.0f;
    left(2) = 0.0f;

    right(0) = 0.0f;
    right(1) = 0.0f;
    right(2) = 0.0f;
}

/**
 * Start moving the robot forward.
 *
 * The robot starts with both tripods in the center and moves the right tripod
 * forward. The base moves by `stride/4` in `gait_period/2` seconds.
 */
void SimpleTripod::starting(Eigen::Vector3f &base, Eigen::Vector3f &left,
                            Eigen::Vector3f &right) volatile {
    // half phase, one tripod moves forward by stride/2
    // base moves by stride/4
}

/**
 * Move the robot forward by stride.
 *
 * The robot starts with the left tripod in the back and the right tripod in the
 * front. The base moves by `stride` in `gait_period` seconds.
 */
void SimpleTripod::move(Eigen::Vector3f &base, Eigen::Vector3f &left,
                        Eigen::Vector3f &right) volatile {
    bool first_half = phase < 0.5f;
    float half_phase = 2.0f * (first_half ? phase : (phase - 0.5f));

    if (first_half) {
        move_half(base, right, left, half_phase);
    } else {
        move_half(base, left, right, half_phase);
    }
}

void SimpleTripod::move_half(Eigen::Vector3f &base, Eigen::Vector3f &support,
                             Eigen::Vector3f &swing, float phase) volatile {
    // we calculate with the reference of the projection of base position on the
    // ground at the start time.
    // support doesn't change.
    support(0) = stride / 4.0f;
    support(1) = 0.0f;
    support(2) = 0.0f;

    // base moves linearly
    base(0) = phase * stride / 2.0f;
    base(1) = 0.0f;
    base(2) = base_height;

    // swing moves like a sine wave in z
    float swing_scale = -cosf(phase * 2.0f * M_PI) + 1.0f;
    swing(0) = -stride / 4.0f + phase * stride;
    swing(1) = 0.0f;
    swing(2) = leg_height * swing_scale;
}

/**
 * Stop the robot.
 *
 * The robot starts with the left tripod in the back and the right tripod in the
 * front. The base moves by `stride/4` in `gait_period/2` seconds and the left
 * tripod moves to the center with the right.
 */
void SimpleTripod::stopping(Eigen::Vector3f &base, Eigen::Vector3f &left,
                            Eigen::Vector3f &right) volatile {
    // half phase, one tripod moves forward by stride/2
    // base moves by stride/4
}
