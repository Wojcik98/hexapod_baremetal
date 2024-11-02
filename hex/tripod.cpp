#include "tripod.hpp"

// extern "C" {
// #include <stdint.h>
// }
// #include <cmath>

#include "legs.hpp"
#include "pose.hpp"

Tripod::Tripod(Hexapod &robot, TripodConfig &config)
    : Gait(robot), servo_freq(config.servo_freq),
      iters_per_period(servo_freq * config.gait_period),
      gait_period((float)iters_per_period / servo_freq),
      base_height(config.base_height), leg_height(config.leg_height),
      config(config), iter(0), left_swing(true) {
    base = Eigen::Vector4f{0, 0, base_height, 0};
    base_start = base;
}

void Tripod::set_twist(const AtomicTwist2D &twist) {
    float absolute = twist.norm();

    if (absolute < VEL_EPS) {
        next_twist = 0;
        if (state != STOP) {
            state = STOP_REQUESTED;
        }
    } else {
        next_twist = twist;
        if (state == STOP) {
            state = MOVE;
            iter = 0;
        }
    }
}

void Tripod::get_next_joints(Legsf &angles) {
    if (state == STOP) {
        // do nothing
    } else if (state == MOVE || state == STOP_REQUESTED) {
        move();
        iter++;
    }

    calculate_joints(angles);

    if (iter >= iters_per_period) {
        iter = 0;
        calculate_next_step();
    }

    // wait for the period to end
    if (state == STOP_REQUESTED && iter == 0) {
        state = STOP;
    }
}

/**
 * Move the robot with the set twist.
 */
void Tripod::move() {
    // we calculate with the reference of the projection of base position on the
    // ground at the start time.
    // support doesn't change.

    float phase = (float)iter / iters_per_period;
    auto &swing = left_swing ? left : right;
    auto &support = left_swing ? right : left;

    swing = swing_start.interpolate(swing_target, phase);
    base = base_start.interpolate(base_target, phase);

    // swing moves like a sine wave in z
    float swing_scale = -cosf(phase * 2.0f * M_PI) + 1.0f;
    swing.z = leg_height * swing_scale;
}

void Tripod::calculate_next_step() {
    left_swing = !left_swing;
    twist = Twist2D(next_twist);

    // move to "world" frame to the projection of the base on the ground
    T3D projection = base.toEigen();
    projection.translation()(2) = 0.0f;
    T3D inv = projection.inverse();
    left = inv * left.toEigen();
    right = inv * right.toEigen();
    base = Eigen::Vector4f{0, 0, base_height, 0};
    base_start = base;

    // calculate the target positions for the next step
    base_target = base + twist * gait_period;

    swing_target = base_target + twist * (gait_period / 2.0f);
    swing_target.z = 0.0f;
    swing_start = left_swing ? left : right;
}

void Tripod::calculate_joints(Legsf &out) {
    auto inv = base.toEigen().inverse();
    auto left_base = inv * left.toEigen();
    auto right_base = inv * right.toEigen();

    // calculate end effectors' positions w.r.t. base
    LegsVec3f ee_base = {
        (left_base * config.left_tripod.front_trans).translation(),
        (right_base * config.right_tripod.mid_trans).translation(),
        (left_base * config.left_tripod.rear_trans).translation(),
        (right_base * config.right_tripod.front_trans).translation(),
        (left_base * config.left_tripod.mid_trans).translation(),
        (right_base * config.right_tripod.rear_trans).translation(),
    };

    // calculate joint angles
    robot.inverse_kinematics(ee_base, out);
}
