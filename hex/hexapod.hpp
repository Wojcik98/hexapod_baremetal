#ifndef __HEXAPOD_HPP
#define __HEXAPOD_HPP

#include <cstdint>

#include <Eigen/Dense>

#include "legs.hpp"

constexpr uint8_t NUM_LEGS = 6;
constexpr uint8_t NUM_JOINTS_PER_LEG = 3;
constexpr uint8_t NUM_JOINTS = NUM_LEGS * NUM_JOINTS_PER_LEG;

struct Hexapod {
    void inverse_kinematics(const LegsVec3f &ee_base, Legsf &angles);
    void inverse_kinematics_leg(const Eigen::Vector3f &point, Legf &angles);

    Legsf angles_offsets;
    LegsVec3f hip_offsets;
    Legf leg_dims;
};

#endif /* __HEXAPOD_HPP */
