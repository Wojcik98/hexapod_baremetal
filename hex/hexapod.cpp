#include "hexapod.hpp"
#include "legs.hpp"

#include <cmath>

void Hexapod::inverse_kinematics(const LegsVec3f &ee_base, Legsf &angles) {
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        Eigen::Vector3f ee_hip = ee_base[i] - hip_offsets[i];
        inverse_kinematics_leg(ee_hip, angles[i]);

        angles[i].coxa += angles_offsets[i].coxa;
        angles[i].femur += angles_offsets[i].femur;
        angles[i].tibia += angles_offsets[i].tibia;
    }
}

void Hexapod::inverse_kinematics_leg(const Eigen::Vector3f &point,
                                     Legf &angles) {
    float x = point(0);
    float y = point(1);
    float z = point(2);

    float l1 = leg_dims.coxa;
    float l2 = leg_dims.femur;
    float l3 = leg_dims.tibia;

    float w = sqrtf(x * x + y * y) - l1;
    float r = sqrtf(w * w + z * z);

    float theta1 = atan2f(y, x);
    float delta = acosf((l2 * l2 + l3 * l3 - r * r) / (2 * l2 * l3));
    float gamma = M_PI - delta;
    float b1 = atan2f(w, -z);
    float b2 = acosf((l2 * l2 + r * r - l3 * l3) / (2 * l2 * r));
    float beta = b1 + b2;

    angles.coxa = theta1;
    angles.femur = beta - M_PI / 2;
    angles.tibia = -gamma;
}
