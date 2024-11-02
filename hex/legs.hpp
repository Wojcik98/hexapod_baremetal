#ifndef __LEGS_HPP
#define __LEGS_HPP

#include <cstdint>

#include <Eigen/Dense>

constexpr float M_PI = 3.1415926535f;
constexpr float M_PI_2 = M_PI / 2.0f;

template <typename T> struct Legs {
    T fl;
    T ml;
    T rl;
    T fr;
    T mr;
    T rr;

    T operator[](uint8_t i) const {
        switch (i) {
        case 0:
            return fl;
        case 1:
            return ml;
        case 2:
            return rl;
        case 3:
            return fr;
        case 4:
            return mr;
        case 5:
            return rr;
        default:
            return fl;
        }
    }

    T &operator[](uint8_t i) {
        switch (i) {
        case 0:
            return fl;
        case 1:
            return ml;
        case 2:
            return rl;
        case 3:
            return fr;
        case 4:
            return mr;
        case 5:
            return rr;
        default:
            return fl;
        }
    }
};

template <typename T> struct Leg {
    T coxa;
    T femur;
    T tibia;
};

using Legf = Leg<float>;
using LegsVec3f = Legs<Eigen::Vector3f>;
using Legsf = Legs<Legf>;

#endif // __LEGS_HPP
