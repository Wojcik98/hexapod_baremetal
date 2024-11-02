#ifndef __POSE_HPP
#define __POSE_HPP

#include <atomic>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using T3D = Eigen::Transform<float, 3, Eigen::Isometry>;

struct AtomicPose {
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> z;
    std::atomic<float> yaw;

    AtomicPose(float x = 0.0f, float y = 0.0f, float z = 0.0f, float yaw = 0.0f)
        : x(x), y(y), z(z), yaw(yaw) {}

    AtomicPose(const AtomicPose &other)
        : x(other.x.load()), y(other.y.load()), z(other.z.load()),
          yaw(other.yaw.load()) {}

    AtomicPose &operator=(const AtomicPose &other) {
        if (this == &other) {
            return *this;
        }

        x = other.x.load();
        y = other.y.load();
        z = other.z.load();
        yaw = other.yaw.load();
        return *this;
    }

    AtomicPose &operator=(const Eigen::Vector4f &other) {
        x = other(0);
        y = other(1);
        z = other(2);
        yaw = other(3);
        return *this;
    }

    AtomicPose &operator=(float other) {
        x = other;
        y = other;
        z = other;
        yaw = other;
        return *this;
    }

    float norm() const { return Eigen::Vector4f(x, y, z, yaw).norm(); }
};

struct Pose {
    float x;
    float y;
    float z;
    float yaw;

    Pose(float x = 0.0f, float y = 0.0f, float z = 0.0f, float yaw = 0.0f)
        : x(x), y(y), z(z), yaw(yaw) {}

    Pose(const AtomicPose &other)
        : x(other.x), y(other.y), z(other.z), yaw(other.yaw) {}

    Pose &operator=(const Eigen::Vector4f &other) {
        x = other(0);
        y = other(1);
        z = other(2);
        yaw = other(3);
        return *this;
    }

    Pose &operator=(const T3D &other) {
        auto rot = other.rotation();
        x = other.translation()(0);
        y = other.translation()(1);
        z = other.translation()(2);
        yaw = atan2(rot(1, 0), rot(0, 0));
        return *this;
    }

    Pose &operator=(float other) {
        x = other;
        y = other;
        z = other;
        yaw = other;
        return *this;
    }

    Pose operator+(const Pose &other) const {
        return {x + other.x, y + other.y, z + other.z, yaw + other.yaw};
    }

    Pose operator-(const Pose &other) const {
        return {x - other.x, y - other.y, z - other.z, yaw - other.yaw};
    }

    Pose operator*(float other) const {
        return {x * other, y * other, z * other, yaw * other};
    }

    Eigen::Vector3f translation() const { return {x, y, z}; }

    T3D toEigen() const {
        auto pose = T3D::Identity();
        auto rot = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        pose.fromPositionOrientationScale(translation(), rot,
                                          Eigen::Vector3f::Ones());
        return pose;
    }

    float norm() const { return Eigen::Vector4f(x, y, z, yaw).norm(); }

    Pose interpolate(const Pose &other, float alpha) const {
        return *this + (other - *this) * alpha;
    }
};

using Twist2D = Pose;
using AtomicTwist2D = AtomicPose;

#endif // __POSE_HPP
