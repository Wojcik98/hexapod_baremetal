#include "hexapod.hpp"
#include "legs.hpp"

class Gait {
public:
    Gait(Hexapod &robot) : robot(robot){};
    virtual ~Gait() = default;

    virtual void get_next_joints(Legsf &angles) = 0;

protected:
    Hexapod &robot;
};