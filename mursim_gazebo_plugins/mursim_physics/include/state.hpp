#ifndef MURSIM_GAZEBO_STATE_HPP
#define MURSIM_GAZEBO_STATE_HPP

namespace mursim
{
    struct State
    {
        double x;
        double y;
        double yaw;
        double v_x;
        double v_y;
        double r;
        double a_x;
        double a_y;

    };
}

#endif // MURSIM_GAZEBO_STATE_HPP