#ifndef MURSIM_GAZEBO_STATE_HPP
#define MURSIM_GAZEBO_STATE_HPP

namespace mursim
{
    struct State
    {
        State operator*(const double &dt) const
        {
            return {dt * x,
                    dt * y,
                    dt * yaw,
                    dt * v_x,
                    dt * v_y,
                    dt * r,
                    dt * a_x,
                    dt * a_y};
        };

        State operator+(const State &state_two) const
        {
            return {x + state_two.x,
                    y + state_two.y,
                    yaw + state_two.yaw,
                    v_x + state_two.v_x,
                    v_y + state_two.v_y,
                    r + state_two.r,
                    a_x + state_two.a_x,
                    a_y + state_two.a_y};
        };

        inline std::string getString() const 
        {
            std::string str = "x: "   + std::to_string(x)
                         + " | y: "   + std::to_string(y)
                         + " | yaw: " + std::to_string(yaw)
                         + " | v_x: " + std::to_string(v_x)
                         + " | v_y: " + std::to_string(v_y)
                         + " | r: "   + std::to_string(r)
                         + " | a_x: " + std::to_string(a_x)
                         + " | a_y: " + std::to_string(a_y);
            return str;
        }

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