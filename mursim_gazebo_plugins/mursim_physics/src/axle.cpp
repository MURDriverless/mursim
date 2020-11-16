#include "axle.hpp"

namespace mursim
{
    Axle::Axle(const gazebo::physics::ModelPtr &model,
               const sdf::ElementPtr &sdf,
               const std::string &name,
               const gazebo::transport::NodePtr &gznode,
               const std::shared_ptr<ros::NodeHandle> &nh) 
               : l_wheel(model, sdf, name + "_left_wheel", gznode, nh),
                 r_wheel(model, sdf, name + "_right_wheel", gznode, nh)
    {
        const auto axle_temp = l_wheel.getCentrePos() - r_wheel.getCentrePos(); // ignition::math::Vector3d

        this->axle_width = axle_temp.Length();
        this->axle_centre_pos = (l_wheel.getCentrePos() + r_wheel.getCentrePos()) / 2.0;
        this->name = name;
        this->axle_factor = name == "front" ? 1.0 : -1.0;
    }

    const ignition::math::Vector3d Axle::getCentrePos() const
    {
        return axle_centre_pos;
    }

    void Axle::setSteerAngle(const double &delta)
    {
        this->delta = delta;
        l_wheel.setAngle(delta);
        r_wheel.setAngle(delta);
    }

    double Axle::getFy(const State &state, const double &delta)
    {
        l_wheel.setSlipAngle(calcSlipAngle(state, delta));
        r_wheel.setSlipAngle(calcSlipAngle(state, delta));
        l_wheel.calcFy(f_z);
        r_wheel.calcFy(f_z);
    }

    void Axle::setFz(const double &fz)
    {
        this->f_z = fz;
    }

    inline double Axle::calcSlipAngle(const State &state, const double &delta)
    {
        double v_x = std::max(1.0, state.v_x);

        return std::atan((state.v_y + axle_factor * params.axle_lever * state.r)/
                         (state.v_x - 0.5 * axle_width * state.r)) - delta; // minus delta
    }
}