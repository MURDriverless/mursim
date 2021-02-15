
/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "axle.hpp"


namespace mursim
{
    Axle::Axle(const gazebo::physics::ModelPtr &model,
               const sdf::ElementPtr sdf,
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

        gzmsg << "MURSim: " << name << " axle initialized!" << std::endl;
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

    void Axle::calcFys(const State &state, const double &delta)
    {
        l_wheel.setSlipAngle(calcSlipAngle(state, delta, name));
        r_wheel.setSlipAngle(calcSlipAngle(state, delta, name));
        l_wheel.calcFy(f_z / 2.0);
        r_wheel.calcFy(f_z / 2.0);
        this->f_y = l_wheel.getFy() + r_wheel.getFy();
    }

    void Axle::setFz(const double &fz)
    {
        this->f_z = fz;
    }

    inline double Axle::calcSlipAngle(const State &state, const double &delta, const std::string &position)
    {
        double v_x = std::max(1.0, state.v_x);
        double alpha;

        if (position == "left")
        {
	    if (name == "front")
	    {
                alpha = std::atan((state.v_y + params.cog_to_front * state.r) /
                           (v_x - 0.5 * axle_width * state.r)) - delta;
	    }
	    else
	    {
                alpha = std::atan((state.v_y - params.cog_to_rear * state.r) /
                          (v_x - 0.5 * axle_width * state.r));
	    }

        }
        else 
        {
	    if (name == "front")
	    {
                alpha = std::atan((state.v_y + params.cog_to_front * state.r) /
                          (v_x + 0.5 * axle_width * state.r)) - delta;
	    }
	    else 
	    {
                alpha = std::atan((state.v_y - params.cog_to_rear * state.r) /
                          (v_x + 0.5 * axle_width * state.r));
	    }
        }
            
        return alpha; 
    }

     double Axle::getLeftSlipAngle() const
     {
         return l_wheel.getSlipAngle();
     }

     double Axle::getRightSlipAngle() const
     {
         return r_wheel.getSlipAngle();
     }

     double Axle::getFz() const
     {
         return f_z;
     }

     double Axle::getFy() const
     {
         return f_y;
     }

     double Axle::getLeftFy() const
     {
         return l_wheel.getFy();
     }

     double Axle::getRightFy() const
     {
         return r_wheel.getFy();
     }

     double Axle::getWheelRadius() const
     {
         return l_wheel.getWheelRadius();
     }

     void Axle::getJointNames(std::string &left, std::string &right) const
     {
        left = l_wheel.getJointName();
        right = r_wheel.getJointName();
     }
}
