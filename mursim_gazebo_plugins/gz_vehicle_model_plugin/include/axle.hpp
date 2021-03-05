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

#ifndef MURSIM_GAZEBO_AXLE_HPP
#define MURSIM_GAZEBO_AXLE_HPP

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

#include <math.h>

#include "wheel.hpp"
#include "params.hpp"
#include "state.hpp"


namespace mursim
{
    class Axle
    {
    public:
        Axle(const gazebo::physics::ModelPtr&,
             const sdf::ElementPtr,
             const std::string &name,
             const gazebo::transport::NodePtr&,
             const std::shared_ptr<ros::NodeHandle>&);
        
        double calcSlipAngles();
        void calcFys(const State&, const double&);

        // Getters
        double getLeftSlipAngle() const;
        double getRightSlipAngle() const;
        double getFz() const;
        double getFy() const;
        double getLeftFy() const;
        double getRightFy() const;
        double getWheelRadius() const;
        void getJointNames(std::string&, std::string&) const;
        
        const ignition::math::Vector3d getCentrePos() const;

        // Setters
        void setFz(const double&);
        void setSteerAngle(const double&);

        std::string name;

    private:
        double f_z;
        double f_y;

        double axle_factor;
        double delta = 0.0;

        Wheel l_wheel;
        Wheel r_wheel;

        ignition::math::Vector3d axle_centre_pos;
        Params::Kinematic params;

        inline double calcSlipAngle(const State&, const double&, const std::string&);

        double axle_width; 
        double car_length; // DEPRECATED in Params
        double cog_to_axle; // DEPRECATED in Params
        double cog_position; // DEPRECATED in Params
        
    };        
}

#endif // MURSIM_GAZEBO_AXLE_HPP
