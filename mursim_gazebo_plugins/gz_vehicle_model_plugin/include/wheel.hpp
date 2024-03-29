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


#ifndef MURSIM_GAZEBO_WHEEL_HPP
#define MURSIM_GAZEBO_WHEEL_HPP

#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>

#include "params.hpp"

namespace mursim 
{
    class Wheel 
    {
    public:
        Wheel(const gazebo::physics::ModelPtr &model,
              const sdf::ElementPtr &sdf,
              const std::string &name,
              const gazebo::transport::NodePtr &gznode,
              const std::shared_ptr<ros::NodeHandle> &nh_ptr);
        
        Wheel();

        void calcFy(const double&);

        // Getters
        ignition::math::Vector3d getCentrePos() const;
        double getSlipAngle() const;
        double getFy() const;
        double getWheelRadius() const;
        const std::string getJointName() const;

        // Setters
        void setAngle(const double&);
        void setSlipAngle(const double&);
        
    private:

        gazebo::physics::ModelPtr model_ptr;
        gazebo::physics::JointPtr joint_ptr;

        Params::Tire tire_params;

        double f_y;
        double delta = 0.0;
        double alpha;
        double radius;

        ignition::math::Vector3d centre_pos;

        void calcSlipAngle();
        void calcYFrictionCoeff();
        void getJoint();
        void findCentrePos();
        void getCollisionRadius();

        std::string name;
        uint8_t id = 0;
    };
}

#endif // MURSIM_GAZEBO_WHEEL_HPP