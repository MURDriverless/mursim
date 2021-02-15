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

#include "wheel.hpp"

#include <math.h>

namespace mursim
{
    Wheel::Wheel(const gazebo::physics::ModelPtr &model,
                 const sdf::ElementPtr &sdf,
                 const std::string &obj_name,
                 const gazebo::transport::NodePtr &gznode,
                 const std::shared_ptr<ros::NodeHandle> &nh_ptr)
                : model_ptr(model),
                  name(model->GetName() + "::" + sdf->Get<std::string>(obj_name))
    {
	// Calculate or find parameters
        this->getJoint();
        this->getCollisionRadius();
        this->findCentrePos();

        gzmsg << "MURSim: " << joint_ptr->GetChild()->GetName() << " initialized" << std::endl;
    }

    ignition::math::Vector3d Wheel::getCentrePos() const
    {
        return centre_pos;
    }

    void Wheel::findCentrePos()
    {
        auto collision_ptr = joint_ptr->GetChild()->GetCollision(id);
        if (collision_ptr == nullptr)
        {
            gzerr << "MURSim: Error finding collision for joint: " << name << std::endl;
            return;
        }
        else 
        {
            gzmsg << "MURSim: Collision for " << name << " found" << std::endl;
            this->centre_pos = collision_ptr->WorldPose().Pos();
        }
    }

    void Wheel::getJoint()
    {
        this->joint_ptr = this->model_ptr->GetJoint(name);

        if (this->joint_ptr == nullptr)
        {
            gzerr << "MURSim: Could not find " << name << " link!" << std::endl;
        }
        else
        {
            gzmsg << "MURSim: Successfully initialized joint: " << name <<  std::endl;
        }
        
    }

    void Wheel::getCollisionRadius()
    {
        gazebo::physics::CollisionPtr coll_ptr = joint_ptr->GetChild()->GetCollision(id);
        if (coll_ptr == nullptr)
        {
            gzerr << "MURSim: Could not find child to " << name << std::endl; 
            return;
        }

        auto coll_shape_ptr = static_cast<gazebo::physics::CylinderShape*>(coll_ptr->GetShape().get());
        this->radius = coll_shape_ptr->GetRadius();
    }

    void Wheel::calcFy(const double &f_z)
    {
        const double mu_y = tire_params.D * std::sin(tire_params.C * std::atan(tire_params.B * (1.0 - tire_params.E) * alpha +
                                                     tire_params.E * std::atan(tire_params.B * alpha)));
        this->f_y = f_z * mu_y;
    }

    void Wheel::setAngle(const double &delta)
    {
        this->delta = delta;
    }

    void Wheel::setSlipAngle(const double &alpha)
    {
        this->alpha = alpha;
    }

    double Wheel::getSlipAngle() const
    {
        return this->alpha;
    }

    double Wheel::getFy() const
    {
        return this->f_y;
    }

    double Wheel::getWheelRadius() const
    {
        return this->radius;
    }

    const std::string Wheel::getJointName() const
    {
        return this->name;
    }
}
