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
        gzmsg << name << std::endl;
        joint_ptr = model_ptr->GetJoint(name);

        if (joint_ptr == nullptr)
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

    inline double Wheel::getSlipAngle() const
    {
        return this->alpha;
    }

    inline double Wheel::getFy() const
    {
        return this->f_y;
    }

    inline double Wheel::getWheelRadius() const
    {
        return this->radius;
    }
}
