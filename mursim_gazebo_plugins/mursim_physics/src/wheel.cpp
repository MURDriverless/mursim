#include "wheel.hpp"

#include <math.h>

namespace mursim
{
    Wheel::Wheel(const gazebo::physics::ModelPtr &model,
                 const sdf::ElementPtr &sdf,
                 const std::string name,
                 const gazebo::transport::NodePtr &gznode,
                 const std::shared_ptr<ros::NodeHandle> &nh_ptr)
                : model_ptr(model)
    {
        this->name = model->GetName() + "::" + sdf->Get<std::string>(name);
        this->getJoint();
        this->getCollisionRadius();
        this->centre_pos = joint_ptr->GetChild()->GetCollision(id)->WorldPose().Pos();
    }

    ignition::math::Vector3d Wheel::getCentrePos() const
    {
        return centre_pos;
    }

    void Wheel::getJoint()
    {
        joint_ptr = model_ptr->GetJoint(name);

        if (joint_ptr == nullptr)
            gzerr << "Could not find " << name << " link!" << std::endl;
    }

    void Wheel::getCollisionRadius()
    {
        gazebo::physics::CollisionPtr coll_ptr = joint_ptr->GetChild()->GetCollision(id);
        auto coll_shape_ptr = static_cast<gazebo::physics::CylinderShape*>(coll_ptr->GetShape().get());
        this->radius = coll_shape_ptr->GetRadius();
    }

    void Wheel::calcFy(const double &f_z)
    {
        const double mu_y = tire_params.D * std::sin(tire_params.C * std::atan(tire_params.B * (1.0 - tire_params.E) * alpha +
                                                                               tire_params.E * std::atan(tire_params.B * alpha)));
        
        f_y = f_z * mu_y;
        
    }

    void Wheel::setAngle(const double &delta)
    {
        this->delta = delta;
    }

    void Wheel::setSlipAngle(const double &alpha)
    {
        this->alpha = alpha;
    }
}
