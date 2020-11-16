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
              const std::string name,
              const gazebo::transport::NodePtr &gznode,
              const std::shared_ptr<ros::NodeHandle> &nh_ptr);

        void calcFy(const double&);

        // Getters
        ignition::math::Vector3d getCentrePos() const;

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
        void getJoint();
        void getCollisionRadius();

        std::string name;
        uint8_t id = 0;
    };
}

#endif // MURSIM_GAZEBO_WHEEL_HPP