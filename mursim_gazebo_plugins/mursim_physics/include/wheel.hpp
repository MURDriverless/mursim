#ifndef MURSIM_GAZEBO_WHEEL_HPP
#define MURSIM_GAZEBO_WHEEL_HPP

#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>
#include <ros/ros.h>

namespace mursim 
{
    class Wheel 
    {
    public:
        Wheel(gazebo::physics::ModelPtr &model,
              sdf::ElementPtr &sdf,
              std::string name,
              gazebo::transport::NodePtr &gznode,
              std::shared_ptr<ros::NodeHandle> &nh_ptr);
        
        double getFy(const double&, const double&) const;
        void setAngle(const double&);
    }
}

#endif // MURSIM_GAZEBO_WHEEL_HPP