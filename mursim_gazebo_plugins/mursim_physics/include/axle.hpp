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
