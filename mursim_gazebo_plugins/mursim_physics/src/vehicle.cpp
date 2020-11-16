#include "vehicle.hpp"
#include "params.hpp"

#include <math.h>

#include <ignition/math/Vector3.hh>

namespace mursim
{
    Vehicle::Vehicle(const gazebo::physics::ModelPtr &model, 
                     const sdf::ElementPtr &sdf, 
                     const std::shared_ptr<ros::NodeHandle> &nh, 
                     const gazebo::transport::NodePtr &gznode)
                    : nh(nh),
                    model_ptr(model),
                    front_axle(model, sdf, "front", gznode, nh),
                    rear_axle(model, sdf, "rear", gznode, nh)
    {
        launchPublishers(nh);
        launchSubscribers(nh);

        // Load the model and vehicle parameters into the Vehicle model

        // Initialize all parameters for axles and tires

        // Set the vehicle position in the world (should be taken in as a param)

    }

    void Vehicle::update(const double &dt)
    {
        // TIRE DYNAMICS

        // determine the z-axis load on the vehicle
        double normal = getTotalNormalForce();

        // determine the fy values for each tire
        front_axle.setFz(getAxleNormalForce(normal, "front"));
        rear_axle.setFz(getAxleNormalForce(normal, "rear"));

        // detemrine f_y for each of the tires
        front_axle.getFy(state, input_delta);
        rear_axle.getFy(state, 0.0);

        // set the new steering angle
        front_axle.setSteerAngle(input_delta);

        // DRIVETRAIN (no torque vectoring currently)
            // get fx

        // VEHICLE DYNAMICS
            // calculate current x-velocity
            // calculate next time step position
            // do the kinematic/dynamic correction
        
        // PUBLISH
            // set the position, angular and linear velocity of the car in the world  (model->SetWorldPose(), 
            //                                                                         model->SetLinearVelocity(),
            //                                                                         model->SetAngularVelocity())
            // publish the transform (Tf)
    }
    void Vehicle::launchPublishers(const std::shared_ptr<ros::NodeHandle> &nh) 
    {        
        pub_ground_truth = nh->advertise<mursim_common::car_state_msg>(VEHICLE_GROUND_TRUTH_TOPIC, 1);
    }

    void Vehicle::launchSubscribers(const std::shared_ptr<ros::NodeHandle> &nh)
    {
        sub_actuation = nh->subscribe(ACTUATION_TOPIC, 1, &Vehicle::actuationCallback, this);
    }

    void Vehicle::actuationCallback(const mur_common::actuation_msg &msg)
    {
        input_delta = msg.steering;
        input_acc = msg.acceleration_threshold;
    }

    void Vehicle::initModel(sdf::ElementPtr &sdf)
    {
        chassis_link_name = model_ptr->GetName() + "::" + sdf->Get<std::string>("chassis");
        base_link_name = model_ptr->GetName() + "::" + sdf->Get<std::string>("base_link");

        chassis_link_ptr = model_ptr->GetLink(chassis_link_name);
        base_link_ptr = model_ptr->GetLink(base_link_name);
    }
    
    void Vehicle::setPositionFromWorld()
    {
        auto pos = model_ptr->WorldPose(); // ignition::math::Pose3d 

        state.x = pos.Pos().X();
        state.y = pos.Pos().Y();
        state.yaw = 0.0;
        state.v_x = 0.0;
        state.v_y = 0.0;
        state.r = 0.0;
        state.a_x = 0.0;
        state.a_y = 0.0;
    }

    inline double Vehicle::getTotalNormalForce() const
    {
        return params.inertia.mass * params.inertia.g + getTotalAeroForce(); 
    }

    inline double Vehicle::getTotalAeroForce() const
    {
        return params.aero.c_down * std::pow(state.v_x, 2);
    }

    inline double Vehicle::getAxleNormalForce(const double &fz, std::string axle_pos) const
    {
        // OVERSIMPLIFIED
        if (axle_pos == "front")
            return params.kinematic.w_distribution * fz;
        else if (axle_pos == "rear")
            return (1 - params.kinematic.w_distribution) * fz;
    }
}