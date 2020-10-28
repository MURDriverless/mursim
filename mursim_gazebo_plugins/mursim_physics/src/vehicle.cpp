#include "vehicle.hpp"

namespace mursim
{
    Vehicle::Vehicle(const gazebo::physics::ModelPtr& model, const sdf::ElementPtr& sdf, 
                     const std::shared_ptr<ros::NodeHandle>& nh, const gazebo::transport::NodePtr& gznode)
            : nh(nh)
              // model
              // front axle
              // rear axle
              // aero(param.aero)
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
            // determine the fy values for each tire
            // detemrine m_z for each of the front tires
            // set the new steering angle

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

    void Vehicle::launchPublishers(const ros::NodeHandle &nh) 
    {        
        pub_ground_truth = nh->advertise<mursim::car_state_msg>(VEHICLE_GROUND_TRUTH_PUB, 1);
    }

    void Vehicle::launchSubscribers(const ros::NodeHandle &nh)
    {

    }
}