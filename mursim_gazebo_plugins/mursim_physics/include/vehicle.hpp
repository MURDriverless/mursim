#ifndef MURSIM_GAZEBO_VEHICLE_HPP
#define MURSIM_GAZEBO_VEHICLE_HPP

#include <ros/ros.h>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

typedef std::unique_ptr<mursim::Vehicle> VehiclePtr;

#define VEHICLE_GROUND_TRUTH_PUB "/mursim/vehicle_ground_truth"


namespace mursim
{
    class Vehicle
    {
        public:
            Vehicle(const gazebo::physics::ModelPtr&, const sdf::ElementPtr&, 
                    const std::shared_ptr<ros::NodeHandle>&, const gazebo::transport::NodePtr&);
            
            void update(const double&);

        private:
            std::shared_ptr<ros::NodeHandle> nh;
            gazebo::physics::ModelPtr model_ptr;

            // ROS Publishers
            ros::Publisher pub_ground_truth;
            ros::Publisher pub_car_info;

            void launchPublishers(const ros::NodeHandle&);
            void launchSubscribers(const ros::NodeHandle&);

    };
}

#endif // MURSIM_GAZEBO_VEHICLE_HPP