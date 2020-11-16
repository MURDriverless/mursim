#ifndef MURSIM_GAZEBO_VEHICLE_HPP
#define MURSIM_GAZEBO_VEHICLE_HPP

#include "mursim_common/car_state_msg.h"
#include "mur_common/actuation_msg.h"

#include "params.hpp"
#include "axle.hpp"
#include "state.hpp"

#include <ros/ros.h>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

#define VEHICLE_GROUND_TRUTH_TOPIC "/mursim/vehicle_ground_truth"
#define ACTUATION_TOPIC "/mursim/vehicle_ground_truth"


namespace mursim
{
    class Vehicle
    {
        public:
            Vehicle(const gazebo::physics::ModelPtr&, 
                    const sdf::ElementPtr&, 
                    const std::shared_ptr<ros::NodeHandle>&, 
                    const gazebo::transport::NodePtr&);
            
            void update(const double&);

        private:
            std::shared_ptr<ros::NodeHandle> nh;
            gazebo::physics::ModelPtr model_ptr;

            // Link Pointers
            gazebo::physics::LinkPtr chassis_link_ptr;
            gazebo::physics::LinkPtr base_link_ptr;

            // ROS Publishers
            ros::Publisher pub_ground_truth;
            ros::Publisher pub_car_info;

            // ROS Subscribers
            ros::Subscriber sub_actuation;

            // ROS Callbacks
            void actuationCallback(const mur_common::actuation_msg&); 
            double input_delta = 0.0; 
            double input_acc = 0.0;
            
            // Launch Functions
            void launchPublishers(const std::shared_ptr<ros::NodeHandle>&);
            void launchSubscribers(const std::shared_ptr<ros::NodeHandle>&);
            void initModel(sdf::ElementPtr&);

            Params params;
            State state;

            void setPositionFromWorld();

            // Vehicle Components
            Axle front_axle;
            Axle rear_axle;
            
            // Vehicle Identification
            std::string robot_name;
            std::string chassis_link_name;
            std::string base_link_name;

            // CALCULATION FUNCTIONS
            // Normal Forces
            inline double getTotalNormalForce() const;
            inline double getTotalAeroForce() const;
            inline double getAxleNormalForce(const double&, std::string) const;

    };
    typedef std::unique_ptr<Vehicle> VehiclePtr;

}

#endif // MURSIM_GAZEBO_VEHICLE_HPP