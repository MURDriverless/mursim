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
#define ACTUATION_TOPIC "/mur/control/actuation"


namespace mursim
{
    class Vehicle
    {
        public:
            Vehicle(gazebo::physics::ModelPtr, 
                    sdf::ElementPtr, 
                    std::shared_ptr<ros::NodeHandle>, 
                    gazebo::transport::NodePtr);
            
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
            double f_x = 0.0;
            
            // Launch Functions
            void launchPublishers(const std::shared_ptr<ros::NodeHandle>&);
            void launchSubscribers(const std::shared_ptr<ros::NodeHandle>&);
            void initModel(sdf::ElementPtr&);

            Params params;

            State state;
            State state_dot;
            State next_state;
            State next_state_corrected;

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
            double getTotalNormalForce() const;
            double getTotalAeroForce() const;
            double getTotalDragForce() const;
            double getAxleNormalForce(const double&, std::string) const;
            void calcStateDot();
            void calcFx();
            void kinematicCorrection(const double&);

            // Publishing Functions
            void pushModelState() const;

    };
    typedef std::unique_ptr<Vehicle> VehiclePtr;

}

#endif // MURSIM_GAZEBO_VEHICLE_HPP