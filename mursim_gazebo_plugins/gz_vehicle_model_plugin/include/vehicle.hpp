/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef MURSIM_GAZEBO_VEHICLE_HPP
#define MURSIM_GAZEBO_VEHICLE_HPP

#include "mursim_common/car_state_msg.h"
#include "mursim_common/car_info_msg.h"
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
#define VEHICLE_INFO_TOPIC "/mursim/car_info"
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
            void pushWheelOrientation() const;
            void pushCarInfo() const;

    };

    typedef std::unique_ptr<Vehicle> VehiclePtr;
}

#endif // MURSIM_GAZEBO_VEHICLE_HPP