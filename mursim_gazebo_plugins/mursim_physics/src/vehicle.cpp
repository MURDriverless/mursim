#include "vehicle.hpp"
#include "params.hpp"

#include <math.h>

#include <ignition/math/Vector3.hh>

#include <iostream>

namespace mursim
{
    Vehicle::Vehicle(gazebo::physics::ModelPtr model, 
                     sdf::ElementPtr sdf, 
                     std::shared_ptr<ros::NodeHandle> nh, 
                     gazebo::transport::NodePtr gznode)
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
        setPositionFromWorld();

        gzmsg << "MURSim: Vehicle Initialized" << std::endl;
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
        front_axle.calcFys(state, input_delta);
        rear_axle.calcFys(state, 0.0);

        // set the new steering angle
        front_axle.setSteerAngle(input_delta);

        // DRIVETRAIN (no torque vectoring currently)
        this->calcFx();

        // VEHICLE DYNAMICS

        // calculate time derivative 
        this->calcStateDot();

        // calculate next position with dynamic bicycle
        next_state = state + state_dot * dt;

        // do the kinematic/dynamic correction

        this->kinematicCorrection(dt);
        state = next_state_corrected;

        // PUBLISH
            // set the position, angular and linear velocity of the car in the world  (model->SetWorldPose(), 
            //                                                                         model->SetLinearVelocity(),
            //                                                                         model->SetAngularVelocity())
            // publish the transform (Tf)
        pushModelState();
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

    inline double Vehicle::getTotalDragForce() const
    {
        return params.aero.c_drag * std::pow(state.v_x, 2);
    }

    inline double Vehicle::getAxleNormalForce(const double &fz, std::string axle_pos) const
    {
        // OVERSIMPLIFIED
        if (axle_pos == "front")
        {
            return params.kinematic.w_distribution * fz;
        }
        else if (axle_pos == "rear")
        {
            return (1 - params.kinematic.w_distribution) * fz;
        }
    }

    void Vehicle::calcStateDot()
    {
        const double front_fy = front_axle.getFy();
        const double rear_fy = rear_axle.getFy();
        const double front_left_fy = front_axle.getLeftFy();
        const double front_right_fy = front_axle.getRightFy();
        const double v_x = std::max(1.0, state.v_x);

        state_dot.x =    std::cos(state.yaw) * state.v_x - std::sin(state.yaw) * state.v_y;
        state_dot.y =    std::sin(state.yaw) * state.v_x + std::cos(state.yaw) * state.v_y;
        state_dot.yaw =  state.r;
        state_dot.v_x = (state.r * state.v_y) + (f_x - std::sin(input_delta) * front_fy) / params.m_lon;
        state_dot.v_y = ((std::cos(input_delta) * front_fy) + rear_fy) / params.inertia.mass - (state.r * v_x);
        state_dot.r =   ((std::cos(input_delta) * front_fy * params.kinematic.front_lever +
                          std::sin(input_delta) * (front_left_fy - front_right_fy) * 0.5 * params.kinematic.cog_to_front) - 
                          rear_fy * params.kinematic.rear_lever) / params.inertia.i_z;
        state_dot.a_x = 0.0;
        state_dot.a_y = 0.0;
    }

    void Vehicle::calcFx()
    {
        const double wheel_radius = front_axle.getWheelRadius();
        this->f_x = input_acc * (params.drivetrain.max_torque * wheel_radius) - getTotalDragForce();
    }

    void Vehicle::kinematicCorrection(const double &dt)
    {
        next_state_corrected = next_state;
        const double v_x_dot = f_x / (params.inertia.mass + params.m_lon);
        const double v_abs   = std::hypot(state.v_x, state.v_y);
        const double v_blend = 0.5 * (v_abs - 1.5);
        const double blend   = std::fmax(std::fmin(1.0, v_blend), 0.0);

        next_state_corrected.v_x = blend * next_state.v_x + (1.0 - blend) * (state.v_x + dt * v_x_dot);

        const double v_y = std::tan(input_delta) * next_state.v_x * params.kinematic.rear_lever / params.kinematic.len;
        const double r   = std::tan(input_delta) * next_state.v_x / params.kinematic.len;

        next_state_corrected.v_y = blend * next_state.v_y + (1.0 - blend) * v_y;
        next_state_corrected.r = blend * next_state.r + (1.0 - blend) * r;
    }

    void Vehicle::pushModelState() const
    {
        const ignition::math::Pose3d   pose    {state.x, state.y, 0.0, 0.0, 0.0, state.yaw};
        const ignition::math::Vector3d vel     {state.v_x, state.v_y, 0.0};
        const ignition::math::Vector3d angular {0.0, 0.0, state.r};

        model_ptr->SetWorldPose(pose);
        model_ptr->SetLinearVel(vel);
        model_ptr->SetAngularVel(vel);
    }
}