#ifndef MURSIM_GAZEBO_CONFIG_HPP
#define MURSIM_GAZEBO_CONFIG_HPP

#include "yaml-cpp/yaml.h"
#include <math.h>

namespace mursim
{
    struct Params 
    {
        struct Inertia 
        {
            static constexpr double mass = 190.0;        // Weight of vehicle [kg] static const double GRAVITY = 9.81;        // Gravitational acceleration [m/s2]
            static constexpr double i_z = 110;           // Inertial force 
            static constexpr double g = 9.81;		 // Gravitational acceleration (m/s2)
        };

        struct Kinematic
        {
            static constexpr double len = 1.53;
            static constexpr double cog_to_front = 1.22;
            static constexpr double cog_to_rear = 1.22;
            static constexpr double cog_to_z = 0.262;            // height of COG
            static constexpr double w_distribution = 0.5;        // percentage of weight front

            static constexpr double front_lever = len * (1 - w_distribution);
            static constexpr double rear_lever = len * w_distribution;

        };

        struct Drivetrain
        {
            static constexpr double max_torque = 100;
            static constexpr double num_wheels = 4;
            static constexpr double inertia = 0.4;
            static constexpr double r_dyn = 0.231;
        };

        struct Tire
        {
            static constexpr double coefficient = 1.0;
            static constexpr double B = 12.56;
            static constexpr double C = -1.38;
            static constexpr double D = 1.60;
            static constexpr double E = -0.58;
        };

        struct Aero
        {
            struct CDown
            {
                static constexpr double a = 1.22;
                static constexpr double b = 2.6;
                static constexpr double c = 0.6;
            };

            struct CDrag
            {
                static constexpr double a = 0.7;
                static constexpr double b = 1.0;
                static constexpr double c = 1.0;
            };

            static constexpr double c_down = CDown::a * CDown::b * CDown::c;
            static constexpr double c_drag = CDrag::a * CDrag::b * CDrag::c;
        };

        Inertia inertia;
        Kinematic kinematic;
        Drivetrain drivetrain;
        Aero aero;

        static constexpr double m_lon = Inertia::mass +
                                        Drivetrain::num_wheels * Drivetrain::inertia / (Drivetrain::r_dyn * Drivetrain::r_dyn);
    };

}

#endif // MURSIM_GAZEBO_CONFIG_HPP
