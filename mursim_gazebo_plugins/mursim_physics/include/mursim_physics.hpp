#ifndef MURSIM_GAZEBO_MURSIM_PHYSICS_HPP
#define MURSIM_GAZEBO_MURSIM_PHYSICS_HPP

#include "connection.hpp"

namespace mursim
{
    class PhysicsPlugin : public gazebo::ModelPlugin
    {
    public:
        PhysicsPlugin();
        ~PhysicsPlugin() override;
        void Reset() override;
        void Load(gazebo::physics::ModelPtr, sdf::ElementPtr) override;

    private:
        void update();
        void publish();

        std::unique_ptr<Connection> data_ptr;
    };
}

#endif // MURSIM_GAZEBO_MURSIM_PHYSICS_HPP
