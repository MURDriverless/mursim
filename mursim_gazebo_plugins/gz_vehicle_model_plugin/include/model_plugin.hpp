#ifndef MURSIM_GAZEBO_MODEL_PLUGIN_HPP
#define MURSIM_GAZEBO_MODEL_PLUGIN_HPP

#include "connection.hpp"

namespace mursim
{
    class ModelPlugin: public gazebo::ModelPlugin
    {
    public:
        ModelPlugin();
        ~ModelPlugin() override;
        void Reset() override;
        void Load(gazebo::physics::ModelPtr, sdf::ElementPtr) override;

    private:
        void update();
        void publish();

        std::unique_ptr<Connection> data_ptr;
    };
}

#endif // MURSIM_GAZEBO_MODEL_PLUGIN_HPP
