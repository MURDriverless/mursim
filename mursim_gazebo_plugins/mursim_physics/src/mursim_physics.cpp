#include "mursim_physics.hpp"

namespace mursim
{
    PhysicsPlugin::PhysicsPlugin() : data_ptr(new Connection)
    {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "mursim_physics_plugin");
        data_ptr->nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    }

    PhysicsPlugin::~PhysicsPlugin()
    {
        data_ptr->connection_ptr.reset();
    }

    void PhysicsPlugin::Load(gazebo::physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
    {
        data_ptr->model_ptr = model_ptr;
        data_ptr->world_ptr = data_ptr->model_ptr->GetWorld();
        data_ptr->gznode_ptr = gazebo::transport::NodePtr(new gazebo::transport::Node());
        data_ptr->gznode_ptr->Init();

        // Create new vehicle

        // Bind reference to physics update function
        gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&PhysicsPlugin::update, this));

        data_ptr->world_control_pub_ptr = data_ptr->gznode_ptr->Advertise<gazebo::msgs::WorldControl>("~/world_control");
        data_ptr->last_sim_time = data_ptr->world_ptr->SimTime();
    }

    GZ_REGISTER_MODEL_PLUGIN(PhysicsPlugin)
}
