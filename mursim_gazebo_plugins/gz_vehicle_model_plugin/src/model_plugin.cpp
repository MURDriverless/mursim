#include "model_plugin.hpp"
#include "vehicle.hpp"

namespace mursim
{
    ModelPlugin::ModelPlugin() 
        : data_ptr(new Connection)
    {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "mursim_model_plugin");
        data_ptr->nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    }

    ModelPlugin::~ModelPlugin()
    {
        data_ptr->connection_ptr.reset();
    }

    void ModelPlugin::Load(gazebo::physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr) // Overloaded
    {
        data_ptr->model_ptr = model_ptr;

        gzmsg << "MURSim: Attached to robot named: " << data_ptr->model_ptr->GetName() << std::endl;

        data_ptr->world_ptr = data_ptr->model_ptr->GetWorld();
        data_ptr->gznode_ptr = gazebo::transport::NodePtr(new gazebo::transport::Node());
        data_ptr->gznode_ptr->Init();

        // Create new vehicle
        data_ptr->vehicle_ptr = VehiclePtr(new Vehicle(model_ptr, sdf_ptr, data_ptr->nh, data_ptr->gznode_ptr));

        // Bind reference to physics update function
        data_ptr->connection_ptr = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPlugin::update, this));

        data_ptr->world_control_pub_ptr = data_ptr->gznode_ptr->Advertise<gazebo::msgs::WorldControl>("~/world_control");
        data_ptr->last_sim_time = data_ptr->world_ptr->SimTime();

        gzmsg << "MURSim: Vehicle Model plugin loaded!" << std::endl;
    }

    void ModelPlugin::Reset() // Overloaded
    {
        data_ptr->last_sim_time = 0;
    }

    void ModelPlugin::update()
    {
        std::lock_guard<std::mutex> lock(data_ptr->mutex);

        // PUBLISH()

        gazebo::common::Time current_time = data_ptr->world_ptr->SimTime();
        double dt = (current_time - data_ptr->last_sim_time).Double();

        // CHECK IF LOOP TIME

        data_ptr->last_sim_time = current_time;
        data_ptr->vehicle_ptr->update(dt);
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelPlugin)
}
