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
