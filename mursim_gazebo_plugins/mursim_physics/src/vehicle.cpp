#include "vehicle.hpp"

namespace mursim
{
    Vehicle::Vehicle(const gazebo::physics::ModelPtr &model, const sdf::ElementPtr &sdf, 
                     const std::shared_ptr<ros::NodeHandle> &node_handle, gazebo::transport::NodePtr &gznode)
                     : nh(node_handle), model_ptr(model), aero()
    {

    }
}