#ifndef MURSIM_GAZEBO_CONNECTION_HPP
#define MURSIM_GAZEBO_CONNECTION_HPP

#include <ros/ros.h>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

namespace mursim
{
    class Connection
    {
    public:
        std::shared_ptr<ros::NodeHandle> nh;
        gazebo::physics::WorldPtr world_ptr;
        gazebo::physics::ModelPtr model_ptr;
        gazebo::transport::NodePtr gznode_ptr;
        gazebo::event::ConnectionPtr connection_ptr;
        gazebo::common::Time last_sim_time;
        std::mutex mutex;
        gazebo::transport::PublisherPtr world_control_pub_ptr;
    };
}
#endif // MURSIM_GAZEBO_CONNECTION_HPP