#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <ignition/math/Vector3.hh>

#define MILLION 1E9
#define COUNTER_MAX 20000

using namespace std;

namespace gazebo
{
class ModelPush : public ModelPlugin
{

/// \brief Constructor
    public: ModelPush();

    /// \brief Destructor
    public: virtual ~ModelPush();

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/);

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
    
private:
    ros::NodeHandle *node_handle_;
    ros::Subscriber moving_sub;
    std::string namespace_;
public:
    double time_sec;
    double time_nsec;
    double time_sec_prev;
    double time_nsec_prev;
    int counter;
};
}
