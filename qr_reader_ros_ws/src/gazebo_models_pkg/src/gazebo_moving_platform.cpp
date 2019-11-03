#include "gazebo_moving_platform.h"

using namespace std;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)

void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);

gazebo::ModelPush modelPush;

/////////////////////////////////////////////////
/// \brief ModelPush::ModelPush
///
ModelPush::ModelPush()
{
}

/////////////////////////////////////////////////
/// \brief RayPlugin::~RayPlugin
///
ModelPush::~ModelPush()
{
}

void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ModelPush::OnUpdate, this, _1));

        // Exit if no ROS
        if (!ros::isInitialized())
        {
            gzerr << "Not loading Optical Flow plugin since ROS hasn't been "
                  << "properly initialized.  Try starting gazebo with ros plugin:\n"
                  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }

        if (_sdf->HasElement("robotNamespace"))
          namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
          gzwarn << "[gazebo_moving_platform] Please specify a robotNamespace.\n";

        this->node_handle_ = new ros::NodeHandle(namespace_);

        this->moving_sub = this->node_handle_->subscribe("clock", 10, getSimulationClockTime);

        counter = 0;
    }

    // Called by the world update start event
void ModelPush::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Get Sim time
        common::Time  time = common::Time::GetWallTime();
        double sec = (modelPush.time_sec - modelPush.time_sec_prev) + (modelPush.time_nsec - modelPush.time_nsec_prev) / MILLION;
        //cout << sec << endl;

	// Periodic motion
        double A = 2;
        double T = 8 * M_PI;
        double w = 2 * M_PI / T;
        double vel_periodic = A*w*cos(w*sec); 
	
	// Circular motion
	T = 8.2*M_PI;
	w = 18 * M_PI / T;
	double Ampl_x=0.8;
	double Ampl_y=0.8;
	double vel_circular_x = -Ampl_x*sin(w*sec)*w;
	double vel_circular_y = Ampl_y*cos(w*sec)*w;
	double vel_periodic_x =  0.2*(2 * M_PI /(2 * M_PI) )*cos((2 * M_PI /(2 * M_PI) )*sec);
        
 //       if (counter < COUNTER_MAX){
   //       counter++; 
     //     modelPush.time_sec_prev = modelPush.time_sec;
       //   modelPush.time_nsec_prev = modelPush.time_nsec;      
       // }
        //else
       // {
            // Apply a small linear velocity to the model. CIRCULAR
            this->model->SetLinearVel(ignition::math::Vector3d(vel_circular_x, vel_circular_y, 0));	
            //this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));	
            
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.5));
 //this->model->SetAngularVel(math::Vector3(vel_periodic_x, vel_periodic_x, 0.2)); // IMAV 2017
							
					// PERIODIC
           // this->model->SetLinearVel(math::Vector3(0, vel_periodic, 0));	
           // this->model->SetAngularVel(math::Vector3(0, 0, 0.2));
						

       // }
        // Get world pose
        //       math::Pose pose = this->model->GetWorldPose();
        //       math::Vector3 pos = pose.pos;
        //       cout << "Moving Platform current pose" << endl;
        //       cout << "x: " << pos.x << "y: " << pos.y << "z: " << pos.z << endl;

        float norm_vel = sqrt((vel_circular_x*vel_circular_x)+(vel_circular_y*vel_circular_y));
        //cout << "norm_vel_circular: " << norm_vel << endl;
			  //cout << "perdiodic vel: " << vel_periodic << endl;



    }


void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg){

    ros::Time time = msg->clock;

    modelPush.time_sec = time.sec;
    modelPush.time_nsec = time.nsec;

    //cout << "time secs: " << modelPush.time_sec << endl;
}
