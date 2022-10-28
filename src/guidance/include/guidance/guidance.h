#ifndef FORMATION_GUIDANCE
#define FORMATION_GUIDANCE

#include <Eigen/Geometry>
#include <mavros_msgs/ParamSet.h>
#include <uav_status.h>
#include <cmath>

typedef struct command
{
    Eigen::Vector3d velocity;
    float r;
} command;

class guidance{
	private:
		bool callback_gainUpdate(mavros_msgs::ParamSet::Request &req, mavros_msgs::ParamSet::Response &res);
	
	public:
		ros::ServiceServer service;
		uav_status* uav;
		double_t gain[40];
		
	public:
		guidance(ros::NodeHandle& nh,uav_status* target_uav);
		command	gen_command(Eigen::Vector3d point1);
		command	gen_command_goal(Eigen::Vector3d point1);
		Eigen::Vector3d lla2meter(Eigen::Vector3d point1, Eigen::Vector3d point2);
};

double sign(double data);
struct command saturation(struct command cmd, double max);

#endif
