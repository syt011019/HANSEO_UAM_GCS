#include <guidance.h>
#include <ros/ros.h>
#include <iostream>
#include <math.h>

double myAbs(double val)
{
	if(val > 0)
	{
		return val;
	}
	else
	{
		return -val;
	}
}

guidance::guidance(ros::NodeHandle& nh, uav_status* target_uav)
{
	this->gain[0]=0.2;
	this->gain[1]=0.2;
	this->gain[2]=0.3;	// landing flag
	this->gain[3]=0.8;

	this->gain[13] = 0.7;			// CA LPF alpha
	this->gain[14] = 0.7;			// Landing LPF alpha
	this->gain[15] = 36.5915565;		// I_pos latitude
	this->gain[16] = 126.2951202;	// I_pos longitude
	this->gain[17] = -15.0;			// Altitude command in CA_mode
	this->gain[18] = 10.0;			// Saftey range
	this->gain[19] = 0.125;			// Z-axis gain in CA_mode
	this->gain[20] = 1.0;			// guidance mode - mode 1 : landing, mode 2 : CA
	this->gain[21] = -120.0;
	this->uav = target_uav;
	
	std::string srv_name("/guidance/gain");
	srv_name = this->uav->name + srv_name; 
	
	std::cout<<srv_name.c_str()<<std::endl;
	this->service =  nh.advertiseService(srv_name.c_str(), &guidance::callback_gainUpdate, this);
}

bool guidance::callback_gainUpdate(mavros_msgs::ParamSet::Request &req, mavros_msgs::ParamSet::Response &res)
{
	this->gain[req.value.integer] = req.value.real;
	res.value.real = req.value.real;
	res.value.integer = req.value.integer;
	res.success = true;
	
	return true;
}

command	guidance::gen_command(Eigen::Vector3d point1)
{
		
	struct command cmd;

	Eigen::Vector3d target = lla2meter(this->uav->global, point1);


	float psi = this->uav->attitude_ned[2];
	float psi_cmd = this->gain[21]*3.141592/180;	
	float vx_cmd = this->gain[0]*target[0];
	float vy_cmd = this->gain[1]*target[1];
	float r_cmd = this->gain[3]*(0);
	float vz_cmd = 0.0;
	
	float x = target[0];
	float y = target[1];
	float z = target[2];

	vz_cmd = this->gain[2];
/*


	if(myAbs(this->uav->global[2]) < 3.0)
	{	
		this->gain[0]=0.25;
		this->gain[1]=0.25;
		if(sqrt(x*x + y*y) < 0.3)
		{
			vz_cmd = this->gain[2]*0.5;
		}
		else{
			vz_cmd = 0.0;
		}
	}
	else{
		vz_cmd = this->gain[2];
	}
*/
/*	if(myAbs(z) < 4.0)
	{
		vz_cmd = this->gain[2]*0.2;
	}
*/
	if((psi_cmd - psi) > (3.141592))
	{
		psi_cmd = psi_cmd - 2*3.141592;
	}

	else if((psi_cmd - psi) < (-3.141592))
	{
		psi_cmd = psi_cmd + 2*3.141592;
	}

	//r_cmd = this->gain[3]*(psi_cmd - psi);
	r_cmd = this->gain[3]*(0);
	////////////////////////////////////////////////////////////
	///////////////////////OUTPUT DATA//////////////////////////
	////////////////////////////////////////////////////////////

	cmd.velocity[0] = vx_cmd;
	cmd.velocity[1] = vy_cmd;
	cmd.velocity[2] = vz_cmd; 
	cmd.r = r_cmd;

	cmd = saturation(cmd, 3);
	
	return cmd;	
}

command	guidance::gen_command_goal(Eigen::Vector3d point1)
{
		
	struct command cmd;

	Eigen::Vector3d target = lla2meter(this->uav->global, point1);

	float psi = this->uav->attitude_ned[2];
	
	float vx_cmd = this->gain[0]*target[0];
	float vy_cmd = this->gain[1]*target[1];
	float r_cmd = this->gain[3]*(0);
	float vz_cmd = this->gain[19]*target[2];
	
	cmd.velocity[0] = vx_cmd;
	cmd.velocity[1] = vy_cmd;
	cmd.velocity[2] = vz_cmd; 
	cmd.r = r_cmd;

	cmd = saturation(cmd, 1.5);
	
	return cmd;	
}

double sign(double data)
{
    if(data > 0)
    {
        return 1.0;
    }

    if(data < 0)
    {
        return -1.0;
    }

}

struct command saturation(struct command cmd, double max)
{

	struct command cmdOut= cmd;
	
	double pi = 3.141592;
	double magnitude_cmd = sqrt(cmdOut.velocity[0]*cmdOut.velocity[0] + cmdOut.velocity[1]*cmdOut.velocity[1]);
	
	if(magnitude_cmd > max)
	{
		cmdOut.velocity[0] = max*cmdOut.velocity[0]/magnitude_cmd;
		cmdOut.velocity[1] = max*cmdOut.velocity[1]/magnitude_cmd;
	}

//	if(cmdOut.velocity[2] >1)
//		cmdOut.velocity[2]=1;
//	else if(cmdOut.velocity[2]<-1)
//		cmdOut.velocity[2]=-1;

	cmdOut.velocity[2]= cmd.velocity[2];

	if(cmdOut.r > pi/4)
		cmdOut.r = pi/4;
	else if(cmdOut.r < -pi/4)
		cmdOut.r = -pi/4;
		
	return cmdOut;
	
}


Eigen::Vector3d guidance::lla2meter(Eigen::Vector3d point1, Eigen::Vector3d point2)
{
	double utm_lat2m_deg = 110961.7060516;
	double utm_lon2m_deg = 89476.51124;

	Eigen::Vector3d distance;

	Eigen::Vector3d diff_points = point2 - point1;

	distance[0] = diff_points[0]*utm_lat2m_deg;
	distance[1] = diff_points[1]*utm_lon2m_deg;
	distance[2] = diff_points[2];

	return distance;
}

