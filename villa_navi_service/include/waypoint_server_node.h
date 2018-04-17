#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <villa_navi_service/GoTargetName.h>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace std;

class waypoint_srv{

public:
	explicit waypoint_srv(ros::NodeHandle n_);
	~waypoint_srv();


	ros::Publisher setNavTarget_pub;
	ros::ServiceServer m_service;
	std::map< std::string, std::vector<double> > goal_maps;
		
	int targetup;
	tf::TransformListener 	  listener;

	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Head_Pos;				//x,y,theta
	std::vector<double> global_pose;

	void Publish_nav_target(float x_, float y_);
	bool goTarget(villa_navi_service::GoTargetName::Request &req, villa_navi_service::GoTargetName::Response &res);	
	void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations);
	void parseparameters(ros::NodeHandle n);

};