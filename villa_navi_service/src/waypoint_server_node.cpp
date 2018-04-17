#include "waypoint_server_node.h"


waypoint_srv::waypoint_srv(ros::NodeHandle n_){

  setNavTarget_pub=n_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  m_service = n_.advertiseService("/waypoint_go",  &waypoint_srv::goTarget,this);
  parseparameters(n_);

}
waypoint_srv::~waypoint_srv(){}


void waypoint_srv::Publish_nav_target(float _x, float _y)
{
		
		ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
	
		move_base_msgs::MoveBaseActionGoal Navmsgs;
		Navmsgs.header.stamp =  ros::Time::now();
		Navmsgs.goal.target_pose.header.frame_id = "map";

		Navmsgs.goal.target_pose.pose.position.x=_x;
		Navmsgs.goal.target_pose.pose.position.y=_y;
		Navmsgs.goal.target_pose.pose.position.z=0.0;

		Navmsgs.goal.target_pose.pose.orientation.x=0.0;
		Navmsgs.goal.target_pose.pose.orientation.y=0.0;
		Navmsgs.goal.target_pose.pose.orientation.z=0.0;
		Navmsgs.goal.target_pose.pose.orientation.w=1.0;

		setNavTarget_pub.publish(Navmsgs);
		ROS_INFO("navgation published");
}


void waypoint_srv::LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
  {

      XmlRpc::XmlRpcValue input_loc;
      std::string param_name = "villa_navi_service/" + input_locations;
      n.getParam(param_name, input_loc);
      std::vector<double> tmp_pos(3,0.0);
      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = input_loc.begin(); it != input_loc.end(); ++it) {
          ROS_INFO_STREAM("Loded "<< input_locations << "-" << (std::string)(it->first) << ",  " << input_loc[it->first]);

          tmp_pos[0]=static_cast<double>(input_loc["x"]);
          tmp_pos[1]=static_cast<double>(input_loc["y"]);
          tmp_pos[2]=static_cast<double>(input_loc["t"]);
      }

      goal_maps[input_locations]=tmp_pos;
  }

 void waypoint_srv::parseparameters(ros::NodeHandle n)
  {
      std::string target_frame;
      n.getParam("villa_navi_service/ref_frame", target_frame);
      //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_INFO_STREAM("reference frame: " << target_frame);

      LoadPositionfromConfig(n,"door");
      LoadPositionfromConfig(n,"goal");
      LoadPositionfromConfig(n,"living_room");
      LoadPositionfromConfig(n,"home_pos");
      LoadPositionfromConfig(n,"corridor");

      //print goalmaps
      std::map< std::string, std::vector<double> >::iterator goal_it = goal_maps.begin();
      for(goal_it ; goal_it!=goal_maps.end();goal_it++)
      {
          std::cout<<"goal: " <<  goal_it->first << ", positions: "
              <<(goal_it->second)[0]<<","<<(goal_it->second)[1]<<","<<(goal_it->second)[2]<<std::endl;
      }

  }


bool waypoint_srv::goTarget(villa_navi_service::GoTargetName::Request &req, villa_navi_service::GoTargetName::Response &res)
{


	std::string goal_loc = req.loc_name;
    std::cout<<"received goal locations: " << goal_loc << std::endl;
    auto search = goal_maps.find(goal_loc.c_str());

	if((search->second).size()>0)
	{
		float x_map=(search->second)[0];
        float y_map=(search->second)[1];
		Publish_nav_target(x_map,y_map);
		res.is_possible_go=true;
	}
	else
	{

		res.is_possible_go=false;
	}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "villa_waypoint_service");

  ros::NodeHandle n;
  waypoint_srv waypoint_manager(n);
    
  ros::Rate loop_rate(50);

   double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {
	 ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}








