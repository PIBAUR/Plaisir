#include "path_follower.hpp"
#include <cmath>

PathFollower::PathFollower(ros::NodeHandle nh):
    nh_(nh),
    index_path_(0),
    size_path_(-1),
    du_(INIT_DU),
    dth_(PI),
    cpt_(0)
{
    cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    wanted_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("wanted_cmd_vel", 1);
    ratio_pub_ = nh_.advertise<scenario_msgs::PathFeedback>("path_feedback", 1);

    /*** get rosparam ***/
	nh_.param<double>("/linear_speed_default", linear_speed_default_, LINEAR_SPEED_DEFAULT);
	nh_.param<double>("/angular_speed_max", angular_speed_max_, ANGULAR_SPEED_MAX);
	nh_.param<double>("/angle_thresh_low", angle_thresh_low_, ANGLE_THRESH_LOW);
	nh_.param<double>("/angle_thresh_high", angle_thresh_high_, ANGLE_THRESH_HIGH);
	nh_.param<double>("/next_point_distance_thresh", next_point_distance_thresh_, NEXT_POINT_DISTANCE_THRESH);
	nh_.param<double>("/last_point_distance_thresh", last_point_distance_thresh_, LAST_POINT_DISTANCE_THRESH);
	nh_.param<double>("/last_point_angle_thresh", last_point_angle_thresh_, LAST_POINT_ANGLE_THRESH);
	nh_.param<double>("/k_th", k_th_, K_TH);
	ROS_INFO_STREAM("Init path_follower with param :\n"
					<<"\t linear_speed_default = "<<linear_speed_default_<<"\n"
					<<"\t angular_speed_max = "<<angular_speed_max_<<"\n"
					<<"\t angle_thresh_low = "<<angle_thresh_low_<<"\n"
					<<"\t angle_thresh_high = "<<angle_thresh_high_<<"\n"
					<<"\t next_point_distance_thresh = "<<next_point_distance_thresh_<<"\n"
					<<"\t last_point_distance_thresh = "<<last_point_distance_thresh_<<"\n"
					<<"\t last_point_angle_thresh = "<<last_point_angle_thresh_
					<<"\t k_th = "<<k_th_);

    std::string tf_prefix;
    std::stringstream frame;

    if(nh_.getParam("tf_prefix", tf_prefix))
    {
        frame << tf_prefix <<"/base_link";
    }
    else
    {
        frame<<"/base_link";
    }
    robot_frame_ = frame.str();
    ROS_INFO_STREAM("frame robot in path follower : "<<frame.str());
}




void PathFollower::pathCB(const scenario_msgs::PathPosition &msg)
{
	path_uid_ = msg.uid;
    path_ = msg.path;
    index_path_ = 0;
    size_path_ = path_.poses.size();

    if (size_path_ > 0)
		ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = " << size_path_ << "  |  goal = "
						<< path_.poses.rbegin()->position.x << " ; " <<path_.poses.rbegin()->position.y);
    else
		ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = empty.");
}


void PathFollower::freezePathCB(const std_msgs::Bool &msg) {
	path_follower_frozen = msg.data;
}


void PathFollower::computeCmd(double &lin, double &ang)
{
	tf::StampedTransform tf_robot;

    try
    {
        tf_listener_.lookupTransform("/map", robot_frame_, ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    double x_robot, y_robot, theta_robot;
    double dx, dy;
    double x_des, y_des;
    double alpha, dth;

    x_robot = tf_robot.getOrigin().x();
    y_robot =  tf_robot.getOrigin().y();
    theta_robot = tf::getYaw(tf_robot.getRotation());

    x_des =  path_.poses[index_path_].position.x;
    y_des =  path_.poses[index_path_].position.y;

    dx = x_des - x_robot;
    dy = y_des - y_robot;

    if(du_ == INIT_DU)
        first_du_ = du_;
    du_=sqrt(dx*dx+dy*dy);
    alpha = atan2(dy,dx);
    dth = alpha-theta_robot;

    ROS_DEBUG_STREAM("du = "<< du_<< "  | dth " << dth << " | index_path_ " << index_path_);

    while(dth<-PI)
    	dth+=2*PI;
	while(dth>=PI)
		dth-=2*PI;

	ang=dth*k_th_;
	lin=linear_speed_default_;



	if(fabs(dth) > angle_thresh_high_)
		lin *= 0.00;
	else if(fabs(dth) > angle_thresh_low_)
		lin *= 0.2;

	if(ang>angular_speed_max_)
		ang = angular_speed_max_;
	else if(ang < -angular_speed_max_)
		ang = -angular_speed_max_;

	ROS_DEBUG_STREAM("du = "<< du_<< "  | dth " << dth<< "  | alpha " << alpha
						<< "  | index_path_ " << index_path_ << "  | ang " << ang);
}



void PathFollower::computeLastPointAngleCmd(double &lin, double &ang)
{
    tf::StampedTransform tf_robot;

    try
    {
    	tf_listener_.lookupTransform(path_.header.frame_id, robot_frame_, ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    double theta_robot;
    double theta_des;

    theta_robot = tf::getYaw(tf_robot.getRotation());
    theta_des = tf::getYaw(path_.poses[index_path_].orientation);
    dth_ = theta_des-theta_robot;

    while(dth_<-PI)
        dth_+=2*PI;
    while(dth_>=PI)
        dth_-=2*PI;

    ang = dth_ / 2.0;
    if(ang > angular_speed_max_)
        ang = angular_speed_max_;
    else if(ang < -angular_speed_max_)
        ang = -angular_speed_max_;

    lin=0.0;
}


void PathFollower::publishRatio()
{
	double ratio_to_next = (du_ - next_point_distance_thresh_) / (first_du_ - next_point_distance_thresh_);

	scenario_msgs::PathFeedback pathFeedback;
	pathFeedback.uid = path_uid_;
	pathFeedback.ratio = (1.0*index_path_ + (1-ratio_to_next))/size_path_;
	ratio_pub_.publish(pathFeedback);
}


void PathFollower::spinOnce()
{
	geometry_msgs::Twist cmd;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;

    if(size_path_ > 0 && index_path_<size_path_)
	{
		if(index_path_ == size_path_-1)
		{
			if(du_ > last_point_distance_thresh_)
				computeCmd(cmd.linear.x, cmd.angular.z);
			else if(dth_ > last_point_angle_thresh_)
				computeLastPointAngleCmd(cmd.linear.x, cmd.angular.z);
			if (! path_follower_frozen)
				cmd_pub_.publish(cmd);
			wanted_cmd_pub_.publish(cmd);
		}
		else if( du_ > next_point_distance_thresh_)
		{
			computeCmd(cmd.linear.x, cmd.angular.z);
			if (! path_follower_frozen)
				cmd_pub_.publish(cmd);
			wanted_cmd_pub_.publish(cmd);
		}
		else
		{
			ROS_INFO_STREAM("Heading to point #"<<index_path_<<"/"<<size_path_<<" : ("
				<<path_.poses[index_path_].position.x<<"|"<<path_.poses[index_path_].position.y<<")");
			index_path_++;
			du_=INIT_DU;
			dth_ = PI;
		}
		// publish ratio
		if(cpt_ > RATIO_PUBLISH_RATE_DIVIDOR)
		{
			publishRatio();
			cpt_=0;
		}
		cpt_++;
	}
	else if(size_path_ > 0 && index_path_>=size_path_)
	{
		cmd.linear.x = 0;
		cmd.angular.z = 0;
		if (! path_follower_frozen)
			cmd_pub_.publish(cmd);
		wanted_cmd_pub_.publish(cmd);
		publishRatio();
		size_path_ = -1;
		ROS_INFO_STREAM("Path follower ended");
	}
	else if(size_path_ == 0)
	{
		cmd.linear.x = 0;
		cmd.angular.z = 0;
		if (! path_follower_frozen)
			cmd_pub_.publish(cmd);
		wanted_cmd_pub_.publish(cmd);
		size_path_=-1;
		ROS_INFO_STREAM("Path follower received 0 sized path");
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower_node");
	ros::NodeHandle nh;
	PathFollower pf(nh);
	ros::Subscriber path_sub = nh.subscribe("path_travel", 1, &PathFollower::pathCB, &pf);
	ros::Subscriber freeze_path_follower_sub = nh.subscribe("freeze_path_follower", 1, &PathFollower::freezePathCB, &pf);
	ros::Rate loop(LOOP_RATE);

	while(ros::ok())
	{
		pf.spinOnce();
		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}
