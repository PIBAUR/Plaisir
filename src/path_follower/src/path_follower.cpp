#include "path_follower.hpp"


PathFollower::PathFollower(ros::NodeHandle nh):
    nh_(nh),
    index_path_(0),
    size_path_(-1),
    du_(INIT_DU),
    dth_(PI),
    cpt_(0),
    linear_speed_(0.10),
    backward_(false),
    idle_(false),
    end_idle_time_(0.0)
{
    cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ratio_pub_ = nh_.advertise<scenario_msgs::PathFeedback>("path_feedback", 1);

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




void PathFollower::pathCB(const scenario_msgs::Path &msg)
{
	path_uid_ = msg.uid;
    path_ = msg.path;
    time_at_poses_ = msg.time_at_poses;
    index_sequence_ = 0;
    index_path_ = 0;
    size_path_ = path_.poses.size();


    if (size_path_ > 0)
	{
		ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = " << size_path_ << "  |  goal = "
						<< path_.poses.rbegin()->position.x << " ; " <<path_.poses.rbegin()->position.y
						<<"  |  elements in sequences = " << time_at_poses_.time_at_poses.size());
	}
    else
    {
		ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = " << size_path_ << "  |  goal = ");
    }
}




void PathFollower::computeCmd(double &lin, double &ang)
{
	tf::StampedTransform tf_robot;

    try
    {
        //TODO: replace "map" by path.header.frame_id
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
    double alpha;

    x_robot = tf_robot.getOrigin().x();
    y_robot =  tf_robot.getOrigin().y();
    theta_robot = tf::getYaw(tf_robot.getRotation());

    x_des =  path_.poses[index_path_].position.x;
    y_des =  path_.poses[index_path_].position.y;
    //theta_des = tf::getYaw(path.poses[index_path].orientation);

    dx = x_des - x_robot;
    dy = y_des - y_robot;
    if(du_ == INIT_DU)
        first_du_ = du_;
    du_=sqrt(dx*dx+dy*dy);

    alpha = atan2(dy,dx);
    ang = alpha-theta_robot;
    while(ang<-PI)
        ang+=2*PI;
    while(ang>=PI)
        ang-=2*PI;

    if(backward_)
    {
        if(ang<0)
            ang=PI-abs(ang);
        else
            ang=-(PI-abs(ang));
    }
    ROS_DEBUG_STREAM("du = "<< du_<< "  | ang " << ang);
    ang*=K_TH;
    lin=linear_speed_;

    if(ang>ANGULAR_SPEED_MAX)
    {
    	ang = ANGULAR_SPEED_MAX;
    	lin*=0.2;
    }
    else if(ang < (- ANGULAR_SPEED_MAX) )
    {
    	ang = -ANGULAR_SPEED_MAX;
    	lin*=0.2;
    }

    if(lin>LINEAR_SPEED_MAX)
		lin = LINEAR_SPEED_MAX;
	else if(lin < (- LINEAR_SPEED_MAX) )
		lin = -LINEAR_SPEED_MAX;

}





void PathFollower::computeLastPointAngleCmd(double &lin, double &ang)
{
    tf::StampedTransform tf_robot;

    try
    {
        //TODO: replace "map" by path.header.frame_id
        tf_listener_.lookupTransform("/map", "base_link", ros::Time(0), tf_robot);
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
    while(ang<-PI)
        dth_+=2*PI;
    while(ang>=PI)
        dth_-=2*PI;

    ang = dth_;
    ang*=K_TH;
    if(ang>ANGULAR_SPEED_MAX)
        ang = ANGULAR_SPEED_MAX;
    else if(ang < (- ANGULAR_SPEED_MAX) )
        ang = -ANGULAR_SPEED_MAX;

    lin=0.0;
}


void PathFollower::publishRatio()
{
	double ratio_to_next = (du_ - NEXT_POINT_DISTANCE_THRESH) / (first_du_ - NEXT_POINT_DISTANCE_THRESH);

	scenario_msgs::PathFeedback pathFeedback;
	pathFeedback.uid = path_uid_;
	pathFeedback.ratio = (1.0*index_path_ + (1-ratio_to_next))/size_path_;
	ratio_pub_.publish(pathFeedback);
}




float PathFollower::distanceToGoal(size_t index_goal)
{
    float distance = 0.0;
    for(size_t index = index_path_; index<index_goal; index++ )
    {
        distance += distanceBetweenPoints(index,index+1);
    }
    return distance;
}


float PathFollower::distanceBetweenPoints(size_t index1, size_t index2)
{
    float d, dx, dy;
    dx = path_.poses[index1].position.x - path_.poses[index2].position.x;
    dy = path_.poses[index1].position.y - path_.poses[index2].position.y;
    d = sqrt(dx*dx + dy*dy);
    return d;
}



void PathFollower::computeAverageSpeed(size_t index_goal, float time)
{
    if(time == 0.0)
    {
        ROS_ERROR_STREAM("RECIEVED TIME NUL FOR NEXT GOAL IN THE CURRENT SEQUENCE");
        linear_speed_ = 0.0;
        return;
    }
    float distance = distanceToGoal(index_goal);
    linear_speed_ = distance / abs(time) ;
    ROS_INFO_STREAM("Linear speed  = "<<linear_speed_<<"   for distance/time : "<<distance<<" / "<<time);
    if(backward_)
    {
        linear_speed_ *= -1;
        ROS_INFO("Going backward to next goal");
    }
    else
    {
        ROS_INFO("Going forward to next goal");
    }

    if(abs(linear_speed_) >= LINEAR_SPEED_MAX)
    {
        ROS_WARN_STREAM("Linear speed (asbolute) is too high : "<<linear_speed_<<" . Value set to max : "<<LINEAR_SPEED_MAX);
        linear_speed_ = LINEAR_SPEED_MAX;
    }
    else if(abs(linear_speed_) >= float(LINEAR_SPEED_MAX/2.0))
    {
        ROS_WARN_STREAM("Linear speed (asbolute) is high : "<<linear_speed_<<" . May not be able to turn");
        linear_speed_ = LINEAR_SPEED_MAX;
    }
    else
    {
        ROS_INFO_STREAM("Average linear speed for next goal : "<<linear_speed_);
    }
}


void PathFollower::initNextGoal()
{
    float delta_time;

    delta_time = abs(time_at_poses_.time_at_poses[index_sequence_+1].time) - abs(time_at_poses_.time_at_poses[index_sequence_].time);

    //if time < 0, go backward (true), else go forward (false)
    backward_ = time_at_poses_.time_at_poses[index_sequence_].backward;
    ROS_INFO_STREAM("Next goal : pose : "<< time_at_poses_.time_at_poses[index_sequence_+1].pose_index
                    <<"  duration : "<<delta_time<<" seconds.");

    computeAverageSpeed(time_at_poses_.time_at_poses[index_sequence_+1].pose_index, delta_time);
    if(time_at_poses_.time_at_poses[index_sequence_+1].pose_index==time_at_poses_.time_at_poses[index_sequence_].pose_index)
    {
        ROS_INFO_STREAM("Idle for "<<delta_time<<" seconds.");
        end_idle_time_ = ros::Time::now() + ros::Duration(delta_time);
        idle_=true;

    }
    index_sequence_++;
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
        if(idle_)
        {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            cmd_pub_.publish(cmd);

            if(end_idle_time_ <= ros::Time::now())
                idle_ = false;
        }
        else
        {
            if( index_sequence_+1 <time_at_poses_.time_at_poses.size()
                    && index_path_ >= time_at_poses_.time_at_poses[index_sequence_].pose_index)
                initNextGoal();
            else if(index_path_ == size_path_-1)
            {
                if(du_ > LAST_POINT_DISTANCE_THRESH)
                {
                    computeCmd(cmd.linear.x, cmd.angular.z);
                    cmd_pub_.publish(cmd);
                }
                else if(dth_ > LAST_POINT_ANGLE_THRESH)
                {
                    computeLastPointAngleCmd(cmd.linear.x, cmd.angular.z);
                    cmd_pub_.publish(cmd);
                }
                else
                {
                    index_path_++;
                    du_=INIT_DU;
                    dth_ = PI;
                    ROS_INFO_STREAM("Heading to point #"<<index_path_<<"/"<<size_path_<<" : ("
                        <<path_.poses[index_path_].position.x<<"|"<<path_.poses[index_path_].position.y<<")");
                }
            }
            else
            {
                if( du_ > NEXT_POINT_DISTANCE_THRESH)
                {
                    computeCmd(cmd.linear.x, cmd.angular.z);
                    cmd_pub_.publish(cmd);
                }
                else
                {
                    index_path_++;
                    du_=INIT_DU;
                    dth_ = PI;
                }
            }
        }
    }
    else if(size_path_ > 0 && index_path_>=size_path_)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_.publish(cmd);
        size_path_=-1;

		ROS_INFO_STREAM("Path follower ended");
    }
    else if(size_path_ ==0)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_.publish(cmd);
        size_path_=-1;

		ROS_INFO_STREAM("Path follower received 0 sized path");
    }

    // publish ratio
	if(cpt_>RATIO_PUBLISH_RATE_DIVIDOR)
	{
		publishRatio();
		cpt_=0;
	}
    cpt_++;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower_node");
	ros::NodeHandle nh;
	PathFollower pf(nh);
	ros::Subscriber path_sub = nh.subscribe("path", 1, &PathFollower::pathCB, &pf);
	ros::Rate loop(LOOP_RATE);

	while(ros::ok())
	{
		pf.spinOnce();
		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}
