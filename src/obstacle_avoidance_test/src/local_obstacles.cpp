/*
 * local_obstacles.cpp
 * Author: Ningxuan Wang
 *
 *
 * Subscribe to laser scan messages
 * When one is received:
 * 	locate laser scan points within a certain range
 *	convert to x,y indice in a grid corresponding to the /map frame
 *	propagate inflation
 *	generate occupancy grid
 *	send out obstacle, inflation, occupancy grid messages
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include <string>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/observation_buffer.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GridCells.h>
#include <vector>
using namespace std;


#define SIZE 550 
//500 
#define RESOLUTION 0.05
//0.1
#define INFLATION_RANGE 4.0
#define LASER_RANGE 1.0
//0.80
#define WINDOW_SIZE 100
//150
//200


// publishers
ros::Publisher obstaclemap_pub;
ros::Publisher inflationmap_pub;
ros::Publisher occupancygrid_pub;
// transform listener
tf::TransformListener* tran;

// obstacle local map
bool obstaclemap[SIZE][SIZE];
//inflation local map
double inflationmap[SIZE][SIZE];
//tf_ prefix
 std::string odom_prefix;
 std::string laser_prefix;
 std::string frame_id;


/*
 * When initial pose is changed, recreate the local map
 */
void initCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  // empty the local map
  for (int i = 0; i < SIZE; i++) {
	for (int j = 0; j < SIZE; j++) {
		obstaclemap[i][j] = 0;
		inflationmap[i][j] = -1.0;
	}
  }

  // wait for 0.1 second
  ros::Rate rate(10.0);
  rate.sleep();

  // empty the local map to compete with the laser scan updating rate
  for (int i = 0; i < SIZE; i++) {
	for (int j = 0; j < SIZE; j++) {
		obstaclemap[i][j] = 0;
		inflationmap[i][j] = -1.0;
	}
  }
}

/*
 * When laser scan data comes in, create obstacles and inflation
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {

  // get coordinates of the origin of /base_laser_link in the /map frame
  geometry_msgs::PointStamped bl;
  //bl.header.frame_id = "/robot01/base_laser_link";
  bl.header.frame_id =laser_prefix;
  //bl.header.frame_id = "/base_laser_link";
  bl.header.stamp = ros::Time(0);
  bl.point.x = 0.0;
  bl.point.y = 0.0;
  bl.point.z = 0.0;
  geometry_msgs::PointStamped robot;

  try {
   	tran->transformPoint(odom_prefix, bl, robot);
   	//	tran->transformPoint("/map", bl, robot);
  } catch (tf::TransformException& ex) {
  }

  double x_o = robot.point.x;
  double y_o = robot.point.y;

  // calculate where the robot is in a grid corresponding to the /map frame

  int x_o_index = (int)((x_o)/ RESOLUTION );
  int y_o_index = (int)((y_o )/ RESOLUTION );
  
  // create a grid cell message for obstacles
  nav_msgs::GridCells gco;
  gco.header.frame_id = "/map";
  gco.header.stamp = ros::Time(0);
  gco.cell_width = RESOLUTION;
  gco.cell_height = RESOLUTION;

  // keeps the most recent valid laser points
  vector<int> laser_x;
  vector<int> laser_y;

  // number of obstacles cells contained in the obstacle grid cell message
  int obstacle_counter = 0;

  //initialize closest scan with first scan
  double closest = scan_in->ranges[0]+100;
  int index = 0;
  double closest_angle, best_x = 0, best_y = 0;

  //loop through all scans
  //for (int i = 0; i < scan_in->ranges.size(); i++) { // choix de 100 degrés à l'avant 

  int nb = (scan_in->angle_max - scan_in->angle_min) /scan_in->angle_increment;
  std::cout<<"nb points dans scan"<<"  "<<nb<<std::endl;
  for (int i = 100; i <225; i++) {
  //for (int i = 135; i <225; i++) {
    double range = scan_in->ranges[i];
   // std::cout<<"scan range"<<"  "<<scan_in->ranges[i]<<std::endl;
   // std::cout<<"angle min"<<"  "<<scan_in->angle_min<<std::endl;
    //std::cout<<"angle max"<<"  "<<scan_in->angle_max<<std::endl;
    // std::cout<<"increment"<<"  "<<scan_in->angle_increment<<std::endl;
    //find the point in base_laser_link frame corresponding to the scan
    closest_angle = ((double)i*scan_in->angle_increment)+scan_in->angle_min;
    std::cout<<"angle le plus proche"<<"  "<<closest_angle<<std::endl;
    double x = range * cos (closest_angle);
    double y = range * sin (closest_angle);

    // get coordinates of the laser scan in the /map frame
    geometry_msgs::PointStamped bll;
    //bll.header.frame_id = "/robot01/base_laser_link";
    bll.header.frame_id =laser_prefix;
    //bl.header.frame_id = "/base_laser_link";
    bll.header.stamp = ros::Time(0);
    bll.point.x = x;
    bll.point.y = y;
    bll.point.z = 0.0;
    geometry_msgs::PointStamped m;

    try {
   	  tran->transformPoint("/map", bll, m);
    } catch (tf::TransformException& ex) {
    }

    double x_l = m.point.x;
    double y_l = m.point.y;

    
    int x_index = (int)((x_l  )/ RESOLUTION );
    int y_index = (int)((y_l )/ RESOLUTION );

    if(y_index<0) y_index=y_index+360; // Remove the seg fault about negative angles for the laser angle range. 
    if(x_index<0) x_index=x_index+360;
    /*
    std::cout<<"X-Y laser origin in map A"<<x_o<<"  "<<y_o<<std::endl;
      std::cout<<"X-Y laser origin in map"<<x_o_index<<"  "<<y_o_index<<std::endl;
      std::cout<<"X-Y laser A"<<"  "<<x_l<<"  "<<y_l<<std::endl;
      std::cout<<"X-Y laser"<<"  "<<x_index<<"  "<<y_index<<std::endl;
      std::cout<<"X-Y laser VAL ABS"<<"  "<<abs(x_index)<<"  "<<abs(y_index)<<std::endl;
      */
     // ray-tracing
     if (x_index == x_o_index) {
     	// special case
	if (y_index > y_o_index) {
		for (int m = y_o_index + 1; m <= y_index - 1; m++) {
			obstaclemap[x_index][m] = false;
		}
        }
	if (y_index < y_o_index) {
		for (int m = y_index + 1; m <= y_o_index - 1; m++) {
			obstaclemap[x_index][m] = false;
		}
        }
     } else {
      std::cout<<"ok1"<<std::endl;
	double slope = (double)(y_index - y_o_index) / (double)(x_index - x_o_index);
	if (x_index > x_o_index) {
	     	for (int m = x_o_index + 1; m < x_index - 1; m++) {
			obstaclemap[m][(int)((double)(m - x_o_index) * slope) + y_o_index] = false;
		}
	} else {
	  std::cout<<"ok2"<<std::endl;
	  int m_int=(x_o_index - 1)-(x_index + 1);
	  std::cout<<m_int<<std::endl;
		for (int m = 0; m <m_int; m++) {
			obstaclemap[m][(int)((double)(m - x_index) * slope) + y_index] = false;
		}
	}
     }
       std::cout<<"ok3"<<std::endl;
     //locate laser scan cells within a certain range
    // if ( (range < LASER_RANGE) && (range > 0.35)) {
    if ( (range!=7.0)) {
        //if (x_index >= 0 && x_index < SIZE && y_index >= 0 && x_index < SIZE) {
		//if (x_index >= 0 && x_index < SIZE && y_index >= 0 && y_index < SIZE) {
		if (x_index >= 0 && x_index < SIZE && y_index < SIZE) {
		    //std::cout<<"laser range"<<"  "<<range<<std::endl;
		    //std::cout<<"laser range define"<<"  "<<LASER_RANGE<<std::endl;
		    laser_x.push_back(x_index);
		    laser_y.push_back(y_index);
		      //std::cout<<"ok41"<<std::endl;
		}
			/*
		if (x_index >= 0 && x_index < SIZE && y_index < 0 && x_index < SIZE){
		//if (x_index < 0 && abs(x_index) < SIZE) {
		    //std::cout<<"laser range"<<"  "<<range<<std::endl;
		    std::cout<<"y_index"<<"  "<<y_index<<std::endl;
		    laser_x.push_back(x_index);
		    laser_y.push_back(y_index);
		      std::cout<<"ok42"<<std::endl;
		}
	
		if (y_index < 0 && abs(x_index) < SIZE) {
		    //std::cout<<"laser range"<<"  "<<range<<std::endl;
		    //std::cout<<"laser range define"<<"  "<<LASER_RANGE<<std::endl;
		    laser_x.push_back(x_index);
		    laser_y.push_back(y_index);
		      std::cout<<"ok43"<<std::endl;
		}
		*/
	}
  }

  // mark obstacle cells in the local map
  while (!laser_x.empty())
  {
    std::cout<<"Mark"<<std::endl;
	obstaclemap[laser_x.back()][laser_y.back()] = true;
        laser_x.pop_back();
	laser_y.pop_back();
  }
  std::cout<<"ok5"<<std::endl;
  // find obstacle cells to be put in the obstacle grid cell message
  vector<geometry_msgs::Point> obstaclerepo;
    std::cout<<"X-Y ZERO"<<"  "<<x_o_index<<"  "<<y_o_index<<"  "<<std::endl;
    for (int i = x_o_index - WINDOW_SIZE / 2; i < x_o_index + WINDOW_SIZE / 2; i++) {
    for (int j = y_o_index - WINDOW_SIZE / 2; j < y_o_index + WINDOW_SIZE / 2; j++) {
    //for (int j = -y_o_index - WINDOW_SIZE / 2; j < y_o_index + WINDOW_SIZE / 2; j++) {
		if (i >= 0 && i < SIZE && j >= 0 && j < SIZE) {
		//if (i >= 0 && i < SIZE && j < SIZE) {
			if (obstaclemap[i][j]) {
				geometry_msgs::Point a;
				a.x = i * RESOLUTION;
				a.y = j * RESOLUTION;
				a.z = 0.0;
				obstaclerepo.push_back(a);
				std::cout<<"Put "<<std::endl;
				std::cout<<"Obstacle x - y "<<" "<<a.x <<" "<<a.y<<std::endl;
	
			}
		}
	}
  }

  // put obstacle cells in the obstacle grid cell message
  gco.cells.resize(obstaclerepo.size());

  while (!obstaclerepo.empty())
  {
    
    gco.cells[obstacle_counter++] = obstaclerepo.back();
    obstaclerepo.pop_back();
     //std::cout<<"yes  "<<" "<<obstacle_counter<<std::endl;

  }

  // inflation

  // create a grid cell message for inflation
  nav_msgs::GridCells gci;
  gci.header.frame_id = "/map";
  gci.header.stamp = ros::Time(0);
  gci.cell_width = RESOLUTION;
  gci.cell_height = RESOLUTION;

  vector<geometry_msgs::Point> repo;

  // number of obstacles cells contained in the obstacle grid cell message
  int cell_counter = 0;

  // propagate inflation
  for (int i = 0; i < SIZE; i++) {
	for (int j = 0; j < SIZE; j++) {
		inflationmap[i][j] = -1.0;
		if (!obstaclemap[i][j]) {
			double closestDistance = DBL_MAX;
			for (int m = i-INFLATION_RANGE; m < i+INFLATION_RANGE; m++) {
				for (int n = j-INFLATION_RANGE; n < j+INFLATION_RANGE; n++) {
					if (m >= 0 && m < SIZE && n >= 0 && n < SIZE && !(m == i && n == j)) {
						double distance = sqrt((double)((m - i) * (m - i) + (n - j) * (n - j)));
						if (distance <= INFLATION_RANGE) {
							if (obstaclemap[m][n]) {
								if (distance < closestDistance)
									closestDistance = distance;
							}
						}
					}
				}
			}
			if (closestDistance < DBL_MAX) {
				inflationmap[i][j] = closestDistance;
			}
		}
	}
  }
    std::cout<<"find inflation"<<std::endl;
  // find inflation cells in a rolling window centered at the robot
  for (int i = x_o_index - WINDOW_SIZE / 2; i < x_o_index + WINDOW_SIZE / 2; i++) {
	for (int j = y_o_index - WINDOW_SIZE / 2; j < y_o_index + WINDOW_SIZE / 2; j++) {
    //for (int j = -y_o_index - WINDOW_SIZE / 2; j < y_o_index + WINDOW_SIZE / 2; j++) {
   	    //if (i >= 0 && i < SIZE && j < SIZE) {
		if (i >= 0 && i < SIZE && j >= 0 && j < SIZE) {
			if (inflationmap[i][j] != -1 && !obstaclemap[i][j]) {
				geometry_msgs::Point a;
				a.x = i * RESOLUTION;
				a.y = j * RESOLUTION;
				a.z = 0.0;
				repo.push_back(a);
			}
		}
	}
  }

  // put inflation cells in the inflation grid cell message
  gci.cells.resize(repo.size());
  while (!repo.empty())
  { 
    gci.cells[cell_counter++]=repo.back();
    repo.pop_back();
 
  }

  // publish grid cell messages
  obstaclemap_pub.publish(gco);
  inflationmap_pub.publish(gci);

  // create a occupancy grid message for the entire local map
  nav_msgs::OccupancyGrid og;
  og.header.frame_id = "/map";
  og.header.stamp = ros::Time(0);
  og.info.resolution = RESOLUTION;
  og.info.width = SIZE;
  og.info.height = SIZE;
  og.data.resize(SIZE * SIZE);

  int og_counter = 0;

  // fill the occupancy grid
  for (int i = 0; i < SIZE; i++) {
	for (int j = 0; j < SIZE; j++) {
		if (obstaclemap[i][j]) {
			og.data[og_counter++] = 100;
		} else if (inflationmap[i][j] != -1.0) {
			og.data[og_counter++] = 100 - (int)(100.0 * inflationmap[i][j] / INFLATION_RANGE);
		} else {
			og.data[og_counter++] = 0;
		}

	}
  }
std::cout<<"nb obstacles"<<" "<<obstacle_counter<<std::endl;
std::cout<<"nb cells"<<" "<<cell_counter<<std::endl;
std::cout<<"nb OG"<<" "<<og_counter<<std::endl;
if( obstacle_counter==1 )  {
std::cout<<"pub occupancy grid"<<std::endl;
 // publish occupancy grid message
  occupancygrid_pub.publish(og);
//exit(0);
}
else  occupancygrid_pub.publish(og);
og_counter=0;
cell_counter=0;
obstacle_counter=0;



}

int main(int argc, char** argv){
  // initialize obstacle and inflation local map
  for (int i = 0; i < SIZE; i++) {
	for (int j = 0; j < SIZE; j++) {
		obstaclemap[i][j] = 0;
		inflationmap[i][j] = -1.0;
	}
  }

  // initialize the node
  ros::init(argc, argv, "local_obstacles");

  // listen to the transform message
  tf::TransformListener lr(ros::Duration(10));;
  tran = &lr;

  // initialize laser scan subscriber
  ros::NodeHandle m;
  if (m.getParam("tf_prefix3", frame_id))
    {   
        odom_prefix=frame_id+"/odom";
        laser_prefix=frame_id+"/base_laser_link";
        
    }
    else
        std::cout<<"Error getting frame ID"<<std::endl;
  //ros::Rate w(0.1);  
   //w.sleep();   
  ros::Subscriber laser_sub = m.subscribe<sensor_msgs::LaserScan>("/scan_filtered",5,scanCallback);
  std::cout<<"ap scan Callback"<<std::endl;
  // amcl pose scan subscriber
  ros::NodeHandle n;
  //ros::Subscriber init_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",5,initCallback);

  ros::Subscriber init_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose_bis",5,initCallback);
  std::cout<<"ap init Callback"<<std::endl;
  // initilize message publishers
  obstaclemap_pub = m.advertise<nav_msgs::GridCells>("obstaclemap", 1);
  inflationmap_pub = m.advertise<nav_msgs::GridCells>("inflationmap", 1);
  occupancygrid_pub = m.advertise<nav_msgs::OccupancyGrid>("occupancygrid", 1);
  
  ros::Rate r(0.2);
  //wait for things to happen
 ros::spin();
 r.sleep();

  return 0;
}


