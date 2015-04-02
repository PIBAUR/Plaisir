///*******************************///
///*********INCLUDES**********///
///*******************************///
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>

#include <ros/ros.h>
#include <map>
#include <vector>
#include <std_msgs/Bool.h>

///*******************************///
///*********DEFINES**********///
///*******************************///
#define HIGH 1
#define LOW  0
#define LOOP_RATE 20


///*******************************///
///*********MAP**********///
///*******************************///
/// key = pin number ; element =  gpio number in system
struct GPIO_TABLE{
    static std::map<int,int> create_map()
	{
		std::map<int,int> m;
		m[13]=309;
		m[14]=316; // none working...?
		m[15]=306;
		m[16]=304;
		m[17]=310;
		m[18]=307;
		m[19]=319;
		m[20]=317;
		m[21]=318;
		m[22]=320;
		m[23]=315;
		m[24]=314;
		m[25]=311; // none working...?
		m[26]=313;
		m[27]=323;// none working...?
		return m;
	}
    static const std::map<int,int> GPIO_Map;

};

const std::map<int,int> GPIO_TABLE::GPIO_Map =  GPIO_TABLE::create_map();





///*******************************///
///*********CLASS MyGPIO**********///
///*******************************///
class MyGPIO
{
private:
	///----------ATTRIBUTES---------------///
	std::fstream file_value_;	    //fichier contenant valeur gpio
	std::ofstream file_direction_;	//fichier contenant valeur gpio
	bool level_;			        //valeur de gpio
	bool mode_;	                    //sens : input ou output
	unsigned int  pin_;            //numero pin

public:
	///-----------CONSTRUTORS--------------///
	MyGPIO(const unsigned int &pin);
	MyGPIO(const MyGPIO &gpio);
	~MyGPIO();

	///-----------METHODES--------------///
	void set_input_mode(){ file_direction_<<"in"; mode_ = false; };
	void set_output_mode(){ file_direction_<<"out"; mode_ = true; };
	void set_level(int level){ if(mode_) file_value_<< level; };
	bool get_level();
	bool get_mode() const {return mode_;};
	unsigned int get_pin() const {return pin_;};
	MyGPIO operator =(const MyGPIO &gpio);
	std::string to_string();
};


MyGPIO::MyGPIO(const unsigned int &pin):
        pin_(pin)
{
	std::stringstream filename;
	std::stringstream filenamevalue, filenamedirection;
	std::map<int,int> map = GPIO_TABLE::GPIO_Map;
	//setting file name to access value
	filenamevalue<<"/sys/class/gpio/gpio"<<map[pin_]<<"/value";
	filenamedirection<<"/sys/class/gpio/gpio"<<map[pin_]<<"/direction";
	file_value_.open(filenamevalue.str().c_str());
	file_direction_.open(filenamedirection.str().c_str());

	ROS_ERROR_STREAM_COND_NAMED(!(file_value_.is_open() && file_value_.is_open()),
			"Bumper","Impossible to open files for pin"<< pin_);
	set_input_mode();
	ROS_INFO_STREAM(""<<to_string());
}

MyGPIO::MyGPIO(const MyGPIO &gpio)
{
    this->pin_ = gpio.get_pin();
    std::stringstream filename;
    std::stringstream filenamevalue, filenamedirection;
    std::map<int,int> map = GPIO_TABLE::GPIO_Map;
    //setting file name to access value
    filenamevalue<<"/sys/class/gpio/gpio"<<map[pin_]<<"/value";
    filenamedirection<<"/sys/class/gpio/gpio"<<map[pin_]<<"/direction";
    this->file_value_.open(filenamevalue.str().c_str());
    this->file_direction_.open(filenamedirection.str().c_str());

    ROS_ERROR_STREAM_COND_NAMED(!(file_value_.is_open() && file_value_.is_open()),
            "Bumper","Impossible to open files for pin"<< pin_);
    this->set_input_mode();
}

MyGPIO::~MyGPIO()
{
	file_value_.close();
	file_direction_.close();
	ROS_ERROR_STREAM_COND_NAMED((file_value_.is_open() || file_value_.is_open()),
			"Bumper","Impossible to close files for pin"<< pin_);
}


 bool MyGPIO::get_level()
{
	char c;
	file_value_.seekg (0, file_value_.beg);
	file_value_.read(&c,1) ;
	level_ = (c == '1');
	return level_;
}


MyGPIO MyGPIO::operator =(const MyGPIO &gpio)
{
    MyGPIO g(gpio);
    return g;
}

std::string MyGPIO::to_string()
{
    std::map<int,int> map = GPIO_TABLE::GPIO_Map;
    std::stringstream ss;
    ss << "GPIO #" << pin_ << "(file = gpio : " << map[pin_] << ") : " <<"direction = ";
    if(mode_)
        ss << "OUTPUT ";
    else
        ss << "INPUT  ";
    ss << "| value = ";
    if(level_)
        ss << "true ";
    else
        ss << "false  ";
    return ss.str() ;

}

///*******************************///
///*********CLASS BUMPERS**********///
///*******************************///

class Bumpers
{
protected :
    ros::NodeHandle nh_;
    ros::Rate loop_;
    size_t nb_bumpers_;
    std::vector<MyGPIO> gpios_;
    ros::Publisher bumpers_pub_;
    std::vector<bool> bumpers_state_;

public:
    Bumpers();
    Bumpers(const std::vector<unsigned int> &pins);
    ~Bumpers(){};

    void add_gpio(const unsigned int &pin);
    void read_state();
    void publish_state();
    void spin();
};



Bumpers::Bumpers() :
        nb_bumpers_(0),
        loop_(LOOP_RATE)
{
    bumpers_pub_ = nh_.advertise<std_msgs::Bool>("bumpers",1);
}



Bumpers::Bumpers(const std::vector<unsigned int> &pins) :
        nb_bumpers_(pins.size()),
        loop_(LOOP_RATE)
{
    bumpers_pub_ = nh_.advertise<std_msgs::Bool>("bumpers",1);

    for(std::vector<unsigned int>::const_iterator it_pin = pins.begin(); it_pin != pins.end(); it_pin++)
    {
        add_gpio(*it_pin);
    }

    read_state();
}



void Bumpers::add_gpio(const unsigned int &pin)
{
    MyGPIO gpio(pin);
    gpios_.push_back(gpio);
}



void Bumpers::read_state()
{
    bumpers_state_.clear();
    for(std::vector<MyGPIO>::iterator it_gpio = gpios_.begin(); it_gpio != gpios_.end(); it_gpio++)
    {
        bumpers_state_.push_back(it_gpio->get_level());
        ROS_DEBUG_STREAM(""<<it_gpio->to_string());
    }
}



void Bumpers::publish_state()
{
    std_msgs::Bool msg;
    msg.data = false;
    for(std::vector<bool>::iterator it_state = bumpers_state_.begin(); it_state != bumpers_state_.end(); it_state++)
    {
        msg.data = msg.data || (*it_state);
    }
    bumpers_pub_.publish(msg);
}



void Bumpers::spin()
{
    while(ros::ok())
    {
        read_state();
        publish_state();
        ros::spinOnce();
        loop_.sleep();
    }
}


int main (int argc, char** argv)
{
    ros::init(argc,argv,"bumper");
    ros::NodeHandle nh;

    std::vector<unsigned int> pins;
    pins.push_back(26);
    pins.push_back(24);
    pins.push_back(22);
    pins.push_back(20);
    Bumpers bumpers_sensor(pins);
    bumpers_sensor.spin();

	return 0;
}
