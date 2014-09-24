///*******************************///
///*********INCLUDES**********///
///*******************************///
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <ros/ros.h>
#include <map>

///*******************************///
///*********DEFINES**********///
///*******************************///
#define HIGH 1
#define LOW  0



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
	std::fstream file_value_;	//fichier contenant valeur gpio
	std::ofstream file_direction_;	//fichier contenant valeur gpio
	int level_;			//valeur de gpio
	bool mode_;	//sens : input ou output
	int  pin_;

public:
	///-----------CONSTRUTORS--------------///
	MyGPIO(int pin);
	~MyGPIO();

	///-----------METHODES--------------///
	void set_input_mode(){ file_direction_<<"in"; mode_ = false; };
	void set_output_mode(){ file_direction_<<"out"; mode_ = true; };
	void set_level(int level){ if(mode_) file_value_<< level; };
	bool get_level();
	bool get_mode() {return mode_;};
};


MyGPIO::MyGPIO(int pin)
{
	std::stringstream filename;
	pin_ = pin;
	int a = GPIO_Map[pin_];
	//setting file name to access value
	filename<<"/sys/class/gpio/gpio"<<"/";
	file_value_.open(filename<<"value");
	file_direction_.open(filename<<"direction");
	
	ROS_ERROR_STREAM_COND_NAMED(!(file_value_.is_open() && file_value_.is_open()),
			"Bumper","Impossible to open files for pin"<< pin_);
	set_input_mode();
}

MyGPIO::~MyGPIO()
{
	file_value_.is_close();
	file_direction_.is_close();
	ROS_ERROR_STREAM_COND_NAMED(!(file_value_.is_close() && file_value_.is_close()),
			"Bumper","Impossible to close files for pin"<< pin_);
}


bool MyGPIO::get_level()
{
	char c;
	file_value_.seekg (0, file_value_.beg);
	file_value_.read(&c,1) ;
	level_ = 48 - (int)c;
	return level_
}


int main () {
	
	return 0;
}
