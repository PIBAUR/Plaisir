/********************************************************************
 * @filename    MD25.h
 * @author      Thomas B. Joergensen (thomas.bal.jorgensen@gmail.com)
 * @date        06 FEB 2012
 * @version     1.0
 * @target      mbed (NXP LPC1768 - ARM Cortex M3 - 32-bit)
 *
 * @desciption  A library for interacting with the MD25 DC motor
 *              controller. Find more at:
 *              http://www.robot-electronics.co.uk/htm/md25tech.htm
 *              http://www.robot-electronics.co.uk/htm/md25i2c.htm
 *******************************************************************/


/* INCLUDES */
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


/* COMMANDS FLAGS */
#define SYNC_BYTE               0x00    // Add this before any command
#define CMD_GET_SPEED1          0x21    // R    | Get the current speed of motor 1
#define CMD_GET_SPEED2          0x22    // R    | Get the current speed of motor 2
#define CMD_GET_ENC1            0x23    // R    | Motor1 encoder count, 4 bytes returned high byte first (signed)
#define CMD_GET_ENC2            0x24    // R    | Motor2 encoder count, 4 bytes returned high byte first (signed)
#define CMD_GET_ENCS            0x25    // R    | Motors encoder count, 8 bytes returned (motor 1 then motor 2) high byte first (signed)
#define CMD_GET_VOLT            0x26    // R    | The supply battery voltage
#define CMD_GET_CURRENT1        0x27    // R    | The current through motor 1
#define CMD_GET_CURRENT2        0x28    // R    | The current through motor 2
#define CMD_GET_ACCELERATION    0x2A    // R    | Get acceleration
#define CMD_GET_MODE            0x2B    // R    | Get mode
#define CMD_SET_ACCELERATION    0x33    // W    | Set acceleration of both motors
#define CMD_SET_SPEED1          0x31    // W    | Motor1 speed (mode 0,1) or speed (mode 2,3)
#define CMD_SET_SPEED2          0x32    // W    | Motor2 speed (mode 0,1) or turn (mode 2,3)
#define CMD_SET_MODE            0x34    // W    | Mode of operation (see below)
#define CMD_RESET_ENCODERS      0x35    // W    | Resets the encoder registers to zero
#define CMD_DISABLE_TIMEOUT     0x38    // W    | Enables a continuous spin
#define CMD_ENABLE_TIMEOUT      0x39    // W    | Enables 2 second timeout of motors when no command


/* MOTORS COMMAND MODES */
#define MODE_0                  0x00    // The meaning of the speed registers is literal speeds in the range of 0 (Full Reverse), 128 (Stop), 255 (Full Forward) (Default Setting).
#define MODE_1                  0x01    // The meaning of the speed registers is literal speeds in the range of -128 (Full Reverse), 0 (Stop), 127 (Full Forward).
#define MODE_2                  0x02    // Speed1 control both motors speed, and speed2 becomes the turn value. Data is in the range of 0 (Full Reverse), 128 (Stop), 255 (Full  Forward).
#define MODE_3                  0x03    // Speed1 control both motors speed, and speed2 becomes the turn value. Data is in the range of -128 (Full Reverse), 0 (Stop), 127 (Full Forward).


/* CONSTANTS */
#define WHEEL_DIAMETER          0.1     //meter
#define SEMI_AXE_LENGTH         0.175   //meter
#define SPD_NULL                0x80    // speed = 0
#define SPD_MAX                 0xFF    // speed = +MAX
#define SPD_MAX_REVERSE         0x00    // speed = -MAX
#define ENCODER_IN_METER        0.00088 // 1 increment = 0.00088 m
#define CURRENT_IN_AMP          0.10    // 123 = 12.3 A
#define BATTERY_IN_VOLT         0.10    // 123 = 12.3 V
#define COMMAND_METER           12.633  // Convert from m.s^-1 to increment : 1m.s^-1 = 127
#define ROTATION_CORRECT        1.0     // Coefficient correcting rotation
#define LOOP_RATE               30      // frenquence du noeud

class MD25
{
private:
    ///----- ATTRIBUTES -----///
    /* Communication */
    std::string name_port;
    int port_opened;

    /* MD25 values */
    unsigned char mode;
    double encoder[2];
    double last_encoder[2];
    double volt;
    double current[2];
    double speed[2];
    double accel;

    /* ROS communication */
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped odom_tf;
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom_msg;
    ros::Publisher publish_odom;
    ros::Time current_time;
    ros::Time last_time;

    /*command values*/
    float twist_linear;
    float twist_angular;
    unsigned char hex_vel[2];

    /*Odometry attributes*/
    double x;
    double y;
    double th;
    
    /* Robot param */
    double semi_axe_length;
    double wheel_diameter;
    

    ///----- CONSTRUCTOR/DESCTRUCTOR -----///
public:
    /* Constructor and Destructor */
    MD25();
    ~MD25(){close(port_opened);}

    ///----- PUBLIC METHODS -----///
    void get_state();
    void twistCb(const geometry_msgs::Twist& twist);
    void publish();
    ros::NodeHandle get_NodeHandled(){return nh_;}
    void stop();

    ///----- PRIVATE METHODS -----///
private :
    /* Control and data methods */
    void reset_encoders();                   // Resets the encoder registers.

    void set_mode(unsigned char mode);                // Set the mode of operation (0 - 3) (default: 0).
    void set_timeout(bool enabled);                     // Enable/disable 2 sec timeout of motors when no coms (default: enabled).
    void set_speed1(unsigned char speed);             // Set the speed of motor 1 (if mode 0 or 1) or linear twist (if mode 2 or 3).
    void set_speed2(unsigned char speed);             // Set the speed of motor 2 (if mode 0 or 1) or angular twist (if mode 2 or 3).
    void set_acceleration(unsigned char acceleration);// Set a desired acceleration rate.
    void set_motor();
    
    void get_mode();                         // Get the current mode of operation.
    void get_encoder1();                     // Encoder 1 position.
    void get_encoder2();                     // Encoder 2 position.
    void get_current1();                     // Returns the current through motor 1.
    void get_current2();                     // Returns the current through motor 2.
    void get_speed1();                       // Returns the current requested speed of motor 1.
    void get_speed2();                       // Returns the current requested speed of motor 2.
    void get_acceleration();                 // Get the set desired acceleration rate.
    void get_volt();                         // Returns the input battery voltage level.
};
