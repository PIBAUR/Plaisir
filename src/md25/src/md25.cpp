 /********************************************************************
 * @filename    MD25.cpp
 * @author      Thomas B. Joergensen (thomas.bal.jorgensen@gmail.com)
 * @date        06 FEB 2012
 * @version     1.0

 * @target      mbed(NXP LPC1768 - ARM Cortex M3 - 32-bit)
 *
 * @desciption  A library for interacting with the MD25 DC motor
 *              controller. Find more at:
 *              http://www.robot-electronics.co.uk/htm/md25tech.htm
 *              http://www.robot-electronics.co.uk/htm/md25i2c.htm
 *******************************************************************/

/* Includes */
#include "md25.h"
#include "sstream"
/*** CONSTRUCTOR AND DESTRUCTOR ***/

MD25::MD25() :
        encoder{0.0, 0.0}, last_encoder{0.0, 0.0},
        twist_linear(0.0), twist_angular(0.0),
        x(0.0), y(0.0), th(0.0)
{
    /*** Initialisation des messages ****/
    /* odometry */
    std::string tf_prefix;
    std::stringstream frame_odom, frame_base;

    if (nh_.getParam("tf_prefix", tf_prefix))
    {
        frame_odom << tf_prefix <<"/odom";
        frame_base << tf_prefix <<"/base_link";
    }
    else
    {
        frame_odom<<"/odom";
        frame_base<<"/base_link";
    }
    odom_msg.header.frame_id = frame_odom.str();

    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    publish_odom= nh_.advertise<nav_msgs::Odometry> ("odom", 50);
    publish_batt= nh_.advertise<geometry_msgs::Vector3> ("md25_input_power", 1);

    /* tf */
    odom_tf.header.frame_id = frame_odom.str();
    //odom_msg.header.frame_id = odom_tf.header.frame_id;
    odom_tf.child_frame_id = frame_base.str();
    odom_tf.transform.translation.z = 0.0;

    /* Time */
    current_time = ros::Time::now();


    /*** get rosparam ***/
    nh_.param<double>("/semi_axe_length",semi_axe_length,SEMI_AXE_LENGTH);
    nh_.param<double>("/wheel_diameter",wheel_diameter,WHEEL_DIAMETER);
    nh_.param<std::string>("/md25_serial_port",name_port,"/dev/ttyUSB0");

    ROS_INFO_STREAM_NAMED("MD25_node","Wheel diameter : "<< wheel_diameter);
    ROS_INFO_STREAM_NAMED("MD25_node","Robot semi axe : "<< semi_axe_length);
    ROS_INFO_STREAM_NAMED("MD25_node","Using port : "<< name_port);


    /**** Initialisation du port ****/
    /* Open and config port */
    port_opened = open((char*)name_port.c_str(), O_RDWR | O_NOCTTY );

    /* Print if could not open the port */
    ROS_ERROR_STREAM_COND_NAMED(port_opened < 0, "MD25_node", "Unable to open " << name_port);

    struct termios options;


    /* Get the current options for the port */
    ROS_ERROR_STREAM_COND_NAMED( (tcgetattr(port_opened, &options)!=0),
        "MD25_node", "tcgetattr() 3 failed");

    /* Set the baud rates to 38400 */
    cfsetospeed(&options, B38400);
    cfsetispeed(&options, B38400);


    /* Mask the character size to 8 bits*/
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    //options.c_cflag &= ~CSIZE;
    //options.c_cflag |=  CS8;

    /* Enable the receiver and set local mode */
    options.c_cflag |= CLOCAL | CREAD;
    //options.c_cflag |= (CLOCAL | CREAD);

    /* No parity */
    options.c_cflag &= ~(PARENB | PARODD);

    /* Two stop bits*/
    options.c_cflag |= CSTOPB;
    //options.c_cflag &= ~CSTOPB;

    /* Disable hardware flow control */
    options.c_cflag &= ~CRTSCTS;

    /* Break */
    options.c_iflag=IGNBRK;

    /* Software handshake */
    options.c_iflag &= ~(IXON|IXOFF|IXANY);

    /* Enable data to be processed as raw input */
    //options.c_lflag &= ~(ICANON | ECHO | ISIG);
    //options.c_lflag &= ~(ISIG);
    options.c_lflag=0;
    options.c_oflag=0;

    /* */
    options.c_cc[VMIN]  = 4;
    options.c_cc[VTIME] = 1;

    /* Set the new options for the port */
    ROS_ERROR_STREAM_COND_NAMED( tcsetattr(port_opened, TCSANOW, &options) != 0,
        "MD25_node", "tcsetattr() 1 failed");
    //tcsetattr(port_opened, TCSANOW, &options) != 0;

    int mcs=0;
    ioctl(port_opened, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
    ioctl(port_opened, TIOCMSET, &mcs);

    /* Clear both the input and output buffers. */
    ROS_ERROR_STREAM_COND_NAMED( tcsetattr(port_opened, TCSANOW, &options)!=0,
        "MD25_node", "tcsetattr() 2 failed");

    tcflush(port_opened, TCIOFLUSH);

    /**** Initialisation des parametres du MD25 ****/
    reset_encoders();
    set_timeout(true);
    set_mode(MODE_0);
}



/*** CONTROL METHODS ***/

/**
 * Set mode of operation.
 *
 * @param m      Set mode of operation (0 - 3) (default: 0).
 */
void MD25::set_mode(unsigned char m)
{
    /* Check input is valid */
    if (m < 0 || m > 3)
    {
        ROS_WARN_STREAM_NAMED("MD25_node", "invalid mode value" );
    }
    else
    {
        unsigned char command[3] = {SYNC_BYTE, CMD_SET_MODE, m};
        /* Set mode */
        if (write(port_opened, command, 3) == -1)
        {
            ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        }
        /* Set mode variable */
        this->mode = m;
    }
    return;
}


void MD25::get_mode()
{
    unsigned char data;
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_MODE};

    /* flush before ask */
    tcflush(port_opened, TCIOFLUSH);

    if(write(port_opened, command, 2) == -1){
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }

    return ;
}


/**
 * Get encoder 1 position.
 */
void MD25::get_encoder1()
{
    /* Variables */
    unsigned char data[4];
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_ENC1};

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    /* Get encoder bytes */
    if (read(port_opened, &data[0], 4) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    /* Combine the position bytes */
    encoder[0] = ((data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3]);

    /* convert value */
    if (encoder[0]>=255.0*255.0*255.0*255.0/2.0)
    {
        encoder[0] = ~((data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3]);
        encoder[0] *= -1;
    }
    encoder[0]*=ENCODER_IN_METER;
    ROS_DEBUG_STREAM("Codeur 1 : "<<encoder[0]<<" |\t"<<(int)data[0]<<"\t"<<(int)data[1]<<"\t"<<(int) data[2]<<"\t"<<(int)data[3]);
    //ROS_DEBUG_STREAM(encoder[0]);
    //std::cout<<"Codeur 1 : "<<(int)data[0]<<" "<<(int)data[1]<<" "<<(int) data[2]<<" "<<(int)data[3]<<std::endl;
    return ;
}


/**
 * Get encoder 2 position.
 */
void MD25::get_encoder2()
{
   /* Variables */
    unsigned char data[4];
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_ENC2};

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    /* Get encoder bytes */
    if (read(port_opened, &data[0], 4) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    /* Combine the position bytes */
    encoder[1] = ((data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3]);

    /* convert value */
    if (encoder[1]>=255.0*255.0*255.0*255.0/2.0)
    {
        encoder[1] = ~((data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3]);
        encoder[1] *= -1;
    }
    encoder[1]*=ENCODER_IN_METER;
    ROS_DEBUG_STREAM("Codeur 2 : "<<encoder[1]<<" |\t"<<(int)data[0]<<"\t"<<(int)data[1]<<"\t"<<(int) data[2]<<"\t"<<(int)data[3]);
    //ROS_DEBUG_STREAM(encoder[1]);
    //std::cout<<"Codeur 2 : "<<(int)data[0]<<" "<<(int)data[1]<<" "<<(int) data[2]<<" "<<(int)data[3]<<std::endl;
    return ;
}


/**
 * Reset the encoder registers.
 */
void MD25::reset_encoders()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_RESET_ENCODERS};

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }
    return ;
}


/**
 *  Get current through motor 1
 */
void MD25::get_current1()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_CURRENT1};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }
    current[0]= data*CURRENT_IN_AMP;
    ROS_DEBUG_STREAM("Current through motor 1 : "<<current[0]);

    return ;
}


/**
 *  Get current through motor 2
 */
void MD25::get_current2()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_CURRENT2};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }
    current[1]= data*CURRENT_IN_AMP;
    ROS_DEBUG_STREAM("Current through motor 1 : "<<current[1]);


    return;
}


/**
 * Enable/disable 2 sec timeout of motors when no I2C comms (default: enabled).
 *
 * @param enabled   enable = 1 | disable = 0.
 */
void MD25::set_timeout(bool enabled)
{
    unsigned char command[2]={SYNC_BYTE, 0};

    if (enabled)
    {
        command[1] = CMD_ENABLE_TIMEOUT;
    }
    else
    {
        command[1] = CMD_DISABLE_TIMEOUT;
    }

    /* Reset encoders */
    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }
    return ;
}


/**
 * Get the speed of motor 1
 */
void MD25::get_speed1()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_SPEED1};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    ROS_DEBUG_STREAM("Speed motor 1 : " << (int)data);
    speed[0]=data;
    return ;
}


/**
 * Get the speed of motor 2
 */
void MD25::get_speed2()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_SPEED2};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    ROS_DEBUG_STREAM("Speed motor 2 : " << (int)data);
    speed[1]=data;
    return ;
}


/**
 * Set the speed of motor 1.
 *
 * @param speed     Faster the higher a number (mode 0/2: 0 -> 255 | mode 1/3: -128 to 127).
 */
void MD25::set_speed1(unsigned char speed)
{
    unsigned char command[3] = {SYNC_BYTE, CMD_SET_SPEED1,speed};

    if (write(port_opened, command, 3) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }

    return ;
}


/**
 * Set the speed of motor 2.
 *
 * @param speed     Faster the higher a number (mode 0: 0 -> 255 | mode 1: -128 to 127).
 */
void MD25::set_speed2(unsigned char speed)
{
    unsigned char command[3] = {SYNC_BYTE, CMD_SET_SPEED2,speed};

    if (write(port_opened, command, 3) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }

    return ;
}


/**
 * Set the desired acceleration rate.
 *
 * if new speed > current speed:
 *      steps = (new speed - current speed) / acceleration register
 * if new speed < current speed:
 *      steps = (current speed - new speed) / acceleration register
 * time = steps * 25ms
 * Example:
 *      Time/step: 25ms | Current speed: 0 | New speed: 255 | Steps: 255 | Acceleration time: 6.375s.
 * @param acceleration  Faster the higher a number (default: 5).
 */
void MD25::set_acceleration(unsigned char acceleration)
{
    if(acceleration > 10)
    {
        ROS_WARN_STREAM_NAMED("MD25","acceleration value not valid");
        return;
    }
    unsigned char command[3] = {SYNC_BYTE, CMD_SET_ACCELERATION, acceleration};

    if (write(port_opened, command, 3) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
    }

    return;
}


/**
 * Get the set desired acceleration rate.
 *
 * if new speed > current speed:
 *      steps = (new speed - current speed) / acceleration register
 * if new speed < current speed:
 *      steps = (current speed - new speed) / acceleration register
 * time = steps * 25ms
 * Example:
 *      Time/step: 25ms | Current speed: 0 | New speed: 255 | Steps: 255 | Acceleration time: 6.375s.
 * @return  Faster the higher a number (default: 5).
 */
void MD25::get_acceleration()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_ACCELERATION};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }
    accel = data;
    ROS_DEBUG_STREAM("Acceleration : " << accel);
    return ;
}


/**
 * Get the current voltage
 */
void MD25::get_volt()
{
    unsigned char command[2] = {SYNC_BYTE, CMD_GET_VOLT};
    unsigned char data;

    tcflush(port_opened, TCIOFLUSH);

    if (write(port_opened, command, 2) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }

    if (read(port_opened, &data, 1) == -1)
    {
        ROS_ERROR_STREAM_NAMED("MD25_node", "cannot write/read port" << name_port);
        return ;
    }
    volt = data*BATTERY_IN_VOLT;
    ROS_DEBUG_STREAM("Voltage : " << volt);
    return;
}


void MD25::get_state()
{
    last_time = current_time;
    current_time = ros::Time::now();
    get_encoder1();
    get_encoder2();
}


void MD25::set_motor()
{
    double vit_1 = round(COMMAND_METER*(twist_linear-semi_axe_length*twist_angular)/wheel_diameter) + (double)SPD_NULL;
    double vit_2 = round(COMMAND_METER*(twist_linear+semi_axe_length*twist_angular)/wheel_diameter) + (double)SPD_NULL;

    /* check speed */
    if(vit_1 < (double)SPD_MAX_REVERSE)
    {
        hex_vel[0] = SPD_MAX_REVERSE;
    }
    else if(vit_1 > (double)SPD_MAX)
    {
        hex_vel[0] = SPD_MAX;
    }
    else{
        hex_vel[0] = (unsigned char)vit_1;
    }

    if(vit_2 < (double)SPD_MAX_REVERSE)
    {
        hex_vel[1] = SPD_MAX_REVERSE;
    }
    else if(vit_2 > (double)SPD_MAX)
    {
        hex_vel[1] = SPD_MAX;
    }
    else{
        hex_vel[1] = (unsigned char)vit_2;
    }

    set_speed1(hex_vel[0]);
    set_speed2(hex_vel[1]);
}

void MD25::twistCb(const geometry_msgs::Twist& msg)
{
    twist_linear = -msg.linear.x;
    twist_angular = msg.angular.z;
    set_motor();
}


void MD25::publish()
{
    geometry_msgs::Quaternion quaternion;

    //calcul delta encoder
    double de1 = encoder[0] - last_encoder[0];
    double de2 = encoder[1] - last_encoder[1];

    //calcul vitesse lineaire et angulaire
    double du = -(de1+de2)/2.0;
    double dth = ROTATION_CORRECT*(de2-de1)/(2*semi_axe_length);
    double dt = current_time.toSec() - last_time.toSec();

    //calcul de la nouvelle posture
    x += du*cos(th);
    y += du*sin(th);
    th += dth;

    quaternion.z = sin(th/2.0);
    quaternion.w = cos(th/2.0);
    quaternion.x = 0;
    quaternion.y = 0;

    //maj odom
    odom_msg.header.stamp = current_time;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation=quaternion;

    odom_msg.twist.twist.linear.x = du/dt;
    odom_msg.twist.twist.angular.z = (dth)/dt;

    //tf
    odom_tf.header.stamp = current_time;
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.rotation = quaternion;

    //publish
    ROS_DEBUG_STREAM("Odom to base : "<< x<< " , "<<y<< " , "<<th);
    publish_odom.publish(odom_msg);
    odom_broadcaster.sendTransform(odom_tf);

    //reaffection last encoder pour calcul delta encoder
    last_encoder[0] = encoder[0];
    last_encoder[1] = encoder[1];
}


void MD25::stop()
{
    twist_linear = 0.0;
    twist_angular = 0.0;
    hex_vel[0]=SPD_NULL;
    hex_vel[1]=SPD_NULL;
    set_speed1(SPD_NULL);
    set_speed2(SPD_NULL);

}

void MD25::get_battery_state()
{
    void get_current1();                     // Returns the current through motor 1.
    void get_current2();                     // Returns the current through motor 2.
    void get_volt();                         // Returns the input battery voltage level.

    geometry_msgs::Vector3 battery_msg;
    battery_msg.x = volt;
    battery_msg.y = current[0];
    battery_msg.z = current[1];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "md25");
    ros::NodeHandle n;
    MD25 m;
    ros :: Subscriber sub = n.subscribe("cmd_vel", 1, &MD25::twistCb, &m);
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();
// ecrire traitement des valeurs twist
    while(ros::ok()){
        m.get_state();
        m.publish();
        m.get_battery_state();
        ros::spinOnce();
        loop_rate.sleep();
    }
    m.stop();
    ros::spin();
    return 0;
}




