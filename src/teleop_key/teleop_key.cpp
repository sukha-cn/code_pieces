#include "teleop_key/teleop_key.h"

struct termios TeleopKey::cooked;

TeleopKey::TeleopKey()
{
    double a_scale, l_scale;
    nh_.param("scale_angular", a_scale, A_SCALE_DEFAULT);
    nh_.param("scale_linear", l_scale, L_SCALE_DEFAULT);
    init("cmd_vel", a_scale, l_scale);
}

TeleopKey::TeleopKey(string topic)
{
    double a_scale, l_scale;
    nh_.param("scale_angular", a_scale, A_SCALE_DEFAULT);
    nh_.param("scale_linear", l_scale, L_SCALE_DEFAULT);
    init(topic, a_scale, l_scale);
}

TeleopKey::TeleopKey(string topic, double l_scale, double a_scale)
{
    init(topic, l_scale, a_scale);
}

void TeleopKey::keyLoop()
{
    char c;
    bool dirty = false;
    signal(SIGINT,TeleopKey::quit);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    
    for(;;)
    {
        // get the next event from the keyboard  
        if(read(KEY_FD, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_ = angular_ = 0;
        ROS_DEBUG("value: 0x%02X\n", c);
    
        switch(c)
        {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            angular_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            angular_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            dirty = true;
            break;
        }
    

        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_ * angular_;
        twist.linear.x = l_scale_ * linear_;
        if(dirty == true)
        {
            twist_pub_.publish(twist);    
            dirty=false;
        }
    }
}


void TeleopKey::init(string topic, double l_scale, double a_scale)
{
    linear_ = 0;
    angular_ = 0;
    l_scale_ = l_scale;
    a_scale_ = a_scale;
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(topic, 1);

    termios raw;
    tcgetattr(KEY_FD, &TeleopKey::cooked);
    memcpy(&raw, &TeleopKey::cooked, sizeof(struct termios));
    // In noncanonical mode input is available immediately (without the user having to type a line-delimiter character), no input processing is performed, and line editing is disabled.
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(KEY_FD, TCSANOW, &raw);
}

void TeleopKey::quit(int sig)
{
    tcsetattr(KEY_FD, TCSANOW, &TeleopKey::cooked);
    ros::shutdown();
    exit(0);
}