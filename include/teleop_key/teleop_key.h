#ifndef TELEOP_KEY_H
#define TELEOP_KEY_H

#include <string>
#include <termios.h>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

const int KEYCODE_R = 0x43;
const int KEYCODE_L = 0x44;
const int KEYCODE_U = 0x41;
const int KEYCODE_D = 0x42;
const int KEYCODE_Q = 0x71;
const int KEY_FD = 0;

const double L_SCALE_DEFAULT = 1.0;
const double A_SCALE_DEFAULT = 1.0;

using std::string;

class TeleopKey
{
public:
    TeleopKey();
    TeleopKey(string topic);
    TeleopKey(string topic, double l_scale, double a_scale);
    void keyLoop();

private:
    void init(string topic, double l_scale, double a_scale);
    static void quit(int sig);

    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
    static struct termios cooked;
};

#endif