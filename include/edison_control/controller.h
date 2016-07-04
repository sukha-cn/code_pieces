#ifndef EDISON_CONTROLLER_H
#define EDISON_CONTROLLER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "teleop_key/teleop_key.h"

#include "mraa.hpp"
#include "math.h"

namespace edison
{

class Controller
{
public:
    Controller() { init("cmd_vel"); }
    Controller(std::string topic) { init(topic); }
    ~Controller() { if (i2c_) delete i2c; }

private:
    void init(std::string topic);
    virtual void drive(const geometry_msgs::Twist::ConstPtr& msg) = 0;

private:
    ros::NodeHandle nh_;
};

class I2cController : public Controller
{
public:
    I2cController() : Controller() { init_driver(); }
    I2cController(std::string topic);
    ~I2cController() {}

private:
    void init_driver(int bus, bool raw, mraa::I2cMode mode);
    virtual void drive(const geometry_msgs::Twist::ConstPtr& msg);
    

private:
    mraa::I2c* i2c_;
};

}

#endif