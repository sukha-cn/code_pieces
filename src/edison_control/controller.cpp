#include "controller/controller.h"

using edison::Controller;
using edison::I2cController;

void Controller::init(std::string topic)
{
    ros::Subscriber sub = nh_.subscribe(topic, 10, drive);
}

I2cController::I2cController(std::string topic) : Controller(topic)
{ 
    init_driver(); 
}

I2cController::I2cController(std::string topic, 
                                int bus = 0, 
                                bool raw = false, 
                                mraa::I2cMode mode = mraa::MRAA_I2C_FAST) : Controller(topic)
{ 
    init_driver(bus, raw, mode); 
}

void I2cController::init_driver(int bus, bool raw, mraa::I2cMode mode)
{
    i2c_ = new mraa::I2c(bus, raw);
    i2c_->frequency(mode);
}

void I2cController::drive()
{


}