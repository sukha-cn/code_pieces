#include "edison_control/controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    
    edison::I2cController controller;
    ros::spin();

}

