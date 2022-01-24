#include "manager.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "April_tags");
    ros::NodeHandle node("~");
    ROS_INFO("April_tags node started.");
    image_transport::ImageTransport image(node);

    manager manager(node,image);
    return 0;
}