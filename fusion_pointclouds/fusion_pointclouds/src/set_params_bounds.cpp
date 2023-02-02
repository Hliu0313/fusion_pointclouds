#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include"fusion_pointclouds/params_boundsConfig.h"

void boundsCallback(fusion_pointclouds::params_boundsConfig &config, 
                                            uint32_t level,const ros::NodeHandle& n_private)
{
    std::string name;
    n_private.getParam("name",name); //参数定义在 launch文件中
    // ros::param::get("/name",name); 
    std::cout << "TransFrameId:" << name <<std::endl;
    ROS_INFO("updated params  :   x_min: %lf  x_max: %lf  y_min: %lf   y_max: %lf   z_min: %lf  z_max: %lf  ",
                                                   config.x_min,
                                                   config.x_max,
                                                   config.y_min,
                                                   config.y_max,
                                                   config.z_min,
                                                   config.z_max
                                                   );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounds");
    ros::NodeHandle n2_private("~");//私有节点  为了每个名字不一样。如果是， 全局节点，参数也是全局参数
    dynamic_reconfigure::Server<fusion_pointclouds::params_boundsConfig> bounds_server;
    dynamic_reconfigure::Server<fusion_pointclouds::params_boundsConfig>::CallbackType b;
    b = boost::bind(&boundsCallback,_1, _2,n2_private);
    bounds_server.setCallback(b);
    ros::spin();
    return 0;
}