#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include"fusion_pointclouds/params_tfConfig.h"

void setPparamscallback(fusion_pointclouds::params_tfConfig &config, 
                                                            uint32_t level,const ros::NodeHandle& n_private)
{
    std::string name;
    n_private.getParam("name",name); //参数定义在 launch文件中
    // ros::param::get("/name",name);
    std::cout << "TransFrameId:" << name <<std::endl;
    ROS_INFO("updated params  :   x: %lf  y: %lf  z: %lf   roll: %lf   pitch: %lf  yaw: %lf  ",
                                                   config.x, 
                                                   config.y,
                                                   config.z,
                                                   config.roll,
                                                   config.pitch,
                                                   config.yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_dynamic_params");
    ros::NodeHandle n_private("~");//私有节点  为了每个名字不一样。如果是， 全局节点，参数也是全局参数
    dynamic_reconfigure::Server<fusion_pointclouds::params_tfConfig> params_tf_sever;
    dynamic_reconfigure::Server<fusion_pointclouds::params_tfConfig>::CallbackType f;
    f = boost::bind(&setPparamscallback,_1, _2, n_private);
    params_tf_sever.setCallback(f);
    ros::spin();
    return 0;
}


