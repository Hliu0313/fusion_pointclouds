//从ros参数空间加载所有参数
#include"fusion_pointclouds/load_params.h"

namespace load_params
{
    void loadParams(initial_parameters_t& i_params)
    {
      //进行动态调参时候，回调函数处理还需要传入节点句柄，不够方便
      //   nh.getParam("/fusion_lidar_num", i_params.fusion_lidar_num);  
      //   nh.param<int>("/fusion_lidar_num", i_params.fusion_lidar_num, 0.0); 
       ros::param::get("/fusion_lidar_num", i_params.fusion_lidar_num);
       ros::param::get("/topics/parent_pc_topic", i_params.parent_pc_topic);
       ros::param::get("/topics/child_pc_topic1", i_params.child_pc_topic1);
       ros::param::get("/topics/child_pc_topic2", i_params.child_pc_topic2);
       ros::param::get("/topics/child_pc_topic3", i_params.child_pc_topic3);

       ros::param::get("/topics/fusion_pc_topic", i_params.fusion_pc_topic);
       ros::param::get("/topics/fusion_pc_frame_id", i_params.fusion_pc_frame_id);    

       ros::param::get("/set_params_tf", i_params.set_params_tf);  
       ros::param::get("/set_params_internal_bounds", i_params.set_params_internal_bounds);
       ros::param::get("/set_params_external_bounds", i_params.set_params_external_bounds);
       ros::param::get("/set_dynamic_params", i_params.set_dynamic_params);  
       
       ros::param::get("/cpc1_to_ppc/x", i_params.cpc1_to_ppc.x);
       ros::param::get("/cpc1_to_ppc/y", i_params.cpc1_to_ppc.y);
       ros::param::get("/cpc1_to_ppc/z", i_params.cpc1_to_ppc.z);
       ros::param::get("/cpc1_to_ppc/roll", i_params.cpc1_to_ppc.roll);
       ros::param::get("/cpc1_to_ppc/pitch", i_params.cpc1_to_ppc.pitch);
       ros::param::get("/cpc1_to_ppc/yaw", i_params.cpc1_to_ppc.yaw);

       ros::param::get("/cpc2_to_ppc/x", i_params.cpc2_to_ppc.x);
       ros::param::get("/cpc2_to_ppc/y", i_params.cpc2_to_ppc.y);
       ros::param::get("/cpc2_to_ppc/z", i_params.cpc2_to_ppc.z);
       ros::param::get("/cpc2_to_ppc/roll", i_params.cpc2_to_ppc.roll);
       ros::param::get("/cpc2_to_ppc/pitch", i_params.cpc2_to_ppc.pitch);
       ros::param::get("/cpc2_to_ppc/yaw", i_params.cpc2_to_ppc.yaw);
       
       ros::param::get("/cpc3_to_ppc/x", i_params.cpc3_to_ppc.x);
       ros::param::get("/cpc3_to_ppc/y", i_params.cpc3_to_ppc.y);
       ros::param::get("/cpc3_to_ppc/z", i_params.cpc3_to_ppc.z);
       ros::param::get("/cpc3_to_ppc/roll", i_params.cpc3_to_ppc.roll);
       ros::param::get("/cpc3_to_ppc/pitch", i_params.cpc3_to_ppc.pitch);
       ros::param::get("/cpc3_to_ppc/yaw", i_params.cpc3_to_ppc.yaw);

      ros::param::get("/internal_bounds/x_min" , i_params.internal_bounds.x_min);
       ros::param::get("/internal_bounds/x_max", i_params.internal_bounds.x_max);
       ros::param::get("/internal_bounds/y_min" , i_params.internal_bounds.y_min);
       ros::param::get("/internal_bounds/y_max", i_params.internal_bounds.y_max);
       ros::param::get("/internal_bounds/z_min" , i_params.internal_bounds.z_min);
       ros::param::get("/internal_bounds/z_max", i_params.internal_bounds.z_max);

       ros::param::get("/external_bounds/x_min" , i_params.external_bounds.x_min);
       ros::param::get("/external_bounds/x_max", i_params.external_bounds.x_max);
       ros::param::get("/external_bounds/y_min" , i_params.external_bounds.y_min);
       ros::param::get("/external_bounds/y_max", i_params.external_bounds.y_max);
       ros::param::get("/external_bounds/z_min" , i_params.external_bounds.z_min);
       ros::param::get("/external_bounds/z_max", i_params.external_bounds.z_max);
    }
}  // namespace load_params
