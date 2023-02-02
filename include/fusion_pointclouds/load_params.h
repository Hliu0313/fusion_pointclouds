#ifndef load_params_h_
#define load_params_h_

#include <ros/ros.h>

namespace load_params
{
   typedef struct ParamTf  ///< The Point transform parameter  
    {
      float x = 0.0f;      ///< unit, m
      float y = 0.0f;      ///< unit, m
      float z = 0.0f;      ///< unit, m
      float roll = 0.0f;   ///< unit, radian
      float pitch = 0.0f;  ///< unit, radian
      float yaw = 0.0f;    ///< unit, radian
    } ParamTf;

    typedef struct ParamsBounds /// PCL滤波保留某个或多个字段的值满足条件的数据点
    {
      float  x_min;      ///< unit, m
      float  x_max;
      float  y_min;
      float  y_max;
      float  z_min;
      float  z_max;
    }ParamsBounds;

    struct  initial_parameters_t
     {  
       int fusion_lidar_num;                    //融合 lidar 点云数量 
       std::string parent_pc_topic;       //订阅 lidar 点云话题
       std::string child_pc_topic1;
       std::string child_pc_topic2;
       std::string child_pc_topic3;

       std::string fusion_pc_topic;       //融合后发布点云话题名称
       std::string fusion_pc_frame_id;

       bool set_params_tf;
       bool set_params_internal_bounds;
       bool set_params_external_bounds;
       bool set_dynamic_params;
       
       ParamTf cpc1_to_ppc;
       ParamTf cpc2_to_ppc;
       ParamTf cpc3_to_ppc;
       ParamsBounds internal_bounds;
       ParamsBounds external_bounds;
     };

    void loadParams( initial_parameters_t& i_params);
}  // namespace load_param

#endif