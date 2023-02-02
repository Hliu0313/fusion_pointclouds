//该功能包接收多雷达话题经过时间同步和坐标变化融合为一个 点云话题   

#include <ros/ros.h>
#include"fusion_pointclouds/load_params.h"
//消息过滤 -- 点云数据时间同步
#include <message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<message_filters/sync_policies/exact_time.h>
//Provides conversions from PCL data types and ROS message types 
#include <pcl_conversions/pcl_conversions.h>  //Provides conversions from PCL data types and ROS message types
#include <pcl/point_types.h>                                     //点类型定义
#include <sensor_msgs/PointCloud2.h>
// tf 坐标变化 四元素/变化矩阵/欧拉角之间转化
#include<tf2/LinearMath/Quaternion.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<pcl/registration/transforms.h>//变换矩阵类 
//滤波 
#include <pcl/filters/passthrough.h> //直通滤波
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/radius_outlier_removal.h>//条件滤波
#include <pcl/filters/conditional_removal.h>//条件滤波

class FusionPcNode
{
    private:
       //1.定义结构体 initial_parameters_t 用于接收yaml文件参数  load_params.h与load_params.cpp 
       load_params::initial_parameters_t params;

       //2.消息同步
       //构造函数中，利用message_filters的Subscriber订阅不同话题，符合时间同步条件进入回调函数
        message_filters::Subscriber<sensor_msgs::PointCloud2>* pc_sub1;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* pc_sub2;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* pc_sub3;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* pc_sub4;

      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
                                                                                                                              sensor_msgs::PointCloud2,
                                                                                                                              sensor_msgs::PointCloud2,
                                                                                                                              sensor_msgs::PointCloud2
                                                                                                                              > MySyncPolicy;

     message_filters::Synchronizer<MySyncPolicy>* sync_;
     //3.回调函数融合处理点云
     void callback(const sensor_msgs::PointCloud2::ConstPtr& pc_raw1,
                                  const sensor_msgs::PointCloud2::ConstPtr& pc_raw2,
                                  const sensor_msgs::PointCloud2::ConstPtr& pc_raw3,
                                  const sensor_msgs::PointCloud2::ConstPtr& pc_raw4
                                  );
      
      //4.在回调函数中对点云进行坐标变换 -- 变化矩阵定义
      Eigen::Isometry3d trans_cpc1_to_ppc = Eigen::Isometry3d::Identity(); //为4阶类型的单位阵 
      Eigen::Isometry3d trans_cpc2_to_ppc = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d trans_cpc3_to_ppc = Eigen::Isometry3d::Identity();

      void loadTransParams( Eigen::Isometry3d& T,
                                const load_params::ParamTf& trans_params);

      //5.直通 -- 点云过滤直接对点云的X、Y、Z轴的点云坐标约束来进行滤波 
      void passthroughFiter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_pc,
                                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pc,
                                                        const load_params::ParamsBounds& params_bound);
     pcl::PassThrough<pcl::PointXYZI> pass;// 声明直通滤波

     //6.条件滤波-- 坐标约束和强度约束，一次设置多个条件进行约束
     void conditionFiter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_pc,
                                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pc,
                                                        const load_params::ParamsBounds& params_bound);
      
      //7.发布回调函数融合后点云
     ros::Publisher pub_fusion_pc;
     //6.点云坐标变化参数动态调整  ---  set_dynamic_params.cpp  直接修改参数值，无需声明
     //7.点云过滤直接对点云的X、Y、Z轴的点云坐标约束来进行滤波 --- 点云过滤直接对点云的X、Y、Z轴的点云坐标约束来进行滤波 ---  

   public:
     FusionPcNode(ros::NodeHandle &nh);
     ~ FusionPcNode();
};