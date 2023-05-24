#include"fusion_pointclouds/fusion_pointclouds.h"
#include"fusion_pointclouds/load_params.h"
#include <iostream>

using namespace message_filters;
using namespace message_filters::sync_policies;

FusionPcNode::FusionPcNode(ros::NodeHandle &nh)
{
     ROS_INFO("start message filter");
     //---> 1.加载 yaml 文件参数，函数申明在load_params.h 定义在 load_params.cpp中.
     load_params::loadParams(params); // 若不进行动态调参只加载一次。

     //---> 2.消息过滤 -- 时间戳同步
     pc_sub1 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,params.parent_pc_topic,10);
     pc_sub2 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,params.child_pc_topic1,10);
     pc_sub3 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,params.child_pc_topic2,10);
     pc_sub4 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,params.child_pc_topic3,10);

     sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *pc_sub1, 
                                                                                                                                                                        *pc_sub2,
                                                                                                                                                                        *pc_sub3,
                                                                                                                                                                        *pc_sub4);
     //注册消息同步回调函数                                                                                                                                                
     sync_->registerCallback(boost::bind(&FusionPcNode::callback,this,_1,_2,_3,_4));
     //发布一个 params.fusion_pc_topic 话题，消息类型sensor_msgs::PointCloud2， 消息队列长度10
     FusionPcNode::pub_fusion_pc = nh.advertise<sensor_msgs::PointCloud2>(params.fusion_pc_topic,10);
      ROS_INFO(" message filter success!");
     ros::spin();
}
//声明一个析构函数
FusionPcNode::~FusionPcNode(){};

void FusionPcNode::callback(const sensor_msgs::PointCloud2::ConstPtr& pc_raw1,
                                                              const sensor_msgs::PointCloud2::ConstPtr& pc_raw2,
                                                              const sensor_msgs::PointCloud2::ConstPtr& pc_raw3,
                                                              const sensor_msgs::PointCloud2::ConstPtr& pc_raw4
                                                              )
{
     //把 ROS PointCloud2 转为 PCL 第一代 PointCloud，方便用 PCL 库处理：
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pc_raw1, *pc_local_1);//父坐标系点云
    //子坐标系点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_trans_2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pc_raw2, *pc_local_2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_3(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_trans_3(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pc_raw3, *pc_local_3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_4(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_trans_4(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pc_raw4, *pc_local_4);

       if (params.set_dynamic_params) //是否接收rqt_reconfigure 动态参数信息
             {
                 load_params::loadParams(params); //重新加载rosparam参数空间参数
             }
       //---> 3.在回调函数中对点云进行坐标变换 -- 变化矩阵定义
        if (params.set_params_tf)// Poins进行坐标转化
            {
                FusionPcNode::loadTransParams(trans_cpc1_to_ppc, params.cpc1_to_ppc);
                pcl::transformPointCloud(*pc_local_2, *pc_trans_2 , trans_cpc1_to_ppc.matrix());

                FusionPcNode::loadTransParams(trans_cpc2_to_ppc, params.cpc2_to_ppc);
                pcl::transformPointCloud(*pc_local_3, *pc_trans_3 , trans_cpc2_to_ppc.matrix());

                FusionPcNode::loadTransParams(trans_cpc3_to_ppc, params.cpc3_to_ppc);
                pcl::transformPointCloud(*pc_local_4, *pc_trans_4 , trans_cpc3_to_ppc.matrix());
            }
        else //不进行坐标变换
            {
                *pc_trans_2 = *pc_local_2;
                *pc_trans_3 = *pc_local_3;
                *pc_trans_4 = *pc_local_4;
            }
        
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_fusion_local(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_fusion_external_bounds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_fusion_internal_bounds(new pcl::PointCloud<pcl::PointXYZI>());

    *pc_fusion_local = *pc_local_1;//初始化
    //---> 3.回调函数融合处理点云
    switch (params.fusion_lidar_num)
    {
    case 4:*pc_fusion_local = *pc_fusion_local + *pc_trans_4;
    case 3:*pc_fusion_local = *pc_fusion_local + *pc_trans_3;
    case 2:*pc_fusion_local = *pc_fusion_local + *pc_trans_2; break;
    default:std::cout << "Error: There is no matching option for fusion_lidar_num."; break;
    }

//---> 5.点云过滤直接对点云的X、Y、Z轴的点云坐标约束来进行滤波 --- 点云过滤直接对点云的X、Y、Z轴的点云坐标约束来进行滤波 ---  

    if (params.set_params_external_bounds)
    {
       FusionPcNode::passthroughFiter(pc_fusion_local, pc_fusion_external_bounds, params.external_bounds);
    }
    else
    {
        *pc_fusion_external_bounds = *pc_fusion_local;
    }
    //---> 6.条件滤波--》坐标约束和强度约束，一次设置多个条件进行约束
  if (params.set_params_internal_bounds)
    {
        FusionPcNode::conditionFiter(pc_fusion_external_bounds, pc_fusion_internal_bounds, params.internal_bounds);
    }
    else
    {
        *pc_fusion_internal_bounds = *pc_fusion_external_bounds;
    }

    //---> 7.发布回调函数融合后点云
    sensor_msgs::PointCloud2 fusion_pc_msg;

    //转为ros点云便于发布话题
    pcl::toROSMsg(*pc_fusion_internal_bounds, fusion_pc_msg); 
    fusion_pc_msg.header.frame_id = params.fusion_pc_frame_id;
    FusionPcNode::pub_fusion_pc.publish(fusion_pc_msg);
}

//定义一个 RPY 转化为变化矩阵，便于 pcl::transformPointCloud 调用
void FusionPcNode::loadTransParams( Eigen::Isometry3d& T,
                                                                         const load_params::ParamTf& trans_params)
{
    T = Eigen::Isometry3d::Identity(); //虽然是3d,实质4x4矩阵
    //方法1：定义旋转向量--有问题  RPY --> 旋转向量
    // Eigen::AngleAxisd rotation_vector(1, Eigen::Vector3d(trans_params.roll, trans_params.pitch, trans_params.yaw) );
    // T.rotate(rotation_vector);

    //方法2：定义旋转矩阵-- RPY --> 旋转矩阵
    //1.初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw) 
    Eigen::Vector3d rpy(trans_params.roll, trans_params.pitch, trans_params.yaw);
    //2.欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * 
                                            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * 
                                            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
    T.rotate(rotation_matrix3);
    T.pretranslate(Eigen::Vector3d(trans_params.x, trans_params.y ,trans_params.z));
}

void FusionPcNode::passthroughFiter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_pc,
                                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pc,
                                                        const load_params::ParamsBounds& params_bound)
{
        pcl::PointCloud<pcl::PointXYZI>::Ptr x(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr z(new pcl::PointCloud<pcl::PointXYZI>);
        // Filter out the experimental region//Hliu筛选实验区区域
        //直通滤波算作最为简单、粗暴的一种滤波方式，来达到点云滤波效果 

        pass.setInputCloud(input_pc);// 传入点云数据
        pass.setFilterFieldName("x");// 设置操作的坐标轴
        pass.setFilterLimits(params.external_bounds.x_min, params.external_bounds.x_max);
        //设置坐标范围	// pass.setFilterLimitsNegative(true); // 保留数据函数
        // 下面图片点云过滤是上一张图片的全局剩余的点云，这里在代码中的pass.setFilterLimitsNegative(true);
        // 就是这个功能作用，是否保存滤波的限制范围内的点云，默认为false，保存限制范围点云，true时候是相反。
        pass.filter(*x); // 进行滤波输出

        pass.setInputCloud(x);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(params.external_bounds.z_min, params.external_bounds.z_max);
        pass.filter(*z);

        pass.setInputCloud(z);
        pass.setFilterFieldName("y");

        pass.setFilterLimits(params.external_bounds.y_min, params.external_bounds.y_max);
        pass.filter(*output_pc);
        // *output_pc = *input_pc;
};

void FusionPcNode::conditionFiter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_pc,
                                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pc,
                                                        const load_params::ParamsBounds& params_bounds)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *cloud = *input_pc;
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond (new
    pcl::ConditionOr<pcl::PointXYZI> ());
 // 设置Z轴的限制范围 
 range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
  pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::GT,params_bounds.z_max )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::LT,params_bounds.z_min)));
//   // 设置Y轴的限制范围 
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GT,params_bounds.y_max)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT,params_bounds.y_min)));
//   // 设置X轴的限制范围
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::GT,params_bounds.x_max)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::LT,params_bounds.x_min)));

  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud);
 condrem.filter (*output_pc);

  //  std::cerr << "Cloud before filtering: " << input_pc->points.size() << std::endl;

  // // display pointcloud after filtering
  // std::cerr << "Cloud after filtering: " << output_pc->points.size() << std::endl;
}


int main(int argc, char** argv)
{
    
    ros::init(argc, argv,"fusion_pc_node");

    ros::NodeHandle nh;
    
    FusionPcNode fusionPc(nh);//括号法 -- 显示法FusionPcNode fusionPc = FusionPcNode(nh);

    return 0;
}
