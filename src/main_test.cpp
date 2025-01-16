 #include "SCCFS.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <boost/thread.hpp>
// #include "object_msgs/DynamicObjectArray.h"
// #include "object_msgs/Semantic.h"
// #include "object_msgs/Shape.h"
// #include "object_msgs/State.h"
// #include "CoarsePathGenerator.hpp"
// #include "Obstacles.hpp"
using namespace std;
// #include "modules/planning/planning_base/reference_line/spiral_reference_line_smoother.h"
// #include "CoarsePathGenerator.hpp"
// #include "gtest/gtest.h"
// // #include "modules/planning/planning_base/proto/reference_line_smoother_config.pb.h"
// using namespace apollo::planning;


// ReferenceLineSmootherConfig config_;
// std::vector<Eigen::Vector2d> raw_points_;
// void SetUp() {
//     config_.mutable_spiral()->set_max_deviation(0.1);
//     config_.mutable_spiral()->set_max_iteration(300);
//     config_.mutable_spiral()->set_opt_tol(1.0e-6);
//     config_.mutable_spiral()->set_opt_acceptable_tol(1.0e-4);

//     config_.mutable_spiral()->set_weight_curve_length(1.0);
//     config_.mutable_spiral()->set_weight_kappa(1.0);
//     config_.mutable_spiral()->set_weight_dkappa(100.0);

//     raw_points_.resize(21);
//     raw_points_[0] = {4.946317773, 0.08436953512};
//     raw_points_[1] = {5.017218975, 0.7205757236};
//     raw_points_[2] = {4.734635316, 1.642930209};
//     raw_points_[3] = {4.425064575, 2.365356462};
//     raw_points_[4] = {3.960102096, 2.991632152};
//     raw_points_[5] = {3.503172702, 3.44091492};
//     raw_points_[6] = {2.989950824, 3.9590821};
//     raw_points_[7] = {2.258523535, 4.554377368};
//     raw_points_[8] = {1.562447892, 4.656801472};
//     raw_points_[9] = {0.8764776599, 4.971705856};
//     raw_points_[10] = {0.09899323097, 4.985845841};
//     raw_points_[11] = {-0.7132021974, 5.010851105};
//     raw_points_[12] = {-1.479055426, 4.680181989};
//     raw_points_[13] = {-2.170306775, 4.463442715};
//     raw_points_[14] = {-3.034455492, 4.074651273};
//     raw_points_[15] = {-3.621987909, 3.585790302};
//     raw_points_[16] = {-3.979289889, 3.014232351};
//     raw_points_[17] = {-4.434628966, 2.367848826};
//     raw_points_[18] = {-4.818245921, 1.467395733};
//     raw_points_[19] = {-4.860190444, 0.8444358019};
//     raw_points_[20] = {-5.09947597, -0.01022405467};
// }

void Turn_obstacles_into_squares(visualization_msgs::Marker &marker,  const MatrixXd& poly, int id)
{
  double Length =fabs(poly(0, 0) - poly(0, 1));
  double width = fabs(poly(1, 1) - poly(1, 2));
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id; // 注意了
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = fabs(poly(0, 0) + poly(0, 2)) / 2;
  marker.pose.position.y = fabs(poly(1, 0) + poly(1, 2)) / 2;
  cout << Length << " "<< width<< " " << marker.pose.position.x<< " " << marker.pose.position.y<< " " << endl;
  cout << fabs(poly(0, 0) - poly(0, 1)) << endl;
  marker.pose.position.z = 0;
  marker.scale.x = Length;
  marker.scale.y = width;
  marker.scale.z = 0.5;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  // marker.lifetime = ros::Duration();
}

int main(int argc, char * argv[])
{
	setlocale(LC_ALL,"");//防止乱码
  //2.初始化 ROS 节点  
    ros::init(argc,argv,"cfstest");//定义了解
 
  // 3.创建节点句柄
    ros::NodeHandle nh;//相当于一个重命名 NodeHandle = nh
 
  //4.创建发布者对象
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);
 
  //5.编写发布逻辑并发布数据  
  //要求以10hz的频率发布，文本后添加编号
  //先创建被发布的消息
    std_msgs::String msg ;//创建一个名为msg，在std_msgs/String里
    msg.data = "hello";
  //发布频率
    ros::Rate rate(2.5);
  //设置编号

  /*************sccfs测试   ***************/
  /*
    int count = 0;//定义一个int数据类型的count
    auto path_generator = SCCFS();
    path_generator.InitPath();
    //auto path = path_generator.solveCFS();
    */


  /*Rviz打印静态障碍测试*/
  /*
  // //发布的消息
  //   int ind = 0;
  //     ros::Publisher obs_pub = nh.advertise<visualization_msgs::MarkerArray>("obs_polygons", 10);
  //     visualization_msgs::MarkerArray obstacle_MarkerArray;
  //     for (int j = 0; j <  path_generator.obs.getNum()-1; ++j) {
  //       visualization_msgs::Marker marker;
  //       auto cur_obs = path_generator.obs.getObs()[j];
  //       MatrixXd poly(2, cur_obs.vertex_x.size());
  //       for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
  //         poly(0, i) = cur_obs.vertex_x[i];
  //         poly(1, i) = cur_obs.vertex_y[i];
  //       }
  //       Turn_obstacles_into_squares(marker, poly,ind);
  //       ind++;
  //       obstacle_MarkerArray.markers.push_back(marker);
  //       //obs_pub.publish(obstacle_MarkerArray);
  //     }
  */

  // /*全局优化测试*/
  //   SmoothConfig config_;
  //   std::vector<Eigen::Vector2d> raw_points_;
  //   config_.max_deviation = 0.1;
  //   config_.max_iteration = (300);
  //   config_.opt_tol = (1.0e-6);
  //   config_.opt_acceptable_tol = (1.0e-4);
  //   config_.weight_curve_length = (1.0);
  //   config_.weight_kappa = (1.0);
  //   config_.weight_dkappa = (100.0);
  //   config_.resolution = 0.1;

  //   // 打开输入文件流
  //   std::ifstream inFile("globalpoints_data.txt");
 
  //   // 检查文件是否成功打开
  //   if (!inFile) {
  //       std::cerr << "无法打开文件！" << std::endl;
  //       return 1;
  //   }
 
  //   std::string line;
  //   // 逐行读取文件内容
  //   while (std::getline(inFile, line)) {
  //       // 使用 std::stringstream 来分割每行的数据
  //       std::stringstream ss(line);
  //       double x, y;
        
  //       // 从字符串流中读取 x 和 y 坐标
  //       if (ss >> x >> y) {
  //           // 将读取到的坐标存储到 Eigen::Vector2d 中，并添加到 vector 中
  //          raw_points_.emplace_back(x, y);
  //       } else {
  //           std::cerr << "文件格式错误，无法读取坐标：" << line << std::endl;
  //       }
  //   }
 
  //   // 关闭文件流
  //   inFile.close();
   
  //   std::vector<double> theta;
  //   std::vector<double> kappa;
  //   std::vector<double> dkappa;
  //   std::vector<double> s;
  //   std::vector<double> x;
  //   std::vector<double> y;

  //   CoarsePathGenerator spiral_smoother(config_);
  //   int res = spiral_smoother.SmoothStandAlone(raw_points_, &theta, &kappa,
  //                                             &dkappa, &s, &x, &y);
  //   std::vector<GlobalPathPoint> smoothed_point2d = spiral_smoother.Interpolate(theta, kappa, dkappa, s, x, y, config_.resolution);

  //   // 打开一个输出文件流
  //   std::ofstream outFile("output.txt");
 
  //   // 检查文件是否成功打开
  //   if (!outFile) {
  //       std::cerr << "无法打开文件！" << std::endl;
  //       return 1;
  //   }
 
  //   // 遍历 vector 并写入文件
  //   for (int i = 0; i < x.size(); ++i) {
  //       outFile << x[i] << " " << y[i] << std::endl;
  //   }
 
  //   // 关闭文件流
  //   outFile.close();
 
  //   std::cout << "数据已成功写入 output.txt 文件。" << std::endl;

  //   // 打开一个输出文件流
  //   std::ofstream outFile3("inter_points.txt");
 
  //   // 检查文件是否成功打开
  //   if (!outFile3) {
  //       std::cerr << "无法打开文件！" << std::endl;
  //       return 1;
  //   }
 
  //   // 遍历 vector 并写入文件
  //   for (const auto& v : smoothed_point2d) {
  //       outFile3 << v.x << " " << v.y << std::endl;
  //   }
 
  //   // 关闭文件流
  //   outFile3.close();
 
  //   std::cout << "数据已成功写入 output.txt 文件。" << std::endl;

  //   // 打开一个输出文件流
  //   std::ofstream outFile2("output2.txt");
 
  //   // 检查文件是否成功打开
  //   if (!outFile2) {
  //       std::cerr << "无法打开文件！" << std::endl;
  //       return 1;
  //   }
 
  //   // 遍历 vector 并写入文件
  //   for (const auto& v : raw_points_) {
  //       outFile2 << v(0) << " " << v(1) << std::endl;
  //   }
 
  //   // 关闭文件流
  //   outFile2.close();
 
  //   std::cout << "数据已成功写入 output.txt 文件。" << std::endl;
  
/**/


  //编写循环，循环中发布数据
    while (ros::ok())
    {
       /*Rviz打印动态障碍测试*/
       /*
    //  if(obstacle_MarkerArray.markers.size() > 4 )
    //     obstacle_MarkerArray.markers.pop_back();

    //   visualization_msgs::Marker marker;
    //   auto cur_obs = path_generator.obs.getObs()[4];
    //   MatrixXd poly(2, cur_obs.vertex_x.size());
    //   for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
    //     poly(0, i) = cur_obs.vertex_x[i];
    //     poly(1, i) = cur_obs.vertex_y[i];
    //   }
    //   poly += cur_obs.speed * MatrixXd::Ones(1, 4) * 0.4 * count; 
    //   Turn_obstacles_into_squares(marker, poly,ind);
    //   obstacle_MarkerArray.markers.push_back(marker);
    //   obs_pub.publish(obstacle_MarkerArray);

    //     count++;
    //     if (count > 99)  {
    //       cout << "max_iter" << endl;
    //       break;
    //       }
    //     pub.publish(msg);//用这个发布者对象发布消息，看定义，也和我们之前的创建发布者那边有关，他只能发布string的
    */
        //添加日志
        ROS_INFO("发布的数据是:%s", msg.data.c_str());//%s表示字符串
        rate.sleep();
    }
    
    return 0;
}