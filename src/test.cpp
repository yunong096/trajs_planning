#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <Obstacles.hpp>
//using Eigen::MatrixXd;

 

#include <iostream>
// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>


 
using namespace Eigen;
// using namespace std;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
 
using namespace std;
Eigen::SparseMatrix<double> hessian;
Eigen::VectorXd gradient;
Eigen::SparseMatrix<double> linearMatrix;
Eigen::VectorXd lowerBound;
Eigen::VectorXd upperBound;
 
int initMat(OsqpEigen::Solver& solver)
{
    unsigned int numOfVar = 3;
    unsigned int numOfCons = 4;
    solver.data()->setNumberOfVariables(numOfVar);
    solver.data()->setNumberOfConstraints(numOfCons);
 
    hessian.resize(numOfVar, numOfVar);
    gradient.resize(numOfVar);
    linearMatrix.resize(numOfCons, numOfVar);
    lowerBound.resize(numOfCons);
    upperBound.resize(numOfCons);
 
 
    hessian.insert(0, 0) = 1;
    hessian.insert(0, 1) = -1;
    hessian.insert(0, 2) = 1;
    hessian.insert(1, 0) = -1;
    hessian.insert(1, 1) = 2;
    hessian.insert(1, 2) = -2;
    hessian.insert(2, 0) = 1;
    hessian.insert(2, 1) = -2;
    hessian.insert(2, 2) = 4;
   //cout << "hessian" << hessian << endl;
   /* hessian << 1, -1, 1,
              -1, 2, -2,
               1, -2, 4;*/
 
    gradient << 2, -3, 1;
    
    linearMatrix.insert(0, 0) = 1;
    linearMatrix.insert(1, 1) = 1;
    linearMatrix.insert(2, 2) = 1;
    linearMatrix.insert(3, 0) = 1;
    linearMatrix.insert(3, 1) = 1;
    linearMatrix.insert(3, 2) = 1;
    /*linearMatrix << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1,
                    1, 1, 1;*/
 
    lowerBound << 0, 0, 0, 0.5;
    upperBound << 1, 1, 1, 0.5;
 
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
 
    return true;
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
  


  //编写循环，循环中发布数据
    while (ros::ok())
    {
           OsqpEigen::Solver solver;
 
    // set the solver
    solver.settings()->setWarmStart(true);
 
    // instantiate the solver
    if (initMat(solver)) {
        if (!solver.initSolver()) return 1;
    }
    else {
        cout << "initilize QP solver failed" << endl;
        return 1;
    }
 
    // solve
    solver.solve();
 
    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();
    
    cout << "x1 = " << QPSolution[0] << endl
        << "x2 = " << QPSolution[1] << endl
        << "x3 = " << QPSolution[2] << endl;
        //添加日志
        ROS_INFO("发布的数据是:%s", msg.data.c_str());//%s表示字符串
        rate.sleep();
    }
    
    return 0;
}