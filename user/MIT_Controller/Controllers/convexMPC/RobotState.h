#ifndef _RobotState
#define _RobotState

#include "common_types.h"
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"


/**
 * @brief 提供一个存储机器人状态信息和打印状态信息的对象
 */
class RobotState
{
public:
    void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
    // void compute_rotations();
    void print();

    Matrix<fpt, 3, 1> p;       ///<<< 机器人位置
    Matrix<fpt, 3, 1> v;       ///<<< 机器人速度
    Matrix<fpt, 3, 1> w;       ///<<< 机器人角速度
    Matrix<fpt, 3, 4> r_feet;  ///<<< 足端位置坐标矩阵
    Matrix<fpt, 3, 3> R;       ///<<< 旋转矩阵
    Matrix<fpt, 3, 3> R_yaw;   ///<<< 仅考虑偏航情况下的旋转矩阵
    Matrix<fpt, 3, 3> I_body;  ///<<< 机器人惯性矩阵
    Quaternionf       q;       ///<<< 四元数
    fpt               yaw;     ///<<< 偏航角
    fpt               m = 9;
    // fpt m = 50.236; //DH
    // private:
};
#endif
