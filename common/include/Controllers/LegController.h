/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "Dynamics/Quadruped.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "cppTypes.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"

/*!
 * Data sent from the control algorithm to the legs.
 * @note 中控程序生成的腿关节指令
 */
template <typename T>
struct LegControllerCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerCommand()
    {
        zero();
    }

    void zero();

    Vec3<T> tauFeedForward;    ///<<< 代表一条腿三个关节的前馈力矩
    Vec3<T> forceFeedForward;  ///<<< 三个关节的前馈力
    Vec3<T> qDes;  ///<<< 代表三个关节的期望位置或目标位置，三个关节为转动关节，因此就是旋转角度（θ）。
    Vec3<T> qdDes;        ///<<< 代表三个关节的期望角速度
    Vec3<T> pDes;         ///<<< 足端位置坐标
    Vec3<T> vDes;         ///<<< 足端位移矢量速度
    Mat3<T> kpCartesian;  ///<<< 四足机器人足端矢量位移变化与力的系数矩阵
    Mat3<T> kdCartesian;  ///<<< 四足机器人足端加速度与力的系数矩阵
    Mat3<T> kpJoint;      ///<<< 三个关节的kp值
    Mat3<T> kdJoint;      ///<<< 三个关节的kd值
};

/*!
 * Data returned from the legs to the control code.
 * @note 存储四足机器人各腿运行数据的数据结构
 */
template <typename T>
struct LegControllerData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData()
    {
        zero();
    }

    void setQuadruped(Quadruped<T>& quad)
    {
        quadruped = &quad;
    }

    void zero();

    Vec3<T>       q;            ///<<< 各关节的位置，旋转角度
    Vec3<T>       qd;           ///<<< 各关节的旋转速度，角速度
    Vec3<T>       p;            ///<<< 足端位置坐标
    Vec3<T>       v;            ///<<< 足端速度
    Mat3<T>       J;            ///<<< 当前雅可比矩阵
    Vec3<T>       tauEstimate;  ///<<< 当前各关节力矩预测值
    Quadruped<T>* quadruped;    ///<<< 当前四足机器人的机械模型对象
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 34
 * @note
 *  管理四足机器人腿控制命令和数据的类
 */
template <typename T>
class LegController
{
public:
    LegController(Quadruped<T>& quad)
        : _quadruped(quad)
    {
        for (auto& data : datas)
            data.setQuadruped(_quadruped);
    }

    void zeroCommand();
  
    /**
     * @brief 根据输入机器人的类型，
     * 对微分系数矩阵kdCartesian或kdJoint使用输入参数gain进行对角赋值
     */
    void edampCommand(RobotType robot, T gain);

    /**
     * @brief 将spiData和tiBoardData中的机器人各腿数据更新datas中的数据
     *    使用输入参数spiData中的成员对datas中的q和qd成员赋值
     *    调用函数computeLegJacobianAndPosition，计算四条腿各自的Jacobian矩阵以及足端位置坐标
		 *    通过正运动学方法计算足端的速度
     */
    void updateData(const SpiData* spiData);


    /**
     * @brief 使用输入参数tiBoardData中的成员对datas中的q、qd、p和v成员赋值
     *        调用函数computeLegJacobianAndPosition，计算四条腿各自的Jacobian矩阵以及足端位置坐标
     *        使用输入参数tiBoardData中的关节力矩成员tau对datas中的tauEstimate成员赋值
     */
    void updateData(const TiBoardData* tiBoardData);

    /**
     * @brief 将生成的控制命令写到spiCommand和tiBoardCommand对象
     */
    void updateCommand(SpiCommand* spiCommand);


    void updateCommand(TiBoardCommand* tiBoardCommand);
    void setEnabled(bool enabled)
    {
        _legsEnabled = enabled;
    };

    /**
     * @brief 将各腿的数据更新到data对象和控制命令更新到command对象。
     */
    void setLcm(leg_control_data_lcmt* data, leg_control_command_lcmt* command);

    /*!
     * Set the maximum torque.  This only works on cheetah 3!
     * @note 设置最大力矩
     */
    void setMaxTorqueCheetah3(T tau)
    {
        _maxTorque = tau;
    }

    LegControllerCommand<T> commands[4];                 ///<<< 存储四条腿的控制指令
    LegControllerData<T>    datas[4];                    ///<<< 存储四条腿的运行数据
    Quadruped<T>&           _quadruped;                  ///<<< 四足机器人的模型的引用
    bool                    _legsEnabled       = false;  ///<<< 四条腿的使能标志
    T                       _maxTorque         = 0;      ///<<< 四足机器人四条腿上各关节的最大力矩
    bool                    _zeroEncoders      = false;  ///<<< 零编码标志
    u32                     _calibrateEncoders = 0;      ///<<< 校准编码值
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J, Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
