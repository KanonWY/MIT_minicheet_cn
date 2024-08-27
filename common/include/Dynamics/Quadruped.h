/*! @file Quadruped.h
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for
 * a quadruped robot.  There are utility functions to generate Quadruped objects
 * for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the quadruped.
 */

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include "Dynamics/ActuatorModel.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/SpatialInertia.h"
#include <vector>

#include <eigen3/Eigen/StdVector>


/*!
 * Basic parameters for a cheetah-shaped robot
 */
namespace cheetah
{
    constexpr size_t num_act_joint = 12;
    constexpr size_t num_q         = 19;
    constexpr size_t dim_config    = 18;
    constexpr size_t num_leg       = 4;
    constexpr size_t num_leg_joint = 3;
}  // namespace cheetah

/*!
 * Link indices for cheetah-shaped robots
 */
namespace linkID
{
    constexpr size_t FR = 9;   // Front Right Foot
    constexpr size_t FL = 11;  // Front Left Foot
    constexpr size_t HR = 13;  // Hind Right Foot
    constexpr size_t HL = 15;  // Hind Left Foot

    constexpr size_t FR_abd = 2;  // Front Right Abduction
    constexpr size_t FL_abd = 0;  // Front Left Abduction
    constexpr size_t HR_abd = 3;  // Hind Right Abduction
    constexpr size_t HL_abd = 1;  // Hind Left Abduction
}  // namespace linkID

using std::vector;

/*!
 * Representation of a quadruped robot's physical properties.
 *
 * When viewed from the top, the quadruped's legs are:
 *
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 *
 */

/**
 * @brief 四足机器人的物理特性表示
 * @details 相关参数可以参考这个：https://blog.csdn.net/weixin_42208705/article/details/125922100
 *
 * @note 牛顿欧拉法
 */
template <typename T>
class Quadruped
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotType _robotType;   ///<<< 机器人类
    T         _bodyLength;  ///<<< 身体长: 前后两个hip电机的中心距离WW
    T         _bodyWidth;   ///<<< 宽
    T         _bodyHeight;  ///<<< 高: 左右两个abad电机的中心距离
    T         _bodyMass;    ///<<< 机身质量

    T _abadGearRatio;  ///<<< 电机减速比
    T _hipGearRatio;   ///<<< 电机减速比
    T _kneeGearRatio;  ///<<< 电机与连杆的减速比

    T _abadLinkLength;  ///<<< abad电机中心到hip电机的中心距离
    T _hipLinkLength;   ///<<< 大腿连杆长度
    T _kneeLinkLength;  ///<<< 小腿连杆长度
    T _maxLegLength;    ///<<< 大腿+小腿的长度
    T _kneeLinkY_offset;

    T _motorKT;  ///<<< 电机的转距常量 KT = 转矩 / 电流
    T _motorR;
    T _batteryV;
    T _motorTauMax;  ///<<< 电机最大力矩
    T _jointDamping;
    T _jointDryFriction;

    // 关节惯性张量
    SpatialInertia<T> _abadInertia;  ///<<< a 关节惯性张量
    SpatialInertia<T> _hipInertia;   ///<<< h 关节惯性张量
    SpatialInertia<T> _kneeInertia;  ///<<< k 关节惯性张量

    // 电机
    SpatialInertia<T> _abadRotorInertia;  ///<<< a 关节电机转子惯性张量
    SpatialInertia<T> _hipRotorInertia;   ///<<< h 关节电机转子惯性张量
    SpatialInertia<T> _kneeRotorInertia;  ///<<< k 关节电机转子惯性张量

    SpatialInertia<T> _bodyInertia;  ///<<< 身体惯性张量

    Vec3<T> _abadLocation;       ///<<< a 关节位置
    Vec3<T> _abadRotorLocation;  ///<<< a 关节电机位置
    Vec3<T> _hipLocation;        ///<<< h 关节位置
    Vec3<T> _hipRotorLocation;   ///<<< h 关节电机位置
    Vec3<T> _kneeLocation;       ///<<< k 关节电机位置
    Vec3<T> _kneeRotorLocation;  ///<<< k 关节电机位置

    /**
     * @brief 构建浮动基模型
     */
    FloatingBaseModel<T> buildModel();

    /**
     * @brief 辅助函数，帮助构建浮动基模型
     */
    bool buildModel(FloatingBaseModel<T>& model);

    std::vector<ActuatorModel<T>> buildActuatorModels();

    /*!
     * Get if the i-th leg is on the left (+) or right (-) of the robot.
     * @param leg : the leg index
     * @return The side sign (-1 for right legs, +1 for left legs)
     */
    /**
     * @brief 获取 i-th leg 的位置
     * @note -1 是右腿，+1 是左腿
     */
    static T getSideSign(int leg)
    {
        const T sideSigns[4] = { -1, 1, -1, 1 };
        assert(leg >= 0 && leg < 4);
        return sideSigns[leg];
    }

    /*!
     * Get location of the hip for the given leg in robot frame
     * @param leg : the leg index
     */
    /**
     * @brief 获取机器人框架中给定腿的臀部位置
     */
    Vec3<T> getHipLocation(int leg)
    {
        assert(leg >= 0 && leg < 4);
        Vec3<T> pHip((leg == 0 || leg == 1) ? _abadLocation(0) : -_abadLocation(0),
            (leg == 1 || leg == 3) ? _abadLocation(1) : -_abadLocation(1), _abadLocation(2));
        return pHip;
    }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);

#endif  // LIBBIOMIMETICS_QUADRUPED_H
