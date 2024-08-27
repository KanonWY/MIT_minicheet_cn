/*!
 * @file RobotInterface.h
 * @brief Interface between simulator and hardware using LCM.
 */

#ifndef PROJECT_ROBOTINTERFACE_H
#define PROJECT_ROBOTINTERFACE_H

#include "Graphics3D.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include <ControlParameters/RobotParameters.h>
#include <Dynamics/Quadruped.h>
#include <Utilities/PeriodicTask.h>
#include <cheetah_visualization_lcmt.hpp>
#include <condition_variable>
#include <lcm-cpp.hpp>
#include <mutex>
#include <thread>

#define ROBOT_INTERFACE_UPDATE_PERIOD (1.f / 60.f)
#define INTERFACE_LCM_NAME            "interface"
#define TIMES_TO_RESEND_CONTROL_PARAM 5

/**
 * @brief 机器人接口类
 * @note 继承自周期性的任务类
 */
class RobotInterface : PeriodicTask
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotInterface(RobotType robotType, Graphics3D* gfx, PeriodicTaskManager* tm, ControlParameters& userParameters);

    /**
     * @brief 获取机器人控制参数
     */
    RobotControlParameters& getParams()
    {
        return _controlParameters;
    }
    void startInterface();
    void stopInterface();
    void lcmHandler();

    /**
     * @brief 发送控制参数
     */
    void sendControlParameter(const std::string& name, ControlParameterValue value, ControlParameterValueKind kind, bool isUser);

    /**
     * @brief 处理控制参数
     */
    void handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_respones_lcmt* msg);

    /**
     * @brief 处理可视化数据
     */
    void handleVisualizationData(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const cheetah_visualization_lcmt* msg);

    void init() override {}

    /**
     * @brief 继承自周期性的任务，将会在每一帧运行该函数
     */
    void run() override;

    void cleanup() override {}

    virtual ~RobotInterface()
    {
        delete _simulator;
        stop();
    }

private:
    PeriodicTaskManager            _taskManager;             ///<<< 周期性的任务管理器
    gamepad_lcmt                   _gamepad_lcmt;            ///<<< 游戏手柄的 lcm msg
    control_parameter_request_lcmt _parameter_request_lcmt;  ///<<< 控制参数请求的 lcm msg
    bool                           _pendingControlParameterSend = false;
    lcm::LCM                       _lcm;                ///<<< LCM 句柄
    uint64_t                       _robotID;            ///<<< 机器人 ID
    std::thread                    _lcmThread;          ///<<< LCM 线程
    VisualizationData              _visualizationData;  ///<<< 可视化数据
    RobotControlParameters         _controlParameters;  ///<<< 机器人控制参数
    ControlParameters&             _userParameters;     ///<<< 控制参数
    Graphics3D*                    _gfx;                ///<<< 3D 渲染对象
    RobotType                      _robotType;          ///<<< 机器人类型
    bool                           _running = false;

    std::mutex              _lcmMutex;
    std::condition_variable _lcmCV;
    bool                    _waitingForLcmResponse = false;
    bool                    _lcmResponseBad        = true;

    // forward kinematics，向前动力学
    Quadruped<double>          _quadruped;
    FloatingBaseModel<double>  _model;
    DynamicsSimulator<double>* _simulator = nullptr;
    FBModelState<double>       _fwdKinState;
};

#endif  // PROJECT_ROBOTINTERFACE_H
