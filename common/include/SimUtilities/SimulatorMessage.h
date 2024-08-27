/*! @file SimulatorMessage.h
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory.   This is basically everything except
 * for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "ControlParameters/ControlParameterInterface.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"

/*!
 * The mode for the simulator
 */

/**
 * @brief 模拟器的模式
 */
enum class SimulatorMode
{
    RUN_CONTROL_PARAMETERS,  // don't run the robot controller, just process
                             // Control Parameters
    RUN_CONTROLLER,          // run the robot controller
    DO_NOTHING,              // just to check connection
    EXIT                     // quit!
};

/*!
 * A plain message from the simulator to the robot
 */

/**
 * @brief 模拟器到机器人的消息
 */
struct SimulatorToRobotMessage
{
    GamepadCommand gamepadCommand;  // joystick
    RobotType      robotType;       // which robot the simulator thinks we are simulating

    // imu data
    VectorNavData        vectorNav;
    CheaterState<double> cheaterState;

    // leg data
    SpiData     spiData;
    TiBoardData tiBoardData[4];
    // todo cheetah 3
    ControlParameterRequest controlParameterRequest;

    SimulatorMode mode;
};

/*!
 * A plain message from the robot to the simulator
 */

/**
 * @brief 从机器人到模拟器的消息
 */
struct RobotToSimulatorMessage
{
    RobotType      robotType;
    SpiCommand     spiCommand;
    TiBoardCommand tiBoardCommand[4];

    VisualizationData        visualizationData;
    CheetahVisualization     mainCheetahVisualization;
    ControlParameterResponse controlParameterResponse;

    char errorMessage[2056];
};

/*!
 * All the data shared between the robot and the simulator
 */
/**
 * @brief 机器人与模拟器之间的消息
 */
struct SimulatorMessage
{
    RobotToSimulatorMessage robotToSim;
    SimulatorToRobotMessage simToRobot;
};

/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by
 * both the simulator and the robot The simulator and robot take turns have
 * exclusive access to the entire message. The intended sequence is:
 *  - robot: waitForSimulator()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do
 * anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do
 * anything)
 *  - robot: robotDone();
 *  - robot: waitForSimulator()
 *  ...
 */

/**
 * @brief 模拟器和机器人之间的共享消息
 * @note 内部使用信号量来保证读写次序
 */
struct SimulatorSyncronizedMessage : public SimulatorMessage
{

    /*!
     * The init() method should only be called *after* shared memory is connected!
     * This initializes the shared memory semaphores used to keep things in sync
     */
    void init()
    {
        robotToSimSemaphore.init(0);
        simToRobotSemaphore.init(0);
    }

    /**
     * @brief 等待模拟器响应
     */
    void waitForSimulator()
    {
        simToRobotSemaphore.decrement();
    }

    /*!
     * Simulator signals that it is done
     */

    /**
     * @brief 模拟器发出信号表示自己已经处理完毕
     */
    void simulatorIsDone()
    {
        simToRobotSemaphore.increment();
    }

    /*!
     * Wait for the robot to finish
     */

    /**
     * @brief 等待机器人发出结束消息
     */
    void waitForRobot()
    {
        robotToSimSemaphore.decrement();
    }

    /*!
     * Check if the robot is done
     * @return if the robot is done
     */

    /**
     * @brief 检测机器人是否处理完毕
     */
    bool tryWaitForRobot()
    {
        return robotToSimSemaphore.tryDecrement();
    }

    /*!
     * Wait for the robot to finish with a timeout
     * @return if we finished before timing out
     */

    /**
     * @brief 在超时时间内等到机器人结束
     */
    bool waitForRobotWithTimeout()
    {
        return robotToSimSemaphore.decrementTimeout(1, 0);
    }

    /*!
     * Signal that the robot is done
     */

    /**
     * @brief 发出信号表示机器人结束
     */
    void robotIsDone()
    {
        robotToSimSemaphore.increment();
    }

private:
    SharedMemorySemaphore robotToSimSemaphore, simToRobotSemaphore;  ///<<< 机器人和模拟器的信号量
};

#endif  // PROJECT_SIMULATORTOROBOTMESSAGE_H
