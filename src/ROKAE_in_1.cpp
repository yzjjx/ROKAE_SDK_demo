#include <iostream>
#include <thread>
#include <chrono>
#include <system_error>
#include "rokae/robot.h"

using namespace rokae;

static bool checkEc(const std::string& step, const std::error_code& ec)
{
    if (ec)
    {
        std::cerr << "[失败] " << step
                  << " | code = " << ec.value()
                  << " | msg = " << ec.message() << std::endl;
        return false;
    }
    std::cout << "[成功] " << step << std::endl;
    return true;
}

int main()
{
    try
    {
        std::string ip = "192.168.2.160";
        std::error_code ec;

        rokae::xMateRobot robot(ip);
        std::cout << "机器人连接成功" << std::endl;

        ec.clear();
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        if (!checkEc("切换自动模式", ec)) return -1;

        ec.clear();
        robot.setPowerState(true, ec);
        if (!checkEc("机器人上电", ec)) return -1;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        ec.clear();
        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        if (!checkEc("切换非实时模式", ec)) return -1;

        ec.clear();
        robot.moveReset(ec);
        if (!checkEc("moveReset", ec)) return -1;

        ec.clear();
        robot.setDefaultSpeed(200, ec);
        if (!checkEc("设置默认速度", ec)) return -1;

        ec.clear();
        robot.setDefaultZone(0, ec);
        if (!checkEc("设置默认转弯区", ec)) return -1;

        // 这里必须用 MoveAbsJCommand
        rokae::MoveAbsJCommand move_q({
            0.0,
            M_PI / 3.0,
            0.0,
            M_PI / 3.0,
            0.0,
            M_PI / 2.0
        });

        ec.clear();
        robot.executeCommand({move_q}, ec);
        if (!checkEc("发送 MoveAbsJ 指令", ec)) return -1;

        std::cout << "已发送关节运动指令" << std::endl;

        bool started = false;
        auto t0 = std::chrono::steady_clock::now();

        while (true)
        {
            ec.clear();
            auto state = robot.operationState(ec);
            if (!checkEc("查询 operationState", ec)) return -1;

            if (state == rokae::OperationState::moving ||
                state == rokae::OperationState::jogging)
            {
                started = true;
            }

            if (started && state == rokae::OperationState::idle)
            {
                std::cout << "机器人已到达目标位置" << std::endl;
                break;
            }

            if (!started &&
                std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5))
            {
                std::cerr << "5秒内未进入 moving，说明控制器未真正开始执行该指令。" << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 调试时先不要自动下电
        // ec.clear();
        // robot.setPowerState(false, ec);
        // checkEc("机器人下电", ec);
    }
    catch (const std::exception& e)
    {
        std::cerr << "异常: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}