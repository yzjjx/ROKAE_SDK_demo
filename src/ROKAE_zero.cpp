/*该代码用于回到机器人零位*/
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "rokae/robot.h"

using namespace rokae;

int main()
{
    try{
        // ======================机器人连接配置==================
        std::string ip = "192.168.2.160";
        std::string local_ip = "192.168.2.2";
        // 错误码
        std::error_code ec;   
        rokae::xMateRobot robot(ip,local_ip);
        std::cout<<"机器人连接成功"<<std::endl;

        // 设置机器人操作模式为自动模式
        robot.setOperateMode(rokae::OperateMode::automatic,ec);

        // 机器人上电
        robot.setPowerState(true,ec);

        // 设置运动控制模式为非实时运动模式
        robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);

        // 发送指令之前清除缓存
        robot.moveReset(ec);

        // 选择到达指定关节角度的速度(50%)
        robot.setDefaultSpeed(500,ec);

        // 选择默认转弯区
        // 转弯区：机器人在到达某个目标点附近时，
        // 允不允许“不完全停准这个点”，而是提前拐过去继续走下一个点的容差范围。
        robot.setDefaultZone(0,ec);

        // ========= 给定目标关节角度 ===========
        rokae::MoveJCommand move_q({
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        });

        // ================= 运动执行 =================
        robot.executeCommand({move_q},ec);
        std::cout << "已发送关节运动指令，机器人开始运动..." << std::endl;

        // 等待机器人运动到指定位置
        while (true)
        {
            auto state = robot.operationState(ec);

            if (state == rokae::OperationState::idle)
            {
                std::cout << "机器人已停止运动" << std::endl;
                break;
            }
        }

        // 读回当前关节角，确认是否接近零位
        auto q_now = robot.jointPos(ec);
        std::cout << "当前关节角(rad): ";
        for (double q : q_now)
        {
            std::cout << q << " ";
        }
        std::cout << std::endl;

        // 机器人下电
        robot.setPowerState(false, ec);
    }
    catch (const std::exception& e)
    {
        std::cerr<<"连接失败"<<e.what()<<std::endl;
        return -1;
    }
    return 0;
}