/*该代码为ROKAE官方示例程序*/
#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

/**
 * @brief 主程序 - 机器人笛卡尔阻抗控制示例
 * 该程序演示了如何使用rokae机器人SDK实现笛卡尔空间阻抗控制，
 * 使机器人在保持一定接触力的情况下沿特定轨迹运动
 */
int main() {
  using namespace std;
  try {
    // ==================== 机器人初始化配置 ====================
    std::string ip = "192.168.21.10";
    std::error_code ec;    
    // 创建xMate 7轴机器人实例
    // 参数1: 机器人IP地址，参数2: 本地IP
    rokae::xMateErProRobot robot(ip, "192.168.21.150");    
    // 设置实时网络容差为50ms，确保实时控制的稳定性
    robot.setRtNetworkTolerance(50, ec);    
    // 设置机器人操作模式为自动模式
    robot.setOperateMode(rokae::OperateMode::automatic, ec);    
    // 设置运动控制模式为实时命令模式，允许实时发送控制指令
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);    
    // 上电使能机器人
    robot.setPowerState(true, ec);
    // 获取实时运动控制器
    auto rtCon = robot.getRtMotionController().lock();
    // ==================== 初始位置运动 ====================
    // 定义目标关节角度 (弧度)，这个需要根据当前机型的设置关节角度
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};    
    // 关节空间运动到初始位置，速度比例0.5
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag_xm7p);
    // ==================== 力控参数设置 ====================
    // 设置力控坐标系为工具坐标系
    // 工具坐标系相对于法兰坐标系的变换矩阵
    // 这个矩阵表示：Z轴指向法兰外侧，Y轴指向法兰上方
    std::array<double, 16> toolToFlange = {
        0, 0, 1, 0,    // X轴在基坐标系中的方向
        0, 1, 0, 0,    // Y轴在基坐标系中的方向  
        -1, 0, 0, 0,   // Z轴在基坐标系中的方向
        0, 0, 0, 1     // 位置坐标 (无偏移)
    };
    rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);    
    // 设置笛卡尔阻抗系数
    // 参数顺序: {X方向刚度, Y方向刚度, Z方向刚度, Rx方向刚度, Ry方向刚度, Rz方向刚度}
    // 单位: 刚度(N/m), 旋转刚度(Nm/rad)
    // 这里X和Y方向设置较高刚度(1200 N/m)，Z方向设置为0以便力控
    rtCon->setCartesianImpedance({1200, 1200, 0, 100, 100, 0}, ec);    
    // 设置期望的接触力/力矩
    // 参数顺序: {Fx, Fy, Fz, Mx, My, Mz}
    // 这里设置X方向3N力，Z方向3N力，其他方向为0
    rtCon->setCartesianImpedanceDesiredTorque({3, 0, 3, 0, 0, 0}, ec);
    // ==================== 获取初始位姿 ====================
    // 获取机器人末端在基坐标系中的当前位姿
    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);
    // ==================== 开始阻抗控制 ====================
    // 启动笛卡尔阻抗控制模式
    rtCon->startMove(RtControllerMode::cartesianImpedance);
    // ==================== 实时控制循环设置 ====================
    double time = 0; // 时间计数器
    std::atomic<bool> stopManually {true}; // 线程安全的停止标志    
    /**
     * @brief 实时控制回调函数
     * 这个函数每1ms被调用一次，生成机器人的目标轨迹
     */
    std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
      time += 0.001; // 每次调用增加1ms      
      // 轨迹参数
      constexpr double kRadius = 0.2; // 运动半径
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time)); // 平滑的角度变化
      double delta_z = kRadius * (std::cos(angle) - 1); // Z轴位置变化量      
      // 创建输出位置指令
      CartesianPosition output{};
      output.pos = init_position; // 保持初始位置和姿态
      output.pos[7] += delta_z;   // 仅修改Z轴位置 (变换矩阵中的第8个元素)      
      // 运动结束条件：40秒后停止
      if(time > 40){
        std::cout << "运动结束" << std::endl;
        output.setFinished();     // 标记运动完成
        stopManually.store(false); // 通知主线程可以停止
      }
      return output;
    };
    
    // ==================== 启动控制循环 ====================
    // 设置控制回调函数
    rtCon->setControlLoop(callback);    
    // 启动实时控制循环，false表示非阻塞模式
    rtCon->startLoop(false);    
    // ==================== 等待运动完成 ====================
    // 主线程等待，直到控制循环设置停止标志
    while(stopManually.load());    
    // 停止控制循环
    rtCon->stopLoop();    
    // 等待2秒确保完全停止
    std::this_thread::sleep_for(std::chrono::seconds(2));    
  } catch (const std::exception &e) {
    // 异常处理
    std::cerr << "程序异常: " << e.what() << std::endl;
  }
  return 0;
}