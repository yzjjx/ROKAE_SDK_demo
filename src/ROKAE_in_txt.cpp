/*该代码用来输入txt关节角度，用实时模式进行*/
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

#include <vector>
#include <array>
#include <functional>
#include <stdexcept>
#include <system_error>

#include "rokae/robot.h"

using namespace rokae;

// 读取角度输入txt的函数,逗号分隔符
std::vector<std::array<double, 6>> readJointTxt(const std::string& filename)
{
    // 例如：points[0]，第1行的6个关节值
    std::vector<std::array<double,6>> points;
    std::ifstream file(filename);

    if(!file.is_open())
    {
        throw std::runtime_error("无法打开轨迹文件");
    }

    // line：用来存每次读到的一整行文本
    // line_no：记录当前读到第几行，方便报错提示
    std::string line;
    int line_no = 0;
    // 只要能读到一整行，就一直循环while，循环将数据从txt加入points变量
    while(std::getline(file,line)){
        ++line_no;
        // 跳过空行
        // if(line.empty())
        //     continue;
        // 逗号分隔切换为空格，便于后面输入
        for (char& ch : line) {
            if (ch == ',' || ch == '\t') ch = ' ';
        }

        std::stringstream ss(line);
        std::array<double,6> q{};

        if (!(ss >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5])) {
            std::cerr << "第 " << line_no << " 行格式错误，已跳过: " << line << std::endl;
            continue;
        }
        //将这个关节加入到points的末尾
        points.push_back(q);
    }
    // 读取之后检查数据是否有效
    if (points.empty()) {
        throw std::runtime_error("轨迹文件中没有有效数据");
    }

    return points;
}

int main()
{
    //==================输入参数
    const std::string robot_ip = "192.168.21.10";
    const std::string local_ip = "192.168.21.150";
    const std::string input_q = "../data_in/q.txt";

    //
    std::error_code ec;
    rokae::xMateRobot SDU_SR4;

    try{
        // 将txt文件的参数一次性读进内存
        std::vector<std::array<double,6>> traj = readJointTxt(input_q);
        std::cout << "成功读取轨迹点数: " << traj.size() << std::endl;

        // 连接机器人
        SDU_SR4.connectToRobot(robot_ip,local_ip);
        std::cout << "机器人连接成功" << std::endl;

        // 设置为自动操作模式
        SDU_SR4.setOperateMode(rokae::OperateMode::automatic,ec);
        if (ec) {
            std::cerr << "setOperateMode失败: " << ec.message() << std::endl;
            return -1;
        }
        
        // 设置为实时模式
        SDU_SR4.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec) {
            std::cerr << "setMotionControlMode失败: " << ec.message() << std::endl;
            return -1;
        } 

        // 设置发送实时运动指令网络延迟阈值,超过阈值报错
        SDU_SR4.setRtNetworkTolerance(20, ec);

        // 上电
        SDU_SR4.setPowerState(true, ec);
        if (ec) {
            std::cerr << "setPowerState失败: " << ec.message() << std::endl;
            return -1;
        }

        // 实例化专门的实时通道
        auto rtCon = SDU_SR4.getRtMotionController().lock();
        if (!rtCon) {
            std::cerr << "获取实时控制器失败" << std::endl;
            return -1;
        }

        // 开始接收实时状态，周期1ms，获取关节位置
        SDU_SR4.startReceiveRobotState(std::chrono::milliseconds(1),
                                     {rokae::RtSupportedFields::jointPos_m});

        auto q_current = SDU_SR4.jointPos(ec);
        if (ec) {
            std::cerr << "读取当前关节角失败: " << ec.message() << std::endl;
            return -1;
        }

        // 移动到零点
        std::cout << "MoveJ到机器人零位" << std::endl;
        std::array<double, 6> q_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtCon->MoveJ(0.2, q_current,q_target);

        std::cout << "MoveJ到轨迹起点..." << std::endl;
        rtCon->MoveJ(0.2, q_current, traj.front());

        // 机器人零位稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 轨迹索引变量，表示当前发送到第几个位置
        std::size_t index = 0;

        // 定义回调函数，每一个实时周期调用该函数
        // 没有输入参数,返回一个 rokae::JointPosition
        // 每次调用它时，它都会返回一个“本周期要发送的关节位置命令”
        std::function<rokae::JointPosition()> callback = [&]() -> rokae::JointPosition
        {
            rokae::JointPosition cmd(6);   // 6轴

            const auto& q_ref = (index >= traj.size()) ? traj.back() : traj[index];

            for (size_t i = 0; i < 6; ++i) {
                cmd.joints[i] = q_ref[i];
            }

            if (index >= traj.size()) {
                cmd.setFinished();
                return cmd;
            }

            ++index;
            if (index >= traj.size()) {
                cmd.setFinished();
            }

            return cmd;
        };

        // 将回调函数注册给实时控制器rtcon
        rtCon -> setControlLoop(callback);

        // 开始实时关节位置控制，启动实时运动，并指定控制模式为关节位置模式
        std::cout << "=====开始实时轨迹下发=====" << std::endl;
        rtCon->startMove(rokae::RtControllerMode::jointPosition);

        // 阻塞直到结束，进入实时控制循环，第一次循环，调用一次 callback()，取 traj[0]
        rtCon->startLoop(true);

        std::cout << "轨迹执行结束" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}