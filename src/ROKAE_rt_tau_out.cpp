/*该代码用于实时模式输出力矩*/
/* 该代码用于：
 * 1. 从 txt 读取关节轨迹
 * 2. 用实时模式下发关节位置
 * 3. 同步记录实时状态到 txt：
 *    - q_ref              : 下发的参考关节角
 *    - q_m                : 实际关节角
 *    - q_c                : 控制器指令关节角
 *    - tau_m              : 关节力矩
 *    - tau_filtered_m     : 滤波后关节力矩
 *    - motor_tau          : 电机转矩
 *    - motor_tau_filtered : 滤波后电机转矩
 */

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
#include <filesystem>

#include "rokae/robot.h"

using namespace rokae;

//====================== 读取轨迹 txt ======================
std::vector<std::array<double, 6>> readJointTxt(const std::string& filename)
{
    std::vector<std::array<double, 6>> points;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("无法打开轨迹文件: " + filename);
    }

    std::string line;
    int line_no = 0;

    while (std::getline(file, line))
    {
        ++line_no;

        if (line.empty())
            continue;

        for (char& ch : line)
        {
            if (ch == ',' || ch == '\t')
                ch = ' ';
        }

        std::stringstream ss(line);
        std::array<double, 6> q{};

        if (!(ss >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5]))
        {
            std::cerr << "第 " << line_no << " 行格式错误，已跳过: " << line << std::endl;
            continue;
        }

        points.push_back(q);
    }

    if (points.empty())
    {
        throw std::runtime_error("轨迹文件中没有有效数据");
    }

    return points;
}

//====================== 日志结构体 ======================
struct LogRow
{
    std::size_t index = 0;
    double time_s = 0.0;

    std::array<double, 6> q_ref{};
    std::array<double, 6> q_m{};
    std::array<double, 6> q_c{};
    std::array<double, 6> tau_m{};
    std::array<double, 6> tau_filtered_m{};
    std::array<double, 6> motor_tau{};
    std::array<double, 6> motor_tau_filtered{};
};

//====================== 写 txt 文件 ======================
void writeLogTxt(const std::string& filename, const std::vector<LogRow>& logs)
{
    std::filesystem::path out_path(filename);
    if (out_path.has_parent_path())
    {
        std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream ofs(filename);
    if (!ofs.is_open())
    {
        throw std::runtime_error("无法创建输出文件: " + filename);
    }

    // 表头
    ofs << "index,time_s,"
        << "qref1,qref2,qref3,qref4,qref5,qref6,"
        << "qm1,qm2,qm3,qm4,qm5,qm6,"
        << "qc1,qc2,qc3,qc4,qc5,qc6,"
        << "tau_m1,tau_m2,tau_m3,tau_m4,tau_m5,tau_m6,"
        << "tau_filtered_m1,tau_filtered_m2,tau_filtered_m3,tau_filtered_m4,tau_filtered_m5,tau_filtered_m6,"
        << "motor_tau1,motor_tau2,motor_tau3,motor_tau4,motor_tau5,motor_tau6,"
        << "motor_tau_filtered1,motor_tau_filtered2,motor_tau_filtered3,motor_tau_filtered4,motor_tau_filtered5,motor_tau_filtered6\n";

    for (const auto& row : logs)
    {
        ofs << row.index << "," << row.time_s;

        for (double v : row.q_ref) ofs << "," << v;
        for (double v : row.q_m) ofs << "," << v;
        for (double v : row.q_c) ofs << "," << v;
        for (double v : row.tau_m) ofs << "," << v;
        for (double v : row.tau_filtered_m) ofs << "," << v;
        for (double v : row.motor_tau) ofs << "," << v;
        for (double v : row.motor_tau_filtered) ofs << "," << v;

        ofs << "\n";
    }

    ofs.close();
}

//====================== 主函数 ======================
int main()
{
    //================== 输入参数 ==================
    const std::string robot_ip   = "192.168.21.10";
    const std::string local_ip   = "192.168.21.150";
    const std::string input_q    = "../data_in/q.txt";
    const std::string output_txt = "../data_out/rt_torque_log.txt";

    std::error_code ec;
    rokae::xMateRobot SDU_SR4;

    std::vector<LogRow> logs;   // 提前定义，异常时也能尽量保留已采样数据

    try
    {
        // 1) 读取轨迹
        std::vector<std::array<double, 6>> traj = readJointTxt(input_q);
        std::cout << "成功读取轨迹点数: " << traj.size() << std::endl;

        logs.reserve(traj.size());

        // 2) 连接机器人
        SDU_SR4.connectToRobot(robot_ip, local_ip);
        std::cout << "机器人连接成功" << std::endl;

        // 3) 自动模式
        SDU_SR4.setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec)
        {
            std::cerr << "setOperateMode失败: " << ec.message() << std::endl;
            return -1;
        }

        // 4) 实时模式
        SDU_SR4.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec)
        {
            std::cerr << "setMotionControlMode失败: " << ec.message() << std::endl;
            return -1;
        }

        // 5) 网络实时容差
        SDU_SR4.setRtNetworkTolerance(20, ec);
        if (ec)
        {
            std::cerr << "setRtNetworkTolerance失败: " << ec.message() << std::endl;
            return -1;
        }

        // 6) 上电
        SDU_SR4.setPowerState(true, ec);
        if (ec)
        {
            std::cerr << "setPowerState失败: " << ec.message() << std::endl;
            return -1;
        }

        // 7) 获取实时控制器
        auto rtCon = SDU_SR4.getRtMotionController().lock();
        if (!rtCon)
        {
            std::cerr << "获取实时控制器失败" << std::endl;
            return -1;
        }

        // 8) 开始接收 1ms 实时状态
        //    这里把需要记录的字段一次性都打开
        SDU_SR4.startReceiveRobotState(
            std::chrono::milliseconds(1),
            {
                rokae::RtSupportedFields::jointPos_m,
                rokae::RtSupportedFields::jointPos_c,
                rokae::RtSupportedFields::tau_m,
                rokae::RtSupportedFields::tauFiltered_m,
                rokae::RtSupportedFields::motorTau,
                rokae::RtSupportedFields::motorTauFiltered
            }
        );

        // 9) 读取当前关节角
        auto q_current = SDU_SR4.jointPos(ec);
        if (ec)
        {
            std::cerr << "读取当前关节角失败: " << ec.message() << std::endl;
            return -1;
        }

        // 10) MoveJ 到零位
        std::cout << "MoveJ到机器人零位..." << std::endl;
        std::array<double, 6> q_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtCon->MoveJ(0.2, q_current, q_zero);

        // 11) MoveJ 到轨迹起点
        std::cout << "MoveJ到轨迹起点..." << std::endl;
        rtCon->MoveJ(0.2, q_zero, traj.front());

        // 稳定一下
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 12) 轨迹索引
        std::size_t index = 0;

        // 13) 实时回调
        std::function<rokae::JointPosition()> callback = [&]() -> rokae::JointPosition
        {
            rokae::JointPosition cmd(6);

            const auto& q_ref = (index < traj.size()) ? traj[index] : traj.back();

            for (std::size_t i = 0; i < 6; ++i)
            {
                cmd.joints[i] = q_ref[i];
            }

            // 读取当前周期的实时状态并记录
            LogRow row;
            row.index  = index;
            row.time_s = static_cast<double>(index) * 0.001;  // 按1ms周期记录
            row.q_ref  = q_ref;

            // 这些 getStateData 在 setControlLoop(..., true) 后，
            // 每次回调前会自动刷新到最新状态
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::jointPos_m, row.q_m);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::jointPos_c, row.q_c);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::tau_m, row.tau_m);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::tauFiltered_m, row.tau_filtered_m);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::motorTau, row.motor_tau);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::motorTauFiltered, row.motor_tau_filtered);

            logs.push_back(row);

            ++index;

            if (index >= traj.size())
            {
                cmd.setFinished();
            }

            return cmd;
        };

        // 14) 注册控制回调
        //     第3个参数 true：每次回调前自动更新实时状态数据
        rtCon->setControlLoop(callback, 0, true);

        // 15) 开始实时关节位置控制
        std::cout << "===== 开始实时轨迹下发 =====" << std::endl;
        rtCon->startMove(rokae::RtControllerMode::jointPosition);

        // 阻塞直到轨迹执行结束
        rtCon->startLoop(true);

        std::cout << "轨迹执行结束" << std::endl;

        // 16) 停止接收实时状态
        SDU_SR4.stopReceiveRobotState();

        // 17) 写 txt 文件
        writeLogTxt(output_txt, logs);
        std::cout << "力矩日志已保存到: " << output_txt << std::endl;

        // 18) 可选：读一次单次关节力矩（非实时接口）
        auto tau_once = SDU_SR4.jointTorque(ec);
        if (!ec)
        {
            std::cout << "单次查询 jointTorque(ec): ";
            for (double v : tau_once)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "异常: " << e.what() << std::endl;

        // 尽量把已采样的数据保存下来
        try
        {
            if (!logs.empty())
            {
                writeLogTxt(output_txt, logs);
                std::cerr << "已保存部分日志到: " << output_txt << std::endl;
            }
        }
        catch (...)
        {
            std::cerr << "保存日志失败" << std::endl;
        }

        return -1;
    }

    return 0;
}