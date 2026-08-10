/* 该代码用于：
 * 1. 从 txt 读取关节轨迹
 * 2. 用实时模式下发关节位置
 * 3. 标定关节力矩传感器
 * 4. 同步记录 q_m、q_dot_m、tau_m 到 txt
 * 5. 通过UDP把相同状态发送给MATLAB
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <stdexcept>
#include <system_error>
#include <filesystem>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rokae/robot.h"

using namespace rokae;

constexpr char UDP_LOCAL_IP[] = "192.168.2.2";  // 本机连接MATLAB网段的网卡
constexpr char MATLAB_IP[] = "192.168.2.180";
constexpr std::uint16_t MATLAB_PORT = 5006;

// 数据包结构（76 bytes）
#pragma pack(push,1)
struct Packet {
    std::uint32_t        seq_id;
    std::array<float,6>  q, dq, tau;   // 76 bytes
};
#pragma pack(pop)

static_assert(sizeof(Packet) == 76, "UDP Packet大小必须为76字节");

//====================== MATLAB UDP发送器 ======================
// 实时回调只负责入队，网络发送放到独立线程，避免sendto阻塞控制回调。
class MatlabUdpSender
{
public:
    MatlabUdpSender(const char* local_ip, const char* dest_ip, std::uint16_t dest_port)
    {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0)
        {
            throw std::system_error(errno, std::generic_category(), "创建UDP socket失败");
        }

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(0);  // 由系统选择本地源端口
        if (::inet_pton(AF_INET, local_ip, &local.sin_addr) != 1)
        {
            closeSocket();
            throw std::runtime_error(std::string("无效的UDP本地IP: ") + local_ip);
        }
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0)
        {
            const int saved_errno = errno;
            closeSocket();
            throw std::system_error(saved_errno, std::generic_category(), "绑定UDP本地IP失败");
        }

        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(dest_port);
        if (::inet_pton(AF_INET, dest_ip, &dest_.sin_addr) != 1)
        {
            closeSocket();
            throw std::runtime_error(std::string("无效的MATLAB IP: ") + dest_ip);
        }

        worker_ = std::thread(&MatlabUdpSender::sendLoop, this);
    }

    MatlabUdpSender(const MatlabUdpSender&) = delete;
    MatlabUdpSender& operator=(const MatlabUdpSender&) = delete;

    ~MatlabUdpSender()
    {
        stop();
    }

    void enqueue(std::size_t sample_index,
                 const std::array<double, 6>& q,
                 const std::array<double, 6>& dq,
                 const std::array<double, 6>& tau)
    {
        Packet packet{};
        packet.seq_id = static_cast<std::uint32_t>(sample_index + 1);
        for (std::size_t i = 0; i < 6; ++i)
        {
            packet.q[i] = static_cast<float>(q[i]);
            packet.dq[i] = static_cast<float>(dq[i]);
            packet.tau[i] = static_cast<float>(tau[i]);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(packet);
        }
        cv_.notify_one();
    }

    void stop() noexcept
    {
        bool expected = true;
        if (running_.compare_exchange_strong(expected, false))
        {
            cv_.notify_one();
        }
        if (worker_.joinable())
        {
            worker_.join();
        }
        closeSocket();
    }

private:
    void sendLoop() noexcept
    {
        while (true)
        {
            Packet packet{};
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return !queue_.empty() || !running_; });
                if (queue_.empty() && !running_)
                {
                    break;
                }
                packet = queue_.front();
                queue_.pop();
            }

            const ssize_t sent = ::sendto(
                sock_, &packet, sizeof(packet), 0,
                reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
            if (sent != static_cast<ssize_t>(sizeof(packet)) &&
                errno != EAGAIN && errno != ENOBUFS)
            {
                std::cerr << "UDP发送失败: " << std::generic_category().message(errno) << std::endl;
            }
        }
    }

    void closeSocket() noexcept
    {
        if (sock_ >= 0)
        {
            ::close(sock_);
            sock_ = -1;
        }
    }

    int sock_ = -1;
    sockaddr_in dest_{};
    std::queue<Packet> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{true};
    std::thread worker_;
};

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

    std::array<double, 6> q_m{};
    std::array<double, 6> q_dot_m{};
    std::array<double, 6> tau_m{};

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
        << "qm1,qm2,qm3,qm4,qm5,qm6,"
        << "qdotm1,qdotm2,qdotm3,qdotm4,qdotm5,qdotm6,"
        << "tau_m1,tau_m2,tau_m3,tau_m4,tau_m5,tau_m6\n";

    for (const auto& row : logs)
    {
        ofs << row.index << "," << row.time_s;

        for (double v : row.q_m) ofs << "," <<"]"<< v;
        for (double v : row.q_dot_m) ofs << "," << v;
        for (double v : row.tau_m) ofs << "," << v;

        ofs << "\n";
    }

    ofs.close();
}

//====================== 关闭碰撞检测 ======================
void disableCollision(xMateRobot& robot)
{
    error_code ec;

    robot.disableCollisionDetection(ec);

    if (ec)
    {
        std::cerr << "关闭碰撞检测失败: "
                  << ec.message() << std::endl;
    }
    else
    {
        std::cout << "碰撞检测已关闭" << std::endl;
    }
}

// //====================== 标定全部关节力矩传感器 ======================
// void calibrateJointTorqueSensors(xMateRobot& robot)
// {
//     error_code ec;

//     // SDK要求标定前设置正确的末端负载。这里沿用当前工具工件组；
//     // 运行前必须确认其质量、质心和惯量与实际末端负载一致。
//     const Toolset active_toolset = robot.toolset(ec);
//     if (ec)
//     {
//         throw std::system_error(ec, "读取当前工具负载失败");
//     }
//     robot.setToolset(active_toolset, ec);
//     if (ec)
//     {
//         throw std::system_error(ec, "设置工具负载失败");
//     }

//     std::cout << "标定使用的工具负载: mass=" << active_toolset.load.mass
//               << " kg, cog=[" << active_toolset.load.cog[0] << ", "
//               << active_toolset.load.cog[1] << ", "
//               << active_toolset.load.cog[2] << "] m" << std::endl;

//     // all_axes=true时axis_index不生效；传0而不是越界的6，避免误导。
//     robot.calibrateForceSensor(true, 0, ec);
//     if (ec)
//     {
//         throw std::system_error(ec, "关节力矩传感器标定指令失败");
//     }

//     // SDK说明该接口异步返回，标定约需100ms。
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//     std::cout << "全部关节力矩传感器标定完成" << std::endl;
// }

//====================== 主函数 ======================
int main()
{
    //================== 输入参数 ==================
    std::string robot_ip = "192.168.2.160";
    std::string local_ip = "192.168.2.2";
    const std::string input_q    = "../data_in/exp_J1_3.txt";
    const std::string output_txt = "../data_out/rt_torque_log5.txt";

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

        // // 标定时机器人应保持静止、不受外力。
        // calibrateJointTorqueSensors(SDU_SR4);

        // 独立线程把实时状态发往MATLAB。
        MatlabUdpSender udp_sender(UDP_LOCAL_IP, MATLAB_IP, MATLAB_PORT);
        std::cout << "UDP数据将发送到 " << MATLAB_IP << ":" << MATLAB_PORT << std::endl;

        // 关闭碰撞检测
        disableCollision(SDU_SR4);

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

        // // 5) 网络实时容差
        // SDU_SR4.setRtNetworkTolerance(20, ec);
        // if (ec)
        // {
        //     std::cerr << "setRtNetworkTolerance失败: " << ec.message() << std::endl;
        //     return -1;
        // }

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
                rokae::RtSupportedFields::jointVel_m,
                rokae::RtSupportedFields::tau_m,
            }
        );

        // 9) 读取当前关节角
        auto q_current = SDU_SR4.jointPos(ec);
        if (ec)
        {
            std::cerr << "读取当前关节角失败: " << ec.message() << std::endl;
            return -1;
        }

        // // 10) MoveJ 到零位
        // std::cout << "MoveJ到机器人零位..." << std::endl;
        // std::array<double, 6> q_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // rtCon->MoveJ(0.2, q_current, q_zero);

        // // 11) MoveJ 到轨迹起点
        // std::cout << "MoveJ到轨迹起点..." << std::endl;
        // rtCon->MoveJ(0.2, q_zero, traj.front());

        std::cout << "MoveJ到轨迹起点..." << std::endl;
        rtCon->MoveJ(0.2, q_current, traj.front());

        // 稳定一下
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

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

            // 这些 getStateData 在 setControlLoop(..., true) 后，
            // 每次回调前会自动刷新到最新状态
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::jointPos_m, row.q_m);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::jointVel_m, row.q_dot_m);
            (void)SDU_SR4.getStateData(rokae::RtSupportedFields::tau_m, row.tau_m);

            logs.push_back(row);
            udp_sender.enqueue(row.index, row.q_m, row.q_dot_m, row.tau_m);

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

        // 等待队列中的数据发送完毕后关闭socket。
        udp_sender.stop();

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
