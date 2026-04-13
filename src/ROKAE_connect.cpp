    /*该代码用于连接机器人*/
    /*返回一些机器人基本信息*/
    #include <iostream>
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
            std::error_code pc;  
            std::error_code oc;  
            // 创建机器人对象,并且实例化机器人类
            rokae::Robot_T<rokae::WorkType::collaborative,6> SDU_SR4;
            // 连接机器人
            SDU_SR4.connectToRobot(ip,local_ip);
            // 连接成功打印
            std::cout<<"机器人连接成功"<<std::endl;

            // 查询机器人信息
            auto robot_info = SDU_SR4.robotInfo(ec);
            if (ec) {
                std::cerr << "获取机器人信息失败: " << ec.message() << std::endl;
                return -1;
            }
            // 打印机器人信息
            std::cout<< "========== 机器人信息 ==========" <<std::endl;
            std::cout<<robot_info.id<<std::endl;
            std::cout<<robot_info.joint_num<<std::endl;
            std::cout<<robot_info.mac<<std::endl;
            std::cout<<robot_info.type<<std::endl;
            std::cout<<robot_info.version<<std::endl;

            // 打印目前机器人处于上电、下电、急停状态
            std::cout << "========== 机器人电源状态信息 ==========" << std::endl;
            PowerState robot_power = SDU_SR4.powerState(pc);
            if (pc) {
                std::cerr << "获取机器人上电状态信息失败: " << ec.message() << std::endl;
                return -1;
            }
            // 打印机器人电源状态
            switch (robot_power)
            {
                case PowerState::on:
                    std::cout << "机器人当前状态：上电" << std::endl;
                    break;
                case PowerState::off:
                    std::cout << "机器人当前状态：下电" << std::endl;
                    break;
                case PowerState::estop:
                    std::cout << "机器人当前状态：急停" << std::endl;
                    break;
                default:
                    std::cout << "机器人当前状态：安全门打开" << std::endl;
                    break;
            }

            // 打印机器人当前操作模式信息
            std::cout<< "========== 机器人当前操作模式 ==========" <<std::endl;
            OperateMode Oper_mode = SDU_SR4.operateMode(oc);
            if (pc) {
                std::cerr << "获取机器人操作模式信息失败: " << ec.message() << std::endl;
                return -1;
            }
            switch(Oper_mode)
            {
                case OperateMode::automatic:
                    std::cout << "机器人当前操作模式：自动" << std::endl;
                case OperateMode::manual:
                    std::cout << "机器人当前操作模式：手动" << std::endl;
                default:
                    std::cout << "机器人当前操作模式未知" << std::endl;
                    break;
            }

        }
        catch (const std::exception& e)
        {
            std::cerr<<"连接失败"<<e.what()<<std::endl;
            return -1;
        }
        return 0;
    }