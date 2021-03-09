#include "wrp_sdk/platforms/bunker/bunker_base.hpp"
#include "../src/stopwatch.h"

#include <cmath>
#include <fstream>

//!  CSMC + NDOB

int main(int argc, char **argv)
{
    std::string device_name;
    int32_t baud_rate = 0;

    if (argc == 2)
    {
        device_name = { argv[1] };
        std::cout << "Specified CAN: " << device_name << std::endl;
    }
    else
    {
        std::cout << "Usage: app_hunter_demo <interface>" << std::endl;
        std::cout << "Example 1: ./app_hunter_demo can0" << std::endl;

        return -1;
    }
    
    //! 将数据存储到 data.txt

    std::ofstream ofs;

    ofs.open("data.txt",std::ios::out|std::ios::app);


    westonrobot::BunkerBase bunker;
    bunker.Connect(device_name);

    westonrobot::StopWatch sw;

    int count = 0;

    double dotX = 0;
    double dotY = 0;
    double dotTheta = 0;
    double x = 0;
    double y = 0;
    double theta = 0;
    double d = 0.1;        //? 控制器增益
    double Ts = 0.02;      //? 控制周期： 20 ms

    double dotXR = 0;
    double dotYR = 0;
    double dotThetaR = 0;

    double xR = 0;
    double yR = 0;
    double thetaR = 0;
    double vR = 0.1;         //? 给定线速度
    double wR = 0;           //? 给定角速度

    double vc = 0;
    double wc = 0;
    double c1 = 0.8;      //? 控制器增益
    double c2 = 0.8;      //? 控制器增益
    double k1 = 0.02;      //? 控制器增益
    double k2 = 0.02;      //? 控制器增益

    double u1 = 0;      //? 控制器1
    double u2 = 0;      //? 控制器2

    double s1 = 0;  //? 滑模面
    double s2 = 0;  //? 滑模面
    double signs1 = 0;
    double signs2 = 0;
    double signs1_jifen = 0;
    double signs2_jifen = 0;


    double xE = 0;  //? X方向 误差
    double yE = 0;  //? Y方向 误差
    double dotxE = 0;
    double dotyE = 0;
    double xE_Last = 0; //? 上一秒 xE 的误差
    double yE_Last = 0;
    //! 定义 NDOB变量
    double p1 = 0;
    double p2 = 0;
    double dotp1 = 0;
    double dotp2 = 0;
    double rou1 = 0;
    double rou2 = 0;
    double rou1_guji = 0;
    double rou2_guji = 0;

    //! 定义 NDOB参数
    double l1 = 0.1;
    double l2 = 0.1;

    while (true)
    {
        sw.tic();

        westonrobot::BunkerState state = bunker.GetBunkerState();
        std::cout << "--------------- " << std::endl;
        std::cout << "count: " << count << std::endl;
        std::cout << "control mode: " << static_cast<int>(state.control_mode) << std::endl;
        std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity << std::endl;
        std::cout << "--------------- " << std::endl;

        //! 里程计 公式 7
        dotTheta = state.angular_velocity;
        theta += dotTheta * Ts;
        dotX = state.linear_velocity * cos(theta) - state.angular_velocity * d * sin(theta);
        dotY = state.angular_velocity * sin(theta) + state.angular_velocity * d * cos(theta);

        x += dotX * Ts;
        y += dotY * Ts;

        //! 轨迹规划模块 公式 8
        dotThetaR = wR;
        thetaR += dotThetaR * Ts;
        dotXR = vR * cos(thetaR) - wR * d * sin(thetaR);
        dotYR = wR * sin(thetaR) + wR * d * cos(thetaR);

        xR += dotXR * Ts;  
        yR += dotYR * Ts;

        //! 误差计算模块 公式 13
    
        xE = xR - x;
        yE = yR - y; 

        dotxE = (xE - xE_Last) / Ts ; //? 一阶导的求法第一步
        xE_Last = xE;                //?  一阶导的求法第二步

        dotyE = (yE - yE_Last) / Ts ; //? 一阶导的求法第一步
        yE_Last = yE;                //?  一阶导的求法第二步
       
        //!  NDOB
        dotp1 = -l1 * p1 - l1 * l1 * x - l1 * u1;
        dotp2 = -l2 * p2 - l2 * l2 * y - l2 * u2;
        rou1_guji = p1 + l1 * x;
        rou2_guji = p2 + l2 * y; 

        p1 += dotp1 * Ts;
        p2 += dotp2 * Ts;

        //! 控制器设计 公式 15
        s1 = c1 * xE + dotxE;
        s2 = c2 * yE + dotyE;

        if(s1 > 0)  //? s1 符号函数表示
        {
            signs1 = 1;
        }
        else if(s1 == 0)
        {
            signs1 = 0;
        }
        else 
        {
            signs1 = -1;
        }

        if(s2 > 0)   //? s2符号函数表示
        {
            signs2 = 1;
        }
        else if(s2 == 0)
        {
            signs2 = 0;
        }
        else 
        {
            signs2 = -1;
        }

        signs1_jifen += signs1 * Ts;
        signs2_jifen += signs2 * Ts;

        u1 = dotXR + c1 * xE + k1 * signs1_jifen - rou1_guji;
        u2 = dotYR + c2 * yE + k2 * signs2_jifen - rou2_guji;

        
        vc = cos(theta) * u1 + sin(theta) * u2;
        wc = 1.0/d * (((-sin(theta)) * u1 + cos(theta) * u2));

        //! 发送指令
        std::cout << "(x, y,theta): " << x << ", " << y << ", " << theta << std::endl;
        std::cout << "(xR, yR,thetaR): " << xR << ", " << yR  << ", " << thetaR << std::endl;
        std::cout << "(u1, u2): " << u1 << ", " << u2 << std::endl;
        std::cout << "Motor: (vc, wc)" << "(" << vc << ", " << wc << ")" << std::endl;

        //! 保存数据
        
        ofs << x << ',' << y << ',' << theta <<','<< xR << ',' << yR << ',' << thetaR << ',' << vc
        << ',' << wc <<',' << xE << ',' << yE << std::endl;

        bunker.SetMotionCommand(vc, wc);
        

       

        sw.sleep_until_ms(Ts*1000);
        ++count;

    }
    

    return 0;
}