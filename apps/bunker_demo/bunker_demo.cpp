#include "wrp_sdk/platforms/bunker/bunker_base.hpp"
#include "../src/stopwatch.h"

#include <cmath>
#include <fstream>



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
    double kx1 = 5;      //? 控制器增益
    double ky1 = 5;      //? 控制器增益
    double kx2 = 0.5;      //? 控制器增益
    double ky2 = 0.5;      //? 控制器增益
        
    double s1 = 0;
    double s2 = 0;
    double u1 = 0;
    double u2 = 0;       

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
    
        

        double xE = x - xR;
        double yE = y - yR;
       

        //! 控制器设计 公式 15
        
        s1 += xE * Ts;
        s2 += yE * Ts;

        u1 = dotXR - kx1 * xE - kx2 * s1;
        u2 = dotYR - ky1 * yE - ky2 * s2;
       

        vc = cos(theta) * u1 + sin(theta) * u2;
        wc = 1.0/d * (((-sin(theta)) * u1 + cos(theta) * u2));

        //! 发送指令
        std::cout << "(x, y,theta): " << x << ", " << y << ", " << theta << std::endl;
        std::cout << "(xR, yR,thetaR): " << xR << ", " << yR  << ", " << thetaR << std::endl;
        std::cout << "(u1, u2): " << u1 << ", " << u2 << std::endl;
        std::cout << "Motor: (vc, wc)" << "(" << vc << ", " << wc << ")" << std::endl;

        //! 保存数据
        

        ofs << x << '\t' << y << '\t' << theta <<'\t'<< xR << '\t' << yR << '\t' << thetaR << '\t' << vc
        << '\t' << wc << std::endl;

        bunker.SetMotionCommand(vc, wc);
        

       

        sw.sleep_until_ms(Ts*1000);
        ++count;

    }
    

    return 0;
}