#include "wrp_sdk/platforms/bunker/bunker_base.hpp"
#include "../src/stopwatch.h"

#include <cmath>
#include <fstream>

//!  PI

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
    double x = 0;          //? 初始位置
    double y = 0;          //? 初始位置
    double theta = 0;
    double d = 0.1;        //? 距离
    double Ts = 0.01;      //? 控制周期： 10 ms

    double dotXR = 0;
    double dotYR = 0;
    double dotThetaR = 0;

    double xR = 0;
    double yR = 0;
    double thetaR = 0;
    double vR = 0.4;         //? 给定线速度
    double wR = 0.1;         //? 给定角速度

    double vc = 0;
    double wc = 0;
    double kp = 1;          //? 控制器增益 kp = 2a = 1
    double ki = 0.25;       //? 控制器增益 ki = a^2 = 1/4

    double v = 0;
    double w = 0;

    double z12 = 0;
    double z22 = 0;

    double y1 = 0;
    double y2 = 0;
    double sumError1 = 0;
    double sumError2 = 0;

    double ds1 = 0;
    double ds2 = 0;

    //? 扰动参数
    double u = 0.25;
    double f = 0.02;

    double vE = 0; 
    double wE = 0;

    while (true)
    {
        sw.tic();

        westonrobot::BunkerState state = bunker.GetBunkerState();
        // std::cout << "--------------- " << std::endl;
        // std::cout << "count: " << count << std::endl;
        // std::cout << "control mode: " << static_cast<int>(state.control_mode) << std::endl;
        // std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        // std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity << std::endl;
        // std::cout << "--------------- " << std::endl;

        //! 里程计 公式 7
        v = state.linear_velocity;                      //?  线速度
        w = state.angular_velocity;                     //? 角速度

        dotTheta = w;
        theta += dotTheta * Ts;   // dotTheta 积分 == theta

        dotX = v * cos(theta) - w * d * sin(theta) + ds1;
        dotY = v * sin(theta) + w * d * cos(theta) + ds2;

        x += dotX * Ts;   
        y += dotY * Ts;

        //* 30 s 之后加入扰动
        if(count < 3000)
        {
            ds1 = 0;
            ds2 = 0;
        }
        else if(count < 7000)
        {
            ds1 = -u * sin(2*M_PI*f*0.01*count);
            ds2 = u * cos(2*M_PI*f*0.01*count);
        }
        else
        {
            ds1 = 0;
            ds2 = 0;
        }
        
        //! 轨迹规划模块 公式 8
        dotThetaR = wR;
        thetaR += dotThetaR * Ts;
        dotXR = vR * cos(thetaR);
        dotYR = vR * sin(thetaR);

        xR += dotXR * Ts;
        yR += dotYR * Ts;

        //! 误差计算模块 公式 13
    
        double xE = x - xR;
        double yE = y - yR;

        //! 控制器设计 公式 15

       sumError1 = sumError1 + xE * Ts;             //? 积分作用
       sumError2 = sumError2 + yE * Ts;             //? 积分作用

        y1 = dotXR - kp*xE - ki*sumError1;          //? u1
        y2 = dotYR - kp*yE - ki*sumError2;          //? u2

       
        vc = cos(theta) * y1 + sin(theta) * y2;
        wc = 1.0/d * (((-sin(theta)) * y1 + cos(theta) * y2));

        if(vc >= 1.5)       //? vc 上限
        {
            vc = 1.5;
        }

        if (vc <= -1.5)     //? vc 下限
        {
            vc = -1.5;
        }
        
        if(wc >= 1)         //? wc 上限
        {
            wc = 1;
        }
        if(wc<=-1)          //? wc 下限
        {
            wc = -1;
        }

        vE = vc - v; 
        wE = wc - w;

        //! 发送指令
        std::cout << "count: " << count << std::endl;
        // std::cout << "(x, y,theta): " << x << ", " << y << ", " << theta << std::endl;
        // std::cout << "(xR, yR,thetaR): " << xR << ", " << yR  << ", " << thetaR << std::endl;
        // std::cout << "(u1, u2): " << y1 << ", " << y2 << std::endl;
        std::cout << "Motor: (vc, wc)" << "(" << vc << ", " << wc << ")" << std::endl;

        //! 保存数据
        
        ofs << x << ',' << y <<','<< xR << ',' << yR << ',' << xE
        << ',' << yE <<',' << vc << ',' << wc << ',' << vE << ','  << wE << ','<< z12
        << ',' << z22  << ',' << theta << ',' << thetaR << std::endl;

        
        bunker.SetMotionCommand(vc, wc);
       

        sw.sleep_until_ms(Ts*1000);
        ++count;

    }
    

    return 0;
}