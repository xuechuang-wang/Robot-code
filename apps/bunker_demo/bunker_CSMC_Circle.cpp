#include "wrp_sdk/platforms/bunker/bunker_base.hpp"
#include "../src/stopwatch.h"

#include <cmath>
#include <fstream>

//!  CSMC
//!  圆

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
    double x = 4;
    double y = 4.5;
    double theta = 0;
    double d = 0.1;        //? 控制器增益
    double Ts = 0.02;      //? 控制周期： 20 ms

    double dotXR = 0;
    double dotYR = 0;
    double dotThetaR = 0;

    double xR = 5;
    double yR = 5;
    double thetaR = 0;
    // double vR = 0.5;         //? 给定线速度
    // double wR = 0;           //? 给定角速度

    double vc = 0;              //? 控制器输出量
    double wc = 0;              //? 控制器输出量
    double v = 0;               //? 移动机器人实际线速度
    double w = 0;               //? 移动机器人实际角速度
    double c1 = 1;      //? 控制器增益
    double c2 = 0.74;      //? 控制器增益
    double k1 = 0.017;      //? 控制器增益
    double k2 = 0.011;      //? 控制器增益
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
    double vE = 0;
    double wE = 0;

    double dotxE = 0;
    double dotyE = 0;
    double xE_Last = 0; //? 上一秒 xE 的误差
    double yE_Last = 0;

    double ds1 = 0;  
    double ds2 = 0;


    double z22 = 0;
    double z12 = 0;



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
        v = state.linear_velocity;
        w = state.angular_velocity;

        dotTheta = w;
        theta += dotTheta * Ts;
        dotX = v * cos(theta) - w * d * sin(theta) - ds1;
        dotY = v * sin(theta) + w * d * cos(theta) + ds2;

        x += dotX * Ts;
        y += dotY * Ts;

        //! 轨迹规划模块 公式 8
        dotXR = -0.4*sin(0.2*0.02*count);
        dotYR = 0.4*cos(0.2*0.02*count);

        // 10 s 之后
        if(count>500)
        {
            ds1 = 0.1 * sin(count*Ts/2.0);
            ds2 = 0.1 * cos(count*Ts);
            // ds1 = 0;
            // ds2 = 0;
        }
        xR += dotXR * Ts;  
        yR += dotYR * Ts;

        //! 误差计算模块 公式 13
    
        xE = xR - x;
        yE = yR - y; 

        dotxE = (xE - xE_Last) / Ts ; //? 一阶导的求法第一步
        xE_Last = xE;                //?  一阶导的求法第二步

        dotyE = (yE - yE_Last) / Ts ; //? 一阶导的求法第一步
        yE_Last = yE;                //?  一阶导的求法第二步
       

        //! 控制器设计 公式 15
        s1 = c1 * xE + dotxE;
        s2 = c2 * yE + dotyE;

        if(s1 > 0)  //? s1 符号函数表示
        {
            signs1 = 1;
        }
        else 
        {
            signs1 = -1;
        }

        if(s2 > 0)   //? s2符号函数表示
        {
            signs2 = 1;
        }
        else 
        {
            signs2 = -1;
        }

        signs1_jifen += signs1 * Ts;
        signs2_jifen += signs2 * Ts;

        u1 = dotXR + c1 * xE + k1 * signs1_jifen;
        u2 = dotYR + c2 * yE + k2 * signs2_jifen;

        
        vc = cos(theta) * u1 + sin(theta) * u2;
        wc = 1.0/d * (((-sin(theta)) * u1 + cos(theta) * u2));
        if(vc >= 1.5)
        {
            vc = 1.5;
        }
        if(wc >= 1)
        {
            wc = 1;
        }
        if(wc<=-1)
        {
            wc = -1;
        }

        //! 发送指令
        std::cout << "count: " << count << std::endl;
        std::cout << "(x, y,theta): " << x << ", " << y << ", " << theta << std::endl;
        std::cout << "(xR, yR,thetaR): " << xR << ", " << yR  << ", " << thetaR << std::endl;
        std::cout << "(u1, u2): " << u1 << ", " << u2 << std::endl;
        std::cout << "Motor: (vc, wc)" << "(" << vc << ", " << wc << ")" << std::endl;

        //! 保存数据

        // x, y, xr, yr, xe, ye, vr, wr, ve, we

        vE = vc - v; 
        wE = wc - w;
        
        ofs << x << ',' << y <<','<< xR << ',' << yR << ',' << xE
        << ',' << yE <<',' << vc << ',' << wc << ',' << vE << ','  << wE << ','<< z12
        << ',' << z22 << std::endl;

        bunker.SetMotionCommand(vc, wc);

       

        sw.sleep_until_ms(Ts*1000);
        ++count;

    }
    

    return 0;
}