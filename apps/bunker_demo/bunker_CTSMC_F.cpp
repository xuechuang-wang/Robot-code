#include "wrp_sdk/platforms/bunker/bunker_base.hpp"
#include "../src/stopwatch.h"

#include <cmath>
#include <fstream>

//!  CTSMC + SMO
//!  CTSMC
//! 方形

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
    double x = -0.1;   //? 初始位置
    double y = -0.1;
    double theta = 0;
    double d = 0.2;        //? 控制器增益
    double Ts = 0.02;      //? 控制周期： 20 ms

    double dotXR = 0;  
    double dotYR = 0;
    double dotThetaR = 0;

    double xR = 0;  //? 初始位置
    double yR = 0;
    double xR_Last = 0;
    double yR_Last = 0;

    double thetaR = 0;
    double thetaReal = 0;
    // double vR = 0.5;         //? 给定线速度
    // double wR = 0.2;           //? 给定角速度

    double vc = 0;              //? 控制器输出量
    double wc = 0;              //? 控制器输出量
    double v = 0;               //? 移动机器人实际线速度
    double w = 0;               //? 移动机器人实际角速度
    double c1 = 0.8;      //? 控制器增益
    double c2 = 0.235;      //? 控制器增益  减小可降ye超调 26
    // //! SMO
    // double k1 = 0.01;      //? 控制器增益
    // double k2 = 0.03;      //? 控制器增益

    //! CTSMC
    double k1 = 0.06;      //? 控制器增益
    double k2 = 0.06;      //? 控制器增益

    double alpha1 = 0.7;
    double alpha2 = 0.33;  //? 终端指数   减小可加快ye初始收敛速度

    double u1 = 0;      //? 控制器1
    double u2 = 0;      //? 控制器2

    double s1 = 0;  //? 滑模面
    double s2 = 0;  //? 滑模面
    double signs1 = 0;
    double signs2 = 0;
    double signs1_jifen = 0;
    double signs2_jifen = 0;

    double signxE = 0;
    double signyE = 0;




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


    //! 定义 SMO变量
    double z11 = -0.1;   //? x 的估计
    double z12 = 0;  //?  -Dsin(t/40) 的估计
    double dotz11 = 0;
    double dotz12 = 0;
    double z21 = -0.1;   //? y 的估计
    double z22 = 0;   //?  Dcos(t/40) 的估计
    double dotz21 = 0;
    double dotz22 = 0;

    double sign_z11_x = 0;
    double sign_z21_y = 0;



    //! 定义 SMO变量
    double lamada11 = 0;
    double lamada21 = 0;
    double lamada10 = 0;
    double lamada20 = 0;

    //! 定义 SMO参数
    // double L = 0.13;
    double L = 0;





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

        //! 运动学模型
        v = state.linear_velocity;
        w = state.angular_velocity;

        dotTheta = w;
        theta += dotTheta * Ts;   // dotTheta 积分 == theta

        dotX = v * cos(theta) - w * d * sin(theta) - ds1;
        dotY = v * sin(theta) + w * d * cos(theta) + ds2;

        x += dotX * Ts;   
        y += dotY * Ts;
        thetaReal = atan2(dotY,dotX);

        //! 轨迹规划模块 公式 8

        if(0.02*count >= 0 && 0.02*count <10)
        {
            xR = 0.13*0.02*count;
            yR = 0;
        }
        else if(0.02*count >= 10 && 0.02*count < 20)
        {
            xR = 1.3;
            yR = 0.13*(0.02*count - 10);
        }
        else if(0.02*count >=20 && 0.02*count < 30)
        {
            xR = 1.3-0.13*(0.02*count - 20);
            yR = 1.3;
        }
        else
        {
            xR = 0;
            yR = 1.3-0.13*(0.02*count - 30);
        }

        dotXR = (xR-xR_Last)/Ts;
        xR_Last = xR;

        dotYR = (yR - yR_Last)/Ts;
        yR_Last = yR;

        thetaR = atan2(dotYR,dotXR);

        // 20 s 之后
        if(count>750)
        {
            ds1 = 0.1 * sin(count*Ts);
            ds2 = 0.1 * cos(count*Ts);
            // ds1 = 0;
            // ds2 = 0;
        }
     
        //! 误差计算模块 公式 13
    
        xE = xR - x;
        yE = yR - y; 

        dotxE = (xE - xE_Last) / Ts ; //? 一阶导的求法第一步
        xE_Last = xE;                //?  一阶导的求法第二步

        dotyE = (yE - yE_Last) / Ts ; //? 一阶导的求法第一步
        yE_Last = yE;                //?  一阶导的求法第二步
       
        //!  SMO
        if( z11-x > 0)  //? s1 符号函数表示
        {
            sign_z11_x = 1;
        }
        else 
        {
           sign_z11_x  = -1;
        }

        if( z21-y > 0)  //? s1 符号函数表示
        {
            sign_z21_y = 1;
        }
        else 
        {
            sign_z21_y = -1;
        }

         lamada11 = lamada21 = 1.5*pow(L, 0.5);
         lamada10 = lamada20 = 1.1 * L;
        
        dotz11 = u1 + z12 - lamada11 * pow(fabs(z11-x),0.5)  * sign_z11_x;
        dotz21 = u2 + z22 - lamada21 * pow(fabs(z21-y),0.5)  * sign_z21_y;
        z11 += dotz11 * Ts;
        z21 += dotz21 * Ts;

        dotz12 = -lamada10 * sign_z11_x;
        dotz22 = -lamada20 * sign_z21_y;
        z12 += dotz12 * Ts;
        z22 += dotz22 * Ts;

        //! 控制器设计 公式 15

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

        if(xE > 0)  //? sgn(ex)
        {
            signxE = 1;            
        }
        else
        {
            signxE = -1;
        }
        
        if(yE > 0)  //? sgn(ey)
        {
            signyE = 1;            
        }
        else
        {
            signyE = -1;
        }



        s1 = c1 * signxE * pow(fabs(xE),alpha1) + dotxE;
        s2 = c2 * signyE * pow(fabs(yE),alpha2) + dotyE;

        signs1_jifen += signs1 * Ts;
        signs2_jifen += signs2 * Ts;

        u1 = dotXR + c1 * signxE * pow(fabs(xE),alpha1) + k1 * signs1_jifen - z12;
        u2 = dotYR + c2 * signyE * pow(fabs(yE),alpha2) + k2 * signs2_jifen - z22;

        
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
        // std::cout << "(x, y,theta): " << x << ", " << y << ", " << theta << std::endl;
        // std::cout << "(xR, yR,thetaR): " << xR << ", " << yR  << ", " << thetaR << std::endl;
        // std::cout << "(u1, u2): " << u1 << ", " << u2 << std::endl;
        std::cout << "Motor: (vc, wc)" << "(" << vc << ", " << wc << ")" << std::endl;

        //! 保存数据

        // x, y, xr, yr, xe, ye, vr, wr, ve, we

        vE = vc - v; 
        wE = wc - w;
        
        ofs << x << ',' << y <<','<< xR << ',' << yR << ',' << xE
        << ',' << yE <<',' << vc << ',' << wc << ',' << vE << ',' 
        << wE << ',' << thetaR << ',' << thetaReal << ','<< z12
        << ',' << z22 << ',' << z11 << ',' << z21 << std::endl;


        bunker.SetMotionCommand(vc, wc);


        sw.sleep_until_ms(Ts*1000);
        ++count;

    }
    

    return 0;
}
