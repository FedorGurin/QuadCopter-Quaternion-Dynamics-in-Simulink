/*! \file main.cpp
 *  \brief Запуск модели квадрокоптера OS4 (аналог sim('OS4dynamics_01'))
 *
 *  Параметры командной строки:
 *   --time   T     время моделирования, с (по умолчанию 5, как StopTime в Simulink)
 *   --dt     H     шаг интегрирования, с (по умолчанию 0.001)
 *   --out    FILE  CSV-файл результатов (по умолчанию os4sim.csv)
 *   --decim  N     писать каждый N-й шаг (по умолчанию 10)
 *   --tau    V     постоянная команда на моторы, Н*м (по умолчанию 0.1)
 *   --gyro-ip      умножать гироскопический момент на Ip
 *   --theta-tip    брать угол лопасти из параметров (12 град) вместо R
*/
#include "dispOS4.h"
#include "os4Vehicle.h"
#include "globalNameID.h"

#include <iostream>
#include <string>
#include <stdlib.h>
#include <string.h>

static void printUsage()
{
    std::cout << "usage: os4sim [--time T] [--dt H] [--out FILE] [--decim N]"
                 " [--tau V] [--gyro-ip] [--theta-tip]" << std::endl;
}

int main(int argc, char *argv[])
{
    double      tEnd     = 5.0;
    double      dt       = 0.001;
    std::string fileName = "os4sim.csv";
    int         decim    = 10;
    double      tau      = 0.1;
    bool        gyroIp   = false;
    bool        thetaTip = false;

    for(int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        bool hasValue   = (i + 1 < argc);
        if     (arg == "--time"  && hasValue) tEnd     = atof(argv[++i]);
        else if(arg == "--dt"    && hasValue) dt       = atof(argv[++i]);
        else if(arg == "--out"   && hasValue) fileName = argv[++i];
        else if(arg == "--decim" && hasValue) decim    = atoi(argv[++i]);
        else if(arg == "--tau"   && hasValue) tau      = atof(argv[++i]);
        else if(arg == "--gyro-ip")           gyroIp   = true;
        else if(arg == "--theta-tip")         thetaTip = true;
        else
        {
            printUsage();
            return 1;
        }
    }

    DispOS4 disp(ID_DispOS4, dt, fileName, decim);
    // настройка модели до инициализации
    OS4Vehicle *os4 = disp.vehicle();
    os4->options.constTau    = tau;
    os4->options.gyroWithIp  = gyroIp;
    os4->options.useThetaTip = thetaTip;

    if(disp.prepare() == false)
        return 1;
    disp.run(tEnd);

    const TOS4Output &o = os4->output;
    std::cout << "t = "         << o.time << " c" << std::endl
              << "Xe [м]     = " << o.Xe.x << " " << o.Xe.y << " " << o.Xe.z << std::endl
              << "Ve [м/с]   = " << o.Ve.x << " " << o.Ve.y << " " << o.Ve.z << std::endl
              << "pqr [рад/с]= " << o.pqr.x << " " << o.pqr.y << " " << o.pqr.z << std::endl
              << "euler [град] = " << o.euler.x * radToGrad << " "
                                   << o.euler.y * radToGrad << " "
                                   << o.euler.z * radToGrad << std::endl
              << "omega [рад/с]= " << o.omega[0] << " " << o.omega[1] << " "
                                   << o.omega[2] << " " << o.omega[3] << std::endl
              << "T [Н]      = " << o.T[0] << " " << o.T[1] << " "
                                 << o.T[2] << " " << o.T[3] << std::endl;
    std::cout << "results: " << fileName << std::endl;
    return 0;
}
