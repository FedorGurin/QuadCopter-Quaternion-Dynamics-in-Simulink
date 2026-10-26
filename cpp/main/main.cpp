/*! \file main.cpp
 *  \brief Запуск модели квадрокоптера OS4 (аналог sim('OS4dynamics_01'))
 *
 *  Параметры командной строки:
 *   --time   T     время моделирования, с (по умолчанию 5, как StopTime в Simulink)
 *   --freq   F     частота расчета, Гц: 1, 6.25, 12.5, 25, 50, 100, 200, 400
 *                  (по умолчанию 400 - setFreq(Hz400), шаг 2.5 мс)
 *   --dt     H     задать шаг интегрирования вручную, с (вместо --freq)
 *   --out    FILE  CSV-файл результатов (по умолчанию os4sim.csv)
 *   --decim  N     писать каждый N-й шаг (по умолчанию 4, т.е. раз в 10 мс)
 *   --tau    V     постоянная команда на моторы, Н*м (по умолчанию 0.1)
 *   --simulink     вариант "как в Simulink": гироскопический момент без Ip
 *                  и угол лопасти, равный R (по умолчанию - исправленный вариант)
 *   --realtime     считать в реальном времени (1 с модели = 1 с по часам);
 *                  с --time 0 расчет идет до Ctrl+C
 *   --print  P     период вывода состояния в консоль, с
 *                  (по умолчанию 0.5 в реальном времени, иначе 0 - не выводить)
*/
#include "dispOS4.h"
#include "os4Vehicle.h"
#include "globalNameID.h"
#include "threadManager.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <string.h>

//! признак остановки по Ctrl+C
static volatile std::sig_atomic_t stopSignal = 0;
static void onSignal(int)
{
    stopSignal = 1;
}

static void printUsage()
{
    std::cout << "usage: os4sim [--time T] [--dt H] [--out FILE] [--decim N]"
                 " [--tau V] [--simulink] [--realtime] [--freq F] [--print P]" << std::endl;
}

int main(int argc, char *argv[])
{
    double      tEnd     = 5.0;
    double      dt       = 0.0;      // 0 - шаг задается частотой
    double      freq     = 400.0;
    std::string fileName = "os4sim.csv";
    int         decim    = 4;
    double      tau      = 0.1;
    bool        simulink = false;
    bool        realtime = false;
    double      print    = -1.0;

    for(int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        bool hasValue   = (i + 1 < argc);
        if     (arg == "--time"  && hasValue) tEnd     = atof(argv[++i]);
        else if(arg == "--dt"    && hasValue) dt       = atof(argv[++i]);
        else if(arg == "--out"   && hasValue) fileName = argv[++i];
        else if(arg == "--decim" && hasValue) decim    = atoi(argv[++i]);
        else if(arg == "--tau"   && hasValue) tau      = atof(argv[++i]);
        else if(arg == "--simulink")          simulink = true;
        else if(arg == "--realtime")          realtime = true;
        else if(arg == "--freq"  && hasValue) freq     = atof(argv[++i]);
        else if(arg == "--print" && hasValue) print    = atof(argv[++i]);
        else
        {
            printUsage();
            return 1;
        }
    }

    DispOS4 disp(ID_DispOS4, fileName, decim);
    // частота расчета
    if(dt > 0.0)
        disp.setTimeStep(dt);
    else
    {
        ICalculateElement::Freq f = ICalculateElement::HzInf;
        for(int k = ICalculateElement::Hz1; k < ICalculateElement::HzInf; k++)
        {
            ICalculateElement::Freq fk = (ICalculateElement::Freq)k;
            if(fabs(ICalculateElement::freqToHz(fk) - freq) < 1e-9)
                f = fk;
        }
        if(f == ICalculateElement::HzInf)
        {
            std::cerr << "--freq: допустимы 1, 6.25, 12.5, 25, 50, 100, 200, 400" << std::endl;
            return 1;
        }
        disp.setFreq(f);
    }
    // настройка модели до инициализации
    OS4Vehicle *os4 = disp.vehicle();
    os4->options.constTau    = tau;
    if(simulink)
    {
        os4->options.gyroWithIp  = false;
        os4->options.useThetaTip = false;
    }

    if(tEnd <= 0.0 && realtime == false)
    {
        std::cerr << "--time 0 допускается только с --realtime" << std::endl;
        return 1;
    }
    disp.setEndTime(tEnd);
    disp.setConsolePeriod(print >= 0.0 ? print : (realtime ? 0.5 : 0.0));

    if(disp.prepare() == false)
        return 1;

    if(realtime)
    {
        // поток с таймером: один шаг расчета за каждый период timeStep()
        std::signal(SIGINT, onSignal);
        Thread thread("pthOS4", &disp, disp.timeStep());
        thread.setUseTimer(true);
        thread.create();
        while(thread.isFinished() == false)
        {
            if(stopSignal != 0)
                thread.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        thread.join();
        std::cout << "шагов: " << thread.callCount
                  << ", не уложились в период: " << thread.missingCallCount
                  << ", макс. время шага: " << thread.maxCalcTime * 1000.0 << " мс"
                  << " (период " << disp.timeStep() * 1000.0 << " мс)" << std::endl;
    }
    else
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
