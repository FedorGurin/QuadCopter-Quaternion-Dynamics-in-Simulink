#include "dispOS4.h"
#include "os4Vehicle.h"
#include "report.h"
#include "globalNameID.h"
#include <iostream>
#include <iomanip>

DispOS4::DispOS4(uint32_t idClass, const std::string &fileName, int decim)
    :ICalculateElement(idClass)
{
    stringUId     = "dispOS4";
    tEnd          = 0.0;
    consolePeriod = 0.0;
    nextConsole   = 0.0;

    os4 = new OS4Vehicle(ID_OS4Vehicle);
    addElement(os4);

    report = nullptr;
    if(!fileName.empty())
    {
        report = new Report(ID_REPORT, fileName, decim);
        addElement(report);
    }
    // частота расчета (шаг передается всем подчиненным модулям)
    setFreq(ICalculateElement::Hz400);
}
bool DispOS4::prepare()
{
    if(bind() == false)
    {
        std::cerr << "DispOS4: bind failed" << std::endl;
        return false;
    }
    init();
    if(report != nullptr && report->isOpen() == false)
    {
        std::cerr << "DispOS4: can't open report file" << std::endl;
        return false;
    }
    nextConsole = 0.0;
    setStart();
    return true;
}
void DispOS4::calculate()
{
    if(os4->isStop() == false)
    {
        // шаг модели, затем расчет выходов и запись
        os4->calculate();
        os4->finite();
        if(report != nullptr)
            report->calculate();
    }
    // половина шага - защита от накопления ошибки округления времени
    if(consolePeriod > 0.0 && os4->time + 0.5 * step >= nextConsole)
    {
        printState();
        nextConsole += consolePeriod;
    }
    if(os4->isStop() || (tEnd > 0.0 && os4->time + 0.5 * step >= tEnd))
        setStop();
}
void DispOS4::run(double tEnd_)
{
    tEnd = tEnd_;
    while(isStop() == false)
        calculate();
}
void DispOS4::printState()
{
    const TOS4Output &o = os4->output;
    std::ios::fmtflags      flags = std::cout.flags();
    std::streamsize         prec  = std::cout.precision();
    std::cout << std::fixed << std::setprecision(2)
              << "t=" << std::setw(7) << o.time
              << "  h=" << std::setw(8) << -o.Xe.z
              << "  x=" << std::setw(8) << o.Xe.x
              << "  V=" << std::setw(6) << o.aero.Vinf
              << "  крен=" << std::setw(7) << o.euler.x * radToGrad
              << "  тангаж=" << std::setw(7) << o.euler.y * radToGrad
              << "  курс=" << std::setw(7) << o.euler.z * radToGrad
              << std::endl;
    std::cout.flags(flags);
    std::cout.precision(prec);
}
