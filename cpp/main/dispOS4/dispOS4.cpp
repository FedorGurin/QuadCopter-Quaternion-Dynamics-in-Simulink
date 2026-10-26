#include "dispOS4.h"
#include "os4Vehicle.h"
#include "report.h"
#include "globalNameID.h"
#include <iostream>

DispOS4::DispOS4(uint32_t idClass, double step_, const std::string &fileName, int decim)
    :ICalculateElement(idClass, step_)
{
    stringUId = "dispOS4";

    os4 = new OS4Vehicle(ID_OS4Vehicle, step_);
    addElement(os4);

    report = nullptr;
    if(!fileName.empty())
    {
        report = new Report(ID_REPORT, fileName, decim);
        addElement(report);
    }
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
    return true;
}
void DispOS4::calculate()
{
    // шаг модели, затем расчет выходов и запись
    os4->calculate();
    os4->finite();
    if(report != nullptr)
        report->calculate();
}
void DispOS4::run(double tEnd)
{
    // половина шага - защита от накопления ошибки округления времени
    while(os4->time + 0.5 * step < tEnd && os4->isStop() == false)
        calculate();
}
