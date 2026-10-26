#pragma once

#ifndef DISP_OS4_H
#define DISP_OS4_H

#include "ICalculateElement.h"
#include <string>

class OS4Vehicle;
class Report;

/*! \brief Диспетчер моделирования квадрокоптера
 *
 *  Корень дерева модулей: содержит модель OS4Vehicle и модуль записи
 *  результатов Report. Выполняет связывание, инициализацию и расчет.
 *
 *  Один вызов calculate() - один кадр длительностью frameStep. За кадр
 *  модель делает frameStep/step шагов интегрирования. Кадры вызываются:
 *   - подряд с максимальной скоростью функцией run();
 *   - по часам потоком Thread (режим реального времени).
 * \ingroup kernel
*/
class DispOS4 : public ICalculateElement
{
public:
    /*!
     * \param idClass  - имя класса модуля
     * \param step_    - шаг интегрирования, с
     * \param fileName - CSV-файл для результатов (пустая строка - не писать)
     * \param decim    - записывать каждый decim-й шаг
    */
    DispOS4(uint32_t idClass, double step_, const std::string &fileName, int decim);

    //! связывание и инициализация всех модулей
    bool prepare();
    //! моделирование до времени tEnd с максимальной скоростью, с
    void run(double tEnd);

    //! один кадр расчета
    virtual void calculate() override;
    virtual void finite() override {}

    //! длительность кадра, с (по умолчанию 0.01 - 100 Гц, как Hz100 в SimTVRS)
    void setFrameStep(double value){frameStep = value;}
    double frameStepSec(){return frameStep;}
    //! время окончания моделирования, с (0 - без ограничения)
    void setEndTime(double value){tEnd = value;}
    //! период вывода состояния в консоль, с модельного времени (0 - не выводить)
    void setConsolePeriod(double value){consolePeriod = value;}

    //! модель квадрокоптера (для настройки параметров до prepare())
    OS4Vehicle *vehicle(){return os4;}
private:
    //! вывести строку состояния в консоль
    void printState();

    OS4Vehicle *os4;
    Report     *report;
    double      frameStep;
    double      tEnd;
    double      consolePeriod;
    double      nextConsole;
};

#endif // DISP_OS4_H
