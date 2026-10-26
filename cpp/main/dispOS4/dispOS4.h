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
 *  результатов Report. Выполняет связывание, инициализацию и цикл
 *  моделирования до заданного времени (StopTime в Simulink).
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
    //! моделирование до времени tEnd, с
    void run(double tEnd);

    virtual void calculate() override;
    virtual void finite() override {}

    //! модель квадрокоптера (для настройки параметров до prepare())
    OS4Vehicle *vehicle(){return os4;}
private:
    OS4Vehicle *os4;
    Report     *report;
};

#endif // DISP_OS4_H
