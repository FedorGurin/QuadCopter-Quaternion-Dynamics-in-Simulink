#pragma once

#ifndef DISP_OS4_H
#define DISP_OS4_H

#include "ICalculateElement.h"
#include <string>

class OS4Vehicle;
class Report;
class MPPM;

/*! \brief Диспетчер моделирования квадрокоптера
 *
 *  Корень дерева модулей: содержит модель OS4Vehicle и модуль записи
 *  результатов Report. Выполняет связывание, инициализацию и расчет.
 *
 *  Частота расчета задается setFreq(), по умолчанию Hz400: один вызов
 *  calculate() - один шаг интегрирования 2.5 мс. Шаги вызываются:
 *   - подряд с максимальной скоростью функцией run();
 *   - по часам потоком Thread с периодом timeStep() (реальное время).
 * \ingroup kernel
*/
class DispOS4 : public ICalculateElement
{
public:
    /*!
     * \param idClass  - имя класса модуля
     * \param fileName - CSV-файл для результатов (пустая строка - не писать)
     * \param decim    - записывать каждый decim-й шаг
    */
    DispOS4(uint32_t idClass, const std::string &fileName, int decim);

    //! связывание и инициализация всех модулей
    bool prepare();
    //! моделирование до времени tEnd с максимальной скоростью, с
    void run(double tEnd);

    //! один шаг расчета
    virtual void calculate() override;
    virtual void finite() override {}

    /*! время окончания моделирования, с (0 - без ограничения:
     *  после касания земли модель ждет перезапуска через OS4_Setup.reset)
    */
    void setEndTime(double value){tEnd = value;}
    //! период вывода состояния в консоль, с модельного времени (0 - не выводить)
    void setConsolePeriod(double value){consolePeriod = value;}

    //! модель квадрокоптера (для настройки параметров до prepare())
    OS4Vehicle *vehicle(){return os4;}
    /*! Создать модуль МППМ (вызывать до prepare()).
     *  Модуль МППМ не считается в calculate() диспетчера: его нужно
     *  запустить в отдельном потоке Thread без таймера.
     * \return указатель на модуль или nullptr, если порт не открылся
    */
    MPPM *createMPPM(uint16_t port);
private:
    //! вывести строку состояния в консоль
    void printState();

    OS4Vehicle *os4;
    Report     *report;
    double      tEnd;
    double      consolePeriod;
    double      nextConsole;
    double      lastTime;
};

#endif // DISP_OS4_H
