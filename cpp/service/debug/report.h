#pragma once

#ifndef REPORT_H
#define REPORT_H

#include "ICalculateElement.h"
#include "globalData.h"
#include <fstream>
#include <string>

/*! \brief Запись результатов моделирования в CSV-файл
 *
 *  Аналог блоков "To Workspace" (os4sim_time, os4sim_stateX, ...).
 *  Одна строка файла - один момент времени, столбцы подписаны
 *  в первой строке.
 * \ingroup kernel
*/
class Report : public ICalculateElement
{
public:
    /*!
     * \param idClass  - имя класса модуля
     * \param fileName - имя CSV-файла
     * \param decim    - записывать каждый decim-й шаг
    */
    Report(uint32_t idClass, const std::string &fileName, int decim = 1);
    ~Report();

    virtual void init() override;
    virtual void calculate() override;
    virtual void finite() override {}

    //! признак успешного открытия файла
    bool isOpen(){return file.is_open();}
private:
    //! записать текущую строку
    void writeRow();

    std::string   fileName;
    std::ofstream file;
    int           decim;
    int           count;
    //! выходы модели
    TOS4Output   *output;
};

#endif // REPORT_H
