#pragma once

#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include "ICalculateElement.h"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

/*! \brief Поток реального времени (по образцу Thread из SimTVRS)
 *
 *  Поток периодически вызывает calculate() у переданного модуля.
 *  Если включен таймер (setUseTimer(true)), вызовы привязаны к часам:
 *  k-й вызов выполняется в момент start + k * period. Ожидание идет
 *  до абсолютного момента (sleep_until), поэтому ошибка не накапливается,
 *  а грубость системного таймера (например, ~1 мс в Windows) дает
 *  только дрожание отдельного кадра, а не отставание.
 *
 *  Если расчет кадра не уложился в период, следующий кадр запускается
 *  сразу без ожидания и увеличивается счетчик missingCallCount:
 *  модель догоняет часы.
 *
 *  Без таймера поток считает с максимальной скоростью.
 *  В отличие от SimTVRS вместо boost::thread используется std::thread,
 *  а bind()/init() выполняются до запуска потока (DispOS4::prepare()).
 * \ingroup kernel
*/
class Thread
{
public:
    /*!
     * \param namePth - имя потока
     * \param obj     - модуль, у которого вызывается calculate()
     * \param period  - период вызова, с
    */
    Thread(const std::string &namePth, ICalculateElement *obj, double period);
    ~Thread();

    //! признак использования таймера (true)/бесконечный цикл (false)
    void setUseTimer(bool value){useTimer = value;}
    //! создать и запустить поток
    void create();
    //! попросить поток завершиться
    void stop(){stopRequest = true;}
    //! дождаться завершения потока
    void join();
    //! поток закончил работу (модуль остановлен или вызван stop())
    bool isFinished(){return finished;}

    //! имя потока
    std::string        name;
    //! кол-во кадров, не уложившихся в период
    std::atomic<int>   missingCallCount;
    //! кол-во выполненных кадров
    std::atomic<long>  callCount;
    //! максимальное время расчета одного кадра, с
    std::atomic<double> maxCalcTime;

private:
    //! главная функция потока
    static void start_thread(Thread *arg);

    //! инкапсулированный объект
    ICalculateElement *obj;
    //! период вызова
    std::chrono::nanoseconds period;
    //! признак использования таймера
    bool               useTimer;
    std::atomic<bool>  stopRequest;
    std::atomic<bool>  finished;
    std::thread        task;
};

#endif // THREAD_MANAGER_H
