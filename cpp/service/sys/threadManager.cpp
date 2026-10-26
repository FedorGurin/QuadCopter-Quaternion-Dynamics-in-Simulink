#include "threadManager.h"

Thread::Thread(const std::string &namePth, ICalculateElement *obj_, double period_)
{
    name             = namePth;
    obj              = obj_;
    period           = std::chrono::nanoseconds((long long)(period_ * 1e9));
    useTimer         = false;
    missingCallCount = 0;
    callCount        = 0;
    maxCalcTime      = 0.0;
    stopRequest      = false;
    finished         = false;
}
Thread::~Thread()
{
    stop();
    join();
}
void Thread::create()
{
    finished = false;
    task     = std::thread(Thread::start_thread, this);
}
void Thread::join()
{
    if(task.joinable())
        task.join();
}
void Thread::start_thread(Thread *arg)
{
    typedef std::chrono::steady_clock clock;
    clock::time_point next = clock::now();

    // основное тело потока
    while(arg->stopRequest == false && arg->obj->isStop() == false)
    {
        if(arg->useTimer == true)
        {
            next += arg->period;
            if(clock::now() > next)
                arg->missingCallCount++;   // не успели: считаем без ожидания
            else
                std::this_thread::sleep_until(next);
        }
        clock::time_point t0 = clock::now();
        arg->obj->calculate();
        double dt = std::chrono::duration<double>(clock::now() - t0).count();
        if(dt > arg->maxCalcTime)
            arg->maxCalcTime = dt;
        arg->callCount++;
    }
    arg->finished = true;
}
