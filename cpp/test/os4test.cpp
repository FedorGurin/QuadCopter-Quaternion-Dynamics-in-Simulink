/*! \file os4test.cpp
 *  \brief Простые проверки модели (без внешних библиотек)
*/
#include "dispOS4.h"
#include "os4Vehicle.h"
#include "globalNameID.h"
#include "math_func.h"
#include "threadManager.h"
#include "mppm.h"

#include <stddef.h>
#include <string.h>

#include <chrono>
#include <thread>

#include <iostream>
#include <math.h>

using namespace math_func;

static int failed = 0;

#define CHECK_NEAR(a, b, eps)                                                   \
    do {                                                                        \
        double va = (a), vb = (b);                                              \
        if(!(fabs(va - vb) <= (eps))) {                                         \
            std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "  " #a      \
                      << " = " << va << ", ожидалось " << vb << std::endl;      \
            failed++;                                                           \
        }                                                                       \
    } while(0)

#define CHECK(cond)                                                             \
    do {                                                                        \
        if(!(cond)) {                                                           \
            std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "  " #cond   \
                      << std::endl;                                             \
            failed++;                                                           \
        }                                                                       \
    } while(0)

//! кватернионы: повороты и углы Эйлера
static void testQuaternion()
{
    TQuaternion q = euler321ToQ(0.1, -0.5, 1.2);
    TVector3    e = qToEuler321(q);
    CHECK_NEAR(e.x,  0.1, 1e-12);
    CHECK_NEAR(e.y, -0.5, 1e-12);
    CHECK_NEAR(e.z,  1.2, 1e-12);

    // туда и обратно
    TVector3 v(1.0, -2.0, 3.0);
    TVector3 w = earthToBody(q, bodyToEarth(q, v));
    CHECK_NEAR(w.x, v.x, 1e-12);
    CHECK_NEAR(w.y, v.y, 1e-12);
    CHECK_NEAR(w.z, v.z, 1e-12);

    // тангаж -30 град: нос опущен, ось x корпуса смотрит вниз (+z в NED)
    TVector3 x = bodyToEarth(euler321ToQ(0.0, -30.0 * gradToRad, 0.0), TVector3(1.0, 0.0, 0.0));
    CHECK_NEAR(x.x, cos(30.0 * gradToRad), 1e-12);
    CHECK_NEAR(x.z, 0.5, 1e-12);

    // курс 90 град: ось x корпуса смотрит на восток (+y)
    x = bodyToEarth(euler321ToQ(0.0, 0.0, M_PI / 2.0), TVector3(1.0, 0.0, 0.0));
    CHECK_NEAR(x.y, 1.0, 1e-12);
}

//! горизонтальное положение без движения: моменты должны уравновеситься
static void testSymmetry()
{
    DispOS4 disp(ID_DispOS4, "", 1);
    OS4Vehicle *os4 = disp.vehicle();
    os4->setup.euler = TVector3();
    os4->setup.vel = TVector3();
    CHECK(disp.prepare());

    TOS4Propeller *prop = (TOS4Propeller *)disp.findShared(ID_OS4_Propeller);
    CHECK(prop != nullptr);
    CHECK_NEAR(prop->Mb.x, 0.0, 1e-12);
    CHECK_NEAR(prop->Mb.y, 0.0, 1e-12);
    CHECK_NEAR(prop->Mb.z, 0.0, 1e-12);
    // тяга направлена вверх (-z) и одинакова у всех винтов
    CHECK(prop->Fe.z < 0.0);
    CHECK_NEAR(prop->T[0], prop->T[3], 1e-12);
    // угол лопасти 12 град: около 6.8 Н на винт при 2100 рад/с
    CHECK(prop->T[0] > 6.0 && prop->T[0] < 7.5);
}

//! вариант "как в Simulink": угол лопасти равен R = 0.075 рад
static void testSimulinkThetaTip()
{
    DispOS4 disp(ID_DispOS4, "", 1);
    OS4Vehicle *os4 = disp.vehicle();
    os4->setup.euler = TVector3();
    os4->setup.vel = TVector3();
    os4->options.useThetaTip = false;
    CHECK(disp.prepare());

    TOS4Propeller *prop = (TOS4Propeller *)disp.findShared(ID_OS4_Propeller);
    // около 1.5 Н на винт при 2100 рад/с (C_T ~ 0.0028)
    CHECK(prop->T[0] > 1.0 && prop->T[0] < 2.0);
}

//! гироскопический момент с множителем Ip и без него
static void testGyroIp()
{
    DispOS4 d1(ID_DispOS4, "", 1);
    DispOS4 d2(ID_DispOS4, "", 1);
    d1.vehicle()->setup.w = TVector3(0.0, 1.0, 0.0);
    d2.vehicle()->setup.w = TVector3(0.0, 1.0, 0.0);
    d1.vehicle()->setup.omega[0] = 2500.0;   // разные скорости винтов
    d2.vehicle()->setup.omega[0] = 2500.0;
    d1.vehicle()->options.gyroWithIp = false;   // как в Simulink
    d2.vehicle()->options.gyroWithIp = true;    // исправленный вариант
    CHECK(d1.prepare());
    CHECK(d2.prepare());
    TOS4Propeller *p1 = (TOS4Propeller *)d1.findShared(ID_OS4_Propeller);
    TOS4Propeller *p2 = (TOS4Propeller *)d2.findShared(ID_OS4_Propeller);
    // q = 1 рад/с, сумма dir*w = -400 => M_gyro.x = 400 (без Ip)
    CHECK_NEAR(p1->M_gyro.x, 400.0, 1e-9);
    CHECK_NEAR(p2->M_gyro.x, 400.0 * 6e-5, 1e-12);
}

//! влияние земли увеличивает тягу у поверхности
static void testGroundEffect()
{
    DispOS4 high(ID_DispOS4, "", 1);
    DispOS4 low (ID_DispOS4, "", 1);
    high.vehicle()->setup.euler = TVector3();
    low .vehicle()->setup.euler = TVector3();
    high.vehicle()->setup.vel = TVector3();
    low .vehicle()->setup.vel = TVector3();
    low .vehicle()->setup.pos = TVector3(0.0, 0.0, -0.15);   // винты на высоте 0.2 м
    CHECK(high.prepare());
    CHECK(low.prepare());
    TOS4Propeller *ph = (TOS4Propeller *)high.findShared(ID_OS4_Propeller);
    TOS4Propeller *pl = (TOS4Propeller *)low .findShared(ID_OS4_Propeller);
    CHECK(pl->lambdaIGE[0] < ph->lambdaIGE[0]);
    CHECK(pl->T[0] > ph->T[0]);
    CHECK(pl->collision == false);
}

//! полный прогон 5 с: числа конечные, кватернион единичный
static void testRun()
{
    DispOS4 disp(ID_DispOS4, "", 1);
    CHECK(disp.prepare());
    disp.run(5.0);
    const TOS4Output &o = disp.vehicle()->output;
    CHECK(std::isfinite(o.Xe.z) && std::isfinite(o.pqr.x) && std::isfinite(o.omega[0]));
    CHECK_NEAR(norm(o.q), 1.0, 1e-9);

    // по умолчанию 400 Гц
    CHECK(disp.isFreq() == ICalculateElement::Hz400);
    CHECK_NEAR(disp.vehicle()->timeStep(), 0.0025, 1e-15);

    // уменьшение шага в 10 раз почти не меняет результат (проверка интегрирования)
    DispOS4 fine(ID_DispOS4, "", 1);
    fine.setTimeStep(0.00025);
    CHECK(fine.prepare());
    fine.run(o.time);
    const TOS4Output &f = fine.vehicle()->output;
    CHECK_NEAR(f.Xe.x, o.Xe.x, 1e-3 * (1.0 + fabs(o.Xe.x)));
    CHECK_NEAR(f.Xe.z, o.Xe.z, 1e-3 * (1.0 + fabs(o.Xe.z)));
}

//! реальное время: 0.3 с модели считаются за ~0.3 с по часам и дают тот же результат
static void testRealtime()
{
    DispOS4 fast(ID_DispOS4, "", 1);
    CHECK(fast.prepare());
    fast.run(0.3);

    DispOS4 rt(ID_DispOS4, "", 1);
    rt.setEndTime(0.3);
    CHECK(rt.prepare());
    Thread thread("pthTest", &rt, rt.timeStep());
    thread.setUseTimer(true);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    thread.create();
    thread.join();
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    CHECK(thread.isFinished());
    CHECK(thread.callCount == 120);   // 0.3 с * 400 Гц
    CHECK(wall > 0.28 && wall < 0.5);
    CHECK_NEAR(rt.vehicle()->output.time, 0.3, 1e-9);
    CHECK_NEAR(rt.vehicle()->output.Xe.z, fast.vehicle()->output.Xe.z, 1e-12);
}

//! клиент МППМ: отправить запрос и дождаться ответа
template<typename T> static bool mppmExchange(UdpSocket &client, T &req, uint16_t serverPort, uint16_t clientPort)
{
    req.head.port = clientPort;
    req.head.size = sizeof(T);
    if(client.sendTo((const uint8_t *)&req, sizeof(T), serverPort, "127.0.0.1") != (int)sizeof(T))
        return false;
    return client.receiveFrom((uint8_t *)&req, sizeof(T)) > 0;
}
//! найти индекс участка памяти по имени
static int16_t mppmIndex(UdpSocket &client, const char *name, uint16_t serverPort, uint16_t clientPort)
{
    TMemRequest mem;
    memset(&mem, 0, sizeof(mem));
    mem.head.type = R_T_LISTPM;
    mem.head.rwm  = E_REQ_READ;
    mem.index     = -1;
    strncpy((char *)mem.name, name, SIZE_NAME - 1);
    if(!mppmExchange(client, mem, serverPort, clientPort))
        return -1;
    return mem.index;
}

//! МППМ: чтение выходов, запись входов и начальных условий по UDP
static void testMPPM()
{
    const uint16_t serverPort = 19999;
    const uint16_t clientPort = 19998;

    DispOS4 disp(ID_DispOS4, "", 1);
    MPPM *mppm = disp.createMPPM(serverPort);
    CHECK(mppm != nullptr);
    if(mppm == nullptr)
        return;
    CHECK(disp.prepare());
    CHECK(mppm->table.size() == 3);

    Thread thread("pthMPPM", mppm, 0.0);
    thread.create();

    UdpSocket client;
    CHECK(client.init(clientPort, "127.0.0.1"));
    CHECK(client.bindTo());
    client.setTimeout(1000);

    int16_t iSetup  = mppmIndex(client, "OS4_Setup",  serverPort, clientPort);
    int16_t iInput  = mppmIndex(client, "OS4_Input",  serverPort, clientPort);
    int16_t iOutput = mppmIndex(client, "OS4_Output", serverPort, clientPort);
    CHECK(iSetup == 0 && iInput == 1 && iOutput == 2);
    CHECK(mppmIndex(client, "NoSuchName", serverPort, clientPort) == -1);

    // запись: высота 10 м, перезапуск и новая команда на мотор 1
    TSelectedRequest sel;
    memset(&sel, 0, sizeof(sel));
    sel.head.type = R_T_SELECTED;
    sel.head.rwm  = E_REQ_WRITE;
    sel.numValues = 3;
    double z = -10.0, tau = 0.15;
    uint8_t reset = 1;
    sel.values[0].indexInTable = iSetup;
    sel.values[0].offset       = offsetof(TOS4Setup, pos) + offsetof(TVector3, z);
    sel.values[0].byte         = sizeof(double);
    memcpy(sel.values[0].value, &z, sizeof(double));
    sel.values[1].indexInTable = iSetup;
    sel.values[1].offset       = offsetof(TOS4Setup, reset);
    sel.values[1].byte         = 1;
    sel.values[1].value[0]     = reset;
    sel.values[2].indexInTable = iInput;
    sel.values[2].offset       = offsetof(TOS4Input, tau);
    sel.values[2].byte         = sizeof(double);
    memcpy(sel.values[2].value, &tau, sizeof(double));
    CHECK(mppmExchange(client, sel, serverPort, clientPort));
    CHECK(sel.values[0].err == NOT_ERROR && sel.values[2].err == NOT_ERROR);

    OS4Vehicle *os4 = disp.vehicle();
    CHECK_NEAR(os4->setup.pos.z, -10.0, 1e-12);
    CHECK_NEAR(os4->input.tau[0], 0.15, 1e-12);

    // шаг диспетчера: модель перезапускается с новыми начальными условиями
    disp.calculate();
    disp.calculate();
    CHECK(os4->setup.reset == 0);
    CHECK_NEAR(os4->output.Xe.z, -10.0, 1e-3);

    // чтение выходов целиком
    TModuleRequest mod;
    memset(&mod, 0, sizeof(mod));
    mod.head.type = R_T_MODULE;
    mod.head.rwm  = E_REQ_READ;
    mod.index     = iOutput;
    CHECK(mppmExchange(client, mod, serverPort, clientPort));
    CHECK(mod.err == NOT_ERROR);
    CHECK(mod.sizeBuf == sizeof(TOS4Output));
    TOS4Output out;
    memcpy(&out, mod.buffer, sizeof(out));
    CHECK_NEAR(out.time, os4->output.time, 1e-12);
    CHECK_NEAR(out.Xe.z, os4->output.Xe.z, 1e-12);

    // ошибка индекса
    sel.numValues = 1;
    sel.head.rwm  = E_REQ_READ;
    sel.values[0].indexInTable = 7;
    CHECK(mppmExchange(client, sel, serverPort, clientPort));
    CHECK(sel.values[0].err == INDEX_TOO_MACH);

    thread.stop();
    thread.join();
}

int main()
{
    testQuaternion();
    testSymmetry();
    testSimulinkThetaTip();
    testGyroIp();
    testGroundEffect();
    testRun();
    testRealtime();
    testMPPM();
    if(failed == 0)
        std::cout << "all tests passed" << std::endl;
    return failed == 0 ? 0 : 1;
}
