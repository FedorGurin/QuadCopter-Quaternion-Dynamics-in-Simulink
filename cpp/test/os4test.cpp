/*! \file os4test.cpp
 *  \brief Простые проверки модели (без внешних библиотек)
*/
#include "dispOS4.h"
#include "os4Vehicle.h"
#include "globalNameID.h"
#include "math_func.h"

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
    DispOS4 disp(ID_DispOS4, 0.001, "", 1);
    OS4Vehicle *os4 = disp.vehicle();
    os4->initState.q   = TQuaternion();
    os4->initState.vel = TVector3();
    CHECK(disp.prepare());

    TOS4Propeller *prop = (TOS4Propeller *)disp.findShared(ID_OS4_Propeller);
    CHECK(prop != nullptr);
    CHECK_NEAR(prop->Mb.x, 0.0, 1e-12);
    CHECK_NEAR(prop->Mb.y, 0.0, 1e-12);
    CHECK_NEAR(prop->Mb.z, 0.0, 1e-12);
    // тяга направлена вверх (-z) и одинакова у всех винтов
    CHECK(prop->Fe.z < 0.0);
    CHECK_NEAR(prop->T[0], prop->T[3], 1e-12);
    // около 1.5 Н на винт при 2100 рад/с (C_T ~ 0.0028)
    CHECK(prop->T[0] > 1.0 && prop->T[0] < 2.0);
}

//! вариант с исправленным гироскопическим моментом
static void testGyroIp()
{
    DispOS4 d1(ID_DispOS4, 0.001, "", 1);
    DispOS4 d2(ID_DispOS4, 0.001, "", 1);
    d1.vehicle()->initState.w = TVector3(0.0, 1.0, 0.0);
    d2.vehicle()->initState.w = TVector3(0.0, 1.0, 0.0);
    d1.vehicle()->initState.omega[0] = 2500.0;   // разные скорости винтов
    d2.vehicle()->initState.omega[0] = 2500.0;
    d2.vehicle()->options.gyroWithIp = true;
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
    DispOS4 high(ID_DispOS4, 0.001, "", 1);
    DispOS4 low (ID_DispOS4, 0.001, "", 1);
    high.vehicle()->initState.q = TQuaternion();
    low .vehicle()->initState.q = TQuaternion();
    high.vehicle()->initState.vel = TVector3();
    low .vehicle()->initState.vel = TVector3();
    low .vehicle()->initState.pos = TVector3(0.0, 0.0, -0.15);   // винты на высоте 0.2 м
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
    DispOS4 disp(ID_DispOS4, 0.001, "", 1);
    CHECK(disp.prepare());
    disp.run(5.0);
    const TOS4Output &o = disp.vehicle()->output;
    CHECK(std::isfinite(o.Xe.z) && std::isfinite(o.pqr.x) && std::isfinite(o.omega[0]));
    CHECK_NEAR(norm(o.q), 1.0, 1e-9);

    // уменьшение шага почти не меняет результат (проверка интегрирования)
    DispOS4 fine(ID_DispOS4, 0.0005, "", 1);
    CHECK(fine.prepare());
    fine.run(o.time);
    const TOS4Output &f = fine.vehicle()->output;
    CHECK_NEAR(f.Xe.x, o.Xe.x, 1e-3 * (1.0 + fabs(o.Xe.x)));
    CHECK_NEAR(f.Xe.z, o.Xe.z, 1e-3 * (1.0 + fabs(o.Xe.z)));
}

int main()
{
    testQuaternion();
    testSymmetry();
    testGyroIp();
    testGroundEffect();
    testRun();
    if(failed == 0)
        std::cout << "all tests passed" << std::endl;
    return failed == 0 ? 0 : 1;
}
