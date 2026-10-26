#include "os4Equations.h"
#include "globalNameID.h"

using namespace math_func;

OS4Equations::OS4Equations(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "12 ODEs";
    // указываем зависимость
    bindTo(ID_OS4_Physical,  &physical);
    bindTo(ID_OS4_State,     &state);
    bindTo(ID_OS4_Gravity,   &gravity);
    bindTo(ID_OS4_Fuselage,  &fuselage);
    bindTo(ID_OS4_Propeller, &prop);
    // открываем интерфейсы
    share(ID_OS4_Derivative, &dx);
}
void OS4Equations::calculate()
{
    const TOS4Inertia &I = physical->inertia;
    TVector3 w = state->w;

    // суммы сил и моментов (блоки AddF и AddM)
    TVector3 F = prop->Fe + fuselage->Fe + gravity->Fe;
    TVector3 M = prop->Mb + fuselage->Mb;

    // поступательное движение
    dx.pos = state->vel;
    dx.vel = F / I.mass;

    // вращательное движение (тензор инерции диагональный)
    dx.w.x = ((I.Iyy - I.Izz) * w.y * w.z + M.x) / I.Ixx;
    dx.w.y = ((I.Izz - I.Ixx) * w.x * w.z + M.y) / I.Iyy;
    dx.w.z = ((I.Ixx - I.Iyy) * w.x * w.y + M.z) / I.Izz;

    // кинематика кватерниона (считается по нормализованному кватерниону)
    dx.q = quatDot(normalize(state->q), w);

    // моторы и индуктивный поток
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        dx.omega[i]   = prop->dOmega[i];
        dx.lambda0[i] = prop->dLambda0[i];
    }
}
