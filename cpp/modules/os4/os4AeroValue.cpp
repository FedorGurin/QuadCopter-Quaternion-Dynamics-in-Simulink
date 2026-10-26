#include "os4AeroValue.h"
#include "globalNameID.h"

using namespace math_func;

OS4AeroValue::OS4AeroValue(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId  = "aero value";
    aero.Vb    = TVector3();
    aero.Vinf  = 0.0;
    aero.psi   = 0.0;
    aero.theta = 0.0;
    aero.rho   = rho_0;
    // указываем зависимость
    bindTo(ID_OS4_State, &state);
    // открываем интерфейсы
    share(ID_OS4_Aero, &aero);
}
void OS4AeroValue::calculate()
{
    TQuaternion q = normalize(state->q);

    // "vel to Body": скорость в связанной СК
    aero.Vb   = earthToBody(q, state->vel);
    aero.Vinf = length(aero.Vb);
    aero.psi  = atan2(aero.Vb.y, aero.Vb.x);
    // в Simulink при Vinf = 0 здесь получается деление на ноль,
    // поэтому добавлена защита
    if(aero.Vinf > 1e-9)
        aero.theta = asin(saturation(-aero.Vb.z / aero.Vinf, -1.0, 1.0));
    else
        aero.theta = 0.0;
    aero.rho = rho_0;
}
