#include "os4Gravity.h"
#include "globalNameID.h"

using namespace math_func;

OS4Gravity::OS4Gravity(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "gravity";
    // указываем зависимость
    bindTo(ID_OS4_Physical, &physical);
    bindTo(ID_OS4_State,    &state);
    // открываем интерфейсы
    share(ID_OS4_Gravity, &gravity);
}
void OS4Gravity::calculate()
{
    TQuaternion q = normalize(state->q);

    gravity.Fe = TVector3(0.0, 0.0, physical->inertia.mass * g_acc);
    gravity.gB = earthToBody(q, TVector3(0.0, 0.0, -g_acc));
}
