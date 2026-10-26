#include "escModel.h"
#include "globalNameID.h"
#include "math_func.h"
#include "IMsg.h"
#include <string.h>

using namespace math_func;

TESCSetup escSetupInit()
{
    TESCSetup setup;
    setup.kv       = 2150.0;   // об/мин/В
    setup.R        = 0.1;      // Ом
    setup.I0       = 0.5;      // А
    setup.gear     = 1.0;      // прямой привод
    setup.eta      = 1.0;
    setup.tauEsc   = 0.005;    // с
    setup.cells    = 3.0;      // 3S LiPo
    setup.capacity = 2200.0;   // мА*ч
    setup.Rint     = 0.03;     // Ом
    setup.socInit  = 1.0;
    return setup;
}

ESCModel::ESCModel(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "ESC";
    setup     = escSetupInit();
    memset((void*)&output, 0, sizeof(output));
    // указываем зависимость
    bindTo(ID_OS4_Input, &input);
    bindTo(ID_OS4_State, &state);
    // открываем интерфейсы
    share(ID_ESC_Setup,  &setup);
    share(ID_ESC_Output, &output);
}
void ESCModel::init()
{
    ICalculateElement::init();
    // регистрация параметров в МППМ
    IRegMPPM *mppm = (IRegMPPM *)findShared(ID_MPPM);
    if(mppm != nullptr)
    {
        mppm->addReq(ptrToReq("ESC_Setup",  &setup));
        mppm->addReq(ptrToReq("ESC_Output", &output));
    }
    restart();
}
void ESCModel::restart()
{
    output.soc  = saturation(setup.socInit, 0.0, 1.0);
    output.used = 0.0;
    output.Ibat = 0.0;
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
        output.duty[i] = saturation(input->throttle[i], 0.0, 1.0);
}
void ESCModel::calculate()
{
    double dt  = step;
    double Ke  = 60.0 / (2.0 * M_PI * setup.kv);   // В*с/рад
    double Kt  = Ke;                               // Н*м/А
    double R   = (setup.R > 1e-6) ? setup.R : 1e-6;
    double kEsc = (setup.tauEsc > 1e-9) ? 1.0 - exp(-dt / setup.tauEsc) : 1.0;

    // батарея: ЭДС по заряду, просадка по току предыдущего шага
    double E    = setup.cells * (3.5 + 0.7 * output.soc);
    output.Ubat = E - setup.Rint * output.Ibat;
    if(output.Ubat < 0.0)
        output.Ubat = 0.0;

    double Ibat = 0.0;
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        // ESC
        double cmd = saturation(input->throttle[i], 0.0, 1.0);
        output.duty[i] += (cmd - output.duty[i]) * kEsc;
        output.U[i]     = output.duty[i] * output.Ubat;

        // мотор
        double omegaM = setup.gear * state->omega[i];
        output.I[i]   = (output.U[i] - Ke * omegaM) / R;
        if(output.I[i] < 0.0)
            output.I[i] = 0.0;
        output.torque0[i] = setup.eta * setup.gear * Kt * (output.U[i] / R - setup.I0);
        output.slope[i]   = setup.eta * setup.gear * setup.gear * Kt * Ke / R;

        Ibat += output.duty[i] * output.I[i];
    }
    output.Ibat  = Ibat;
    output.power = output.Ubat * Ibat;

    // расход заряда
    double dq = Ibat * dt / 3.6;   // мА*ч
    output.used += dq;
    if(setup.capacity > 0.0)
        output.soc = saturation(output.soc - dq / setup.capacity, 0.0, 1.0);
}
