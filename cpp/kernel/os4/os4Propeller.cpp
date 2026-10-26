#include "os4Propeller.h"
#include "globalNameID.h"
#include <string.h>

using namespace math_func;

//! ограничение команды мотора (блок Saturation на входе control), Н*м
static const double TAU_MIN       = 0.0;
static const double TAU_MAX       = 0.2;
//! нижняя граница коэффициента момента C_Q (блок Saturation)
static const double CQ_MIN        = 0.001;
//! h/R, ниже которого считается касание земли (Compare To Constant)
static const double GROUND_STRIKE = 0.25;
//! таблица "cutoff at [-2, -3]": влияние земли полное при h/R <= 2 и отсутствует при h/R >= 3
static const double GE_FULL       = 2.0;
static const double GE_NONE       = 3.0;

OS4Propeller::OS4Propeller(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "Propeller & motor dynamics";
    memset((void*)&prop, 0, sizeof(prop));
    // указываем зависимость
    bindTo(ID_OS4_Physical, &physical);
    bindTo(ID_OS4_Options,  &options);
    bindTo(ID_OS4_State,    &state);
    bindTo(ID_OS4_Aero,     &aero);
    bindTo(ID_OS4_Control,  &control);
    // открываем интерфейсы
    share(ID_OS4_Propeller, &prop);
}
void OS4Propeller::calculate()
{
    TQuaternion q   = normalize(state->q);
    TVector3    pqr = state->w;

    double R     = physical->prop.rad;
    double A     = M_PI * R * R;                                   // площадь диска
    double k_ct  = physical->prop.sigma * physical->prop.a0 / 4.0; // "sigma*a0/4"
    // угол на конце лопасти: в Simulink в блок "theta_t" попадает R (см. TOS4Options)
    double theta_t = options->useThetaTip ? physical->prop.thetaTip : R;

    double sumT = 0.0;
    prop.collision = false;
    prop.M_thrust  = TVector3();
    prop.M_gyro    = TVector3();
    prop.M_react   = TVector3();

    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        TVector3 r      = physical->rotorPos[i];
        double   dir    = physical->rotorDir[i];
        double   omega  = state->omega[i];
        double   lambda = state->lambda0[i];
        double   omegaR = omega * R;                              // "wR"

        // команда мотора
        prop.tau[i] = saturation(control[i], TAU_MIN, TAU_MAX);

        // ---- скорость потока у винта ("Tangent velocity", "vertical speed")
        TVector3 V  = aero->Vb + cross(pqr, r);
        double   Vz = -V.z;
        double   Vh = sqrt(V.x * V.x + V.y * V.y);

        // ---- влияние земли ("ground effect")
        double z_e  = state->pos.z + bodyToEarth(q, r).z;          // координата винта в земной СК
        double hR   = -z_e / R;                                    // высота в радиусах винта
        prop.h[i]   = -z_e;
        if(hR <= GROUND_STRIKE)
            prop.collision = true;
        double hRs    = (hR < GROUND_STRIKE) ? GROUND_STRIKE : hR; // Saturation [1/4, inf]
        double cutoff = lookupClip(hR, GE_FULL, 1.0, GE_NONE, 0.0);
        prop.lambdaIGE[i] = lambda - lambda / (16.0 * hRs * hRs) * cutoff;

        // ---- коэффициент тяги [5.13]
        double vzR = Vz / omegaR + prop.lambdaIGE[i];              // "(Vz+Vi)/wR"
        prop.CT[i] = k_ct * (theta_t - vzR);

        // ---- динамика индуктивного потока по Глауэрту {2-58},{2-59}
        double vhR  = Vh / omegaR;
        double CTgl = 2.0 * lambda * sqrt(vhR * vhR + vzR * vzR);
        prop.dLambda0[i] = (prop.CT[i] - CTgl) / physical->downwashTau;

        // ---- коэффициент момента (3.4); при C_T < 0 в Simulink был бы
        // корень из отрицательного числа, здесь берется 0
        double ct = (prop.CT[i] > 0.0) ? prop.CT[i] : 0.0;
        prop.CQ[i] = ct * sqrt(ct / 2.0);
        if(prop.CQ[i] < CQ_MIN)
            prop.CQ[i] = CQ_MIN;

        // ---- тяга [5.5] и момент [5.6]
        prop.T[i] = aero->rho * A * R * R * prop.CT[i] * omega * omega;
        prop.Q[i] = aero->rho * A * R * R * R * prop.CQ[i] * omega * fabs(omega);

        // ---- мотор [2.30]
        prop.dOmega[i] = (prop.tau[i] - prop.Q[i]) / physical->inertia.Ip;

        // ---- моменты на корпус
        sumT += prop.T[i];
        prop.M_thrust += cross(r, TVector3(0.0, 0.0, -prop.T[i]));        // "R cross F"
        prop.M_gyro   += TVector3(-dir * pqr.y * omega, dir * pqr.x * omega, 0.0);
        prop.M_react  += TVector3(0.0, 0.0, -dir * prop.tau[i]);          // "action and Reaction"
    }
    if(options->gyroWithIp)
        prop.M_gyro = prop.M_gyro * physical->inertia.Ip;

    prop.Fe = bodyToEarth(q, TVector3(0.0, 0.0, -sumT));
    prop.Mb = prop.M_thrust + prop.M_gyro + prop.M_react;
}
