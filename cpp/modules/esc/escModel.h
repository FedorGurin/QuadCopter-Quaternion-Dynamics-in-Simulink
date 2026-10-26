#pragma once

#ifndef ESC_MODEL_H
#define ESC_MODEL_H

#include "ICalculateElement.h"
#include "escData.h"
#include "os4Data.h"

/*! \brief ESC + бесколлекторный мотор + батарея
 *
 *  Вход: газ throttle[i] = 0..1 из OS4_Input.
 *
 *  ESC (апериодическое звено, точная дискретизация):
 *    duty += (throttle - duty) * (1 - exp(-dt / tauEsc))
 *    U = duty * Ubat
 *
 *  Мотор постоянного тока (индуктивность не учитывается: ее постоянная
 *  времени ~0.1 мс много меньше шага 2.5 мс):
 *    Ke = Kt = 60 / (2*pi*Kv)
 *    I  = max(0, (U - Ke * gear * omega) / R)
 *    tau_винт = eta * gear * Kt * (I - I0)
 *
 *  Подставив I, получаем момент как линейную функцию скорости винта:
 *    tau = torque0 - slope * omega
 *    torque0 = eta * gear * Kt * (U/R - I0),  slope = eta * gear^2 * Kt * Ke / R
 *  Модуль считает torque0 и slope один раз за шаг (напряжение держится
 *  постоянным в течение шага, как у реального ESC), а OS4Propeller
 *  берет момент по этой характеристике на каждой стадии RK4. Поэтому
 *  противо-ЭДС учитывается внутри шага интегрирования.
 *
 *  Батарея:
 *    E(soc) = cells * (3.5 + 0.7 * soc),  Ubat = E - Rint * Ibat
 *    Ibat = sum(duty_i * I_i)  (ESC без потерь)
 *    soc -= Ibat * dt / (3.6 * capacity)
 * \ingroup esc
*/
class ESCModel : public ICalculateElement
{
public:
    ESCModel(uint32_t idClass);

    //! регистрация в МППМ и начальное состояние
    virtual void init() override;
    //! расчет на шаг (вызывает OS4Vehicle перед шагом интегрирования)
    virtual void calculate() override;
    virtual void finite() override {}
    //! вернуть начальное состояние (полная батарея, duty по текущему газу)
    void restart();

    //! настроечные параметры
    TESCSetup  setup;
    //! выходные параметры
    TESCOutput output;

private:
    //! входные параметры OS4 (газ)
    TOS4Input *input;
    //! состояние OS4 (скорости винтов)
    TOS4State *state;
};

//! параметры ESC по умолчанию \ingroup esc
TESCSetup escSetupInit();

#endif // ESC_MODEL_H
