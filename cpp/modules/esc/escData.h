#pragma once

#ifndef ESC_DATA_H
#define ESC_DATA_H

#include <stdint.h>
#include "constants.h"

/*! \defgroup esc Модуль ESC
 *  \brief Регулятор оборотов (ESC), бесколлекторный мотор и батарея
*/

/*! \brief Настроечные параметры модуля ESC (МППМ: "ESC_Setup")
 *
 *  Значения по умолчанию подобраны под винт модели OS4 (R = 0.075 м,
 *  висение около 1100 рад/с): мотор класса 2200 об/мин/В на 3S LiPo.
 *  Это не параметры реального аппарата OS4 Bouabdallah (у него мотор
 *  с редуктором и другой винт). Параметры можно менять во время работы.
 * \ingroup esc
*/
typedef struct
{
    //! постоянная скорости мотора Kv, об/мин/В
    double kv;
    //! сопротивление обмотки (фаза-фаза), Ом
    double R;
    //! ток холостого хода (потери на трение и в стали), А
    double I0;
    //! передаточное число редуктора (1 - прямой привод)
    double gear;
    //! КПД редуктора
    double eta;
    //! постоянная времени ESC (отклик на команду), с
    double tauEsc;
    //! число банок LiPo
    double cells;
    //! емкость батареи, мА*ч
    double capacity;
    //! внутреннее сопротивление батареи, Ом
    double Rint;
    //! начальный заряд батареи, 0..1
    double socInit;
}TESCSetup;

/*! \brief Выходные параметры модуля ESC (МППМ: "ESC_Output")
 * \ingroup esc
*/
typedef struct
{
    //! коэффициент заполнения ШИМ (после запаздывания ESC), 0..1
    double duty[OS4_NUM_ROTORS];
    //! напряжение на моторе, В
    double U[OS4_NUM_ROTORS];
    //! ток мотора, А
    double I[OS4_NUM_ROTORS];
    /*! характеристика мотора на текущем шаге: момент на валу винта
     *  tau = max(0, torque0 - slope * omega), Н*м и Н*м*с
    */
    double torque0[OS4_NUM_ROTORS];
    double slope[OS4_NUM_ROTORS];
    //! напряжение батареи под нагрузкой, В
    double Ubat;
    //! ток батареи, А
    double Ibat;
    //! потребляемая мощность, Вт
    double power;
    //! заряд батареи, 0..1
    double soc;
    //! израсходовано, мА*ч
    double used;
}TESCOutput;

#endif // ESC_DATA_H
