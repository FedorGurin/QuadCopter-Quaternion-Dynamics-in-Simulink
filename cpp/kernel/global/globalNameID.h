#pragma once

#ifndef GLOBALNAMEID_H
#define GLOBALNAMEID_H

/*! Глобальный список уникальных идентификаторов
 *
 * Идентификаторы используются в двух случаях:
 *  - как имя класса модуля (idClass в конструкторе ICalculateElement);
 *  - как имя открытого участка памяти (share/bindTo), через который
 *    модули обмениваются данными. Это аналог сигналов Goto/From
 *    и шин Mux/Demux в модели Simulink.
 * \ingroup kernel
*/
enum EGlobalProtoID{
    //! ---------- модули (имена классов) ----------
    ID_DispOS4                  ,   /*диспетчер моделирования*/
    ID_OS4Vehicle               ,   /*квадрокоптер OS4 в целом (интеграторы)*/
    ID_OS4AeroValueModel        ,   /*подсистема "aero value"*/
    ID_OS4GravityModel          ,   /*подсистема "gravity"*/
    ID_OS4FuselageModel         ,   /*подсистема "fuselage aerodynamics"*/
    ID_OS4PropellerModel        ,   /*подсистема "Propeller & motor dynamics"*/
    ID_OS4EquationsModel        ,   /*подсистема "12 ODEs"*/
    ID_REPORT                   ,   /*запись результатов в файл*/
    ID_MPPM                     ,   /*модуль МППМ и его интерфейс регистрации IRegMPPM*/

    //! ---------- открытые данные ----------
    ID_OS4_Physical             ,   /*физические параметры (OS4_physical)*/
    ID_OS4_Options              ,   /*настройки модели*/
    ID_OS4_State                ,   /*вектор состояния, по которому считаются производные*/
    ID_OS4_Input                ,   /*входные параметры: команда на моторы (вход control)*/
    ID_OS4_Setup                ,   /*настроечные параметры: начальные условия*/
    ID_OS4_Aero                 ,   /*сигнал Aero: [Vb, Vinf, psi, theta, rho]*/
    ID_OS4_Gravity              ,   /*сила тяжести и g в связанной СК (IMU_acc_g)*/
    ID_OS4_Fuselage             ,   /*силы и моменты корпуса*/
    ID_OS4_Propeller            ,   /*силы и моменты винтов, датчики моторов*/
    ID_OS4_Derivative           ,   /*производные вектора состояния*/
    ID_OS4_Output               ,   /*выходы модели (stateX, stateB, IMU, ...)*/
    ID_OS4_Time                 ,   /*модельное время, с*/
    ID_OS4_VehicleObj           ,   /*указатель на объект квадрокоптера*/

    ID_LAST_ID
};

#endif // GLOBALNAMEID_H
