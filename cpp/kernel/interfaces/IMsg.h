#pragma once

#ifndef IMSG_H
#define IMSG_H

#include <stdint.h>
#include <stddef.h>
#include <string>

/*! \defgroup mppm_if Регистрация данных в МППМ
 *  \brief Как в SimTVRS: модуль описывает свои структуры запросами
 *  TReqMPPM и передает их модулю МППМ. Здесь вместо сообщения
 *  MsgRegMPPM используется интерфейс IRegMPPM, который МППМ
 *  открывает под идентификатором ID_MPPM.
 *  \ingroup kernel
*/

//! Структура с запросом к модулю MPPM \ingroup mppm_if
typedef struct TReqMPPM_
{
    std::string key;    //! уникальная строка ключ
    uintptr_t   ptr;    //! адрес участка памяти
    size_t      size;   //! размер участка памяти
    uint8_t     typePtr;//! тип указателя: 0  - статический, 1 - динамический
}TReqMPPM;

//! Преобразование указателя \param ptr в запрос типа TReqMPPM \ingroup mppm_if
template<typename T> TReqMPPM ptrToReq(std::string key, T *ptr)
{
    TReqMPPM req;
    req.key     = key;
    req.ptr     = reinterpret_cast<uintptr_t> (ptr);
    req.size    = sizeof(T);
    req.typePtr = 0;
    return req;
}

//! Интерфейс регистрации участков памяти в МППМ \ingroup mppm_if
class IRegMPPM
{
public:
    virtual ~IRegMPPM(){}
    //! добавление запроса
    virtual void addReq(const TReqMPPM &req) = 0;
};

#endif // IMSG_H
