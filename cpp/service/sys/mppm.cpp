#include "mppm.h"
#include "globalNameID.h"

#include <string.h>
#include <algorithm>
#include <iostream>

//! максимальный размер принимаемого пакета
static const size_t SIZE_RX = sizeof(TModuleRequest) > sizeof(TSelectedRequest) ?
                              sizeof(TModuleRequest) : sizeof(TSelectedRequest);

MPPM::MPPM(TSettingMPPM set, uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "MPPM";
    memset((void*)&headReq,     0, sizeof(headReq));
    memset((void*)&moduleReq,   0, sizeof(moduleReq));
    memset((void*)&selectReq,   0, sizeof(selectReq));
    memset((void*)&memReq,      0, sizeof(memReq));

    socketOk = udpSocket.init(set.portReceive, "");
    if(socketOk)
        socketOk = udpSocket.bindTo();
    if(socketOk)
        udpSocket.setTimeout(100);
    if(socketOk == false)
        std::cout << "MPPM: can't open UDP port " << set.portReceive << std::endl;

    // открываем интерфейс регистрации
    share(ID_MPPM, static_cast<IRegMPPM *>(this));
    setFreq(ICalculateElement::HzInf);
    setStart();
}
void MPPM::addReq(const TReqMPPM &req)
{
    std::lock_guard<std::mutex> lock(tableMutex);
    // поиск такого уже добавленного участка
    for(const auto &it : table)
    {
        if(it.name == req.key)
            return;
    }
    TTableMPPM tab;
    tab.name     = req.key;
    tab.id       = (uint16_t)table.size();
    tab.addr     = req.ptr;
    tab.size     = (uint32_t)req.size;
    tab.typeAddr = req.typePtr;
    table.push_back(tab);
}
void MPPM::calculate()
{
    if(socketOk == false)
    {
        setStop();
        return;
    }
    getControlCommand();
}
void MPPM::getControlCommand()
{
    static uint8_t rx[SIZE_RX];

    // получение пакета целиком (в SimTVRS сначала читается заголовок с MSG_PEEK)
    int bytes = udpSocket.receiveFrom(rx, sizeof(rx));
    if(bytes < (int)sizeof(THeadRequest))
        return;
    memcpy((void*)&headReq, rx, sizeof(THeadRequest));
    if(headReq.size != (uint32_t)bytes)
    {
        std::cout << "MPPM: packet size mismatch: head.size = " << headReq.size
                  << ", received = " << bytes << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(tableMutex);
    bool result = false;
    switch(headReq.type)
    {
        case R_T_LISTPM:
        {
            memset((void*)&memReq, 0, sizeof(memReq));
            memcpy((void*)&memReq, rx, std::min((size_t)bytes, sizeof(memReq)));
            if(headReq.rwm == E_REQ_READ)
                result = readReqMem();
            if(result == true)
                result = sendData();
            break;
        }
        case R_T_MODULE:
        {
            memset((void*)&moduleReq, 0, sizeof(moduleReq));
            memcpy((void*)&moduleReq, rx, std::min((size_t)bytes, sizeof(moduleReq)));
            if(headReq.rwm == E_REQ_READ)
                result = readReqModule();
            else if(headReq.rwm == E_REQ_WRITE)
                result = writeReqModule();
            if(result == true)
                result = sendData();
            break;
        }
        case R_T_SELECTED:
        {
            memset((void*)&selectReq, 0, sizeof(selectReq));
            memcpy((void*)&selectReq, rx, std::min((size_t)bytes, sizeof(selectReq)));
            if(headReq.rwm == E_REQ_READ)
                result = readReqSel();
            else if(headReq.rwm == E_REQ_WRITE)
                result = writeReqSel();
            if(result == true)
                result = sendData();
            break;
        }
        default:
            std::cout << "MPPM: headReq.type is incorrect" << std::endl;
    };
    if(result == false)
        std::cout << "MPPM: request was not processed, type = " << (int)headReq.type << std::endl;
}
bool MPPM::readReqMem()
{
    std::string strReq((char*)memReq.name, strnlen((char*)memReq.name, SIZE_NAME));
    for(size_t i = 0; i < table.size(); i++)
    {
        if(table[i].name == strReq)
        {
            memReq.index = (int16_t)i;
            memReq.size  = table[i].size;
            memReq.addr  = (uint32_t)table[i].addr;
            return true;
        }
    }
    return true;
}
bool MPPM::readReqModule(void)
{
    if(moduleReq.index != -1 && table.empty() == false)
    {
        unsigned i = moduleReq.index;

        if(i >= table.size())
        {
            moduleReq.err = INDEX_TOO_MACH;
        }else if(table[i].addr == 0)
        {
            moduleReq.err = NULL_PTR_IN_TABLE;
        }else if(table[i].size == 0)
        {
            moduleReq.err = SIZE_TYPE_IS_ZERO;
        }else
        {
            // структура больше буфера - передается несколькими фреймами
            int maxCount = (table[i].size / SIZE_BUFFER) + 1;
            for(int j = 0; j < maxCount; j++)
            {
                int size = table[i].size - (j * SIZE_BUFFER);
                if(size > SIZE_BUFFER)
                    size = SIZE_BUFFER;
                memcpy((void *)&moduleReq.buffer[0], (void *)(table[i].addr + j * SIZE_BUFFER), size);
                moduleReq.offset        = j * SIZE_BUFFER;
                moduleReq.sizeBuf       = size;
                moduleReq.head.size     = sizeof(TModuleRequest) - (SIZE_BUFFER - size);
                moduleReq.err           = NOT_ERROR;
                moduleReq.head.curFrame = j;
                moduleReq.head.maxFrame = maxCount - 1;
                headReq.size            = moduleReq.head.size;
                // последний фрейм еще раз отправит getControlCommand(), как в SimTVRS
                if(j < maxCount - 1)
                    sendData();
            }
        }
        return true;
    }
    moduleReq.err = NOT_FOUND_PM;
    return true;
}
bool MPPM::writeReqModule(void)
{
    moduleReq.err = NOT_FOUND_PM;

    if(moduleReq.index != -1)
    {
        unsigned i = moduleReq.index;

        if(i >= table.size())
        {
            moduleReq.err = INDEX_TOO_MACH;
        }else if(table[i].addr == 0)
        {
            moduleReq.err = NULL_PTR_IN_TABLE;
        }else if(table[i].size == 0)
        {
            moduleReq.err = SIZE_TYPE_IS_ZERO;
        }else
        {
            moduleReq.err     = NOT_ERROR;
            moduleReq.offset  = std::min(moduleReq.offset, table[i].size);
            moduleReq.sizeBuf = std::min(moduleReq.sizeBuf, table[i].size - moduleReq.offset);
            moduleReq.sizeBuf = std::min(moduleReq.sizeBuf, (uint32_t)SIZE_BUFFER);
            memcpy((void *)(table[i].addr + moduleReq.offset), (void *)(&moduleReq.buffer[0]), moduleReq.sizeBuf);
        }
    }
    return true;
}
bool MPPM::sendData()
{
    const uint8_t *ptrBuf = nullptr;
    switch(headReq.type)
    {
        case R_T_LISTPM:   ptrBuf = (const uint8_t*)&memReq;    break;
        case R_T_MODULE:   ptrBuf = (const uint8_t*)&moduleReq; break;
        case R_T_SELECTED: ptrBuf = (const uint8_t*)&selectReq; break;
    };
    if(ptrBuf == nullptr)
        return false;
    return udpSocket.sendTo(ptrBuf, headReq.size, headReq.port) >= 0;
}
bool MPPM::readReqSel()
{
    for(int i = 0; i < selectReq.numValues && i < SIZE_VALUES; i++)
    {
        TValueRequest &v = selectReq.values[i];
        int index = v.indexInTable;

        if(index < 0)
            v.err = INDEX_IS_NEG;
        else if(index >= (int)table.size())
            v.err = INDEX_TOO_MACH;
        else if(v.byte > sizeof(v.value) || v.offset + v.byte > table[index].size)
            v.err = OFFSET_TOO_MACH;
        else
            memcpy((void *)v.value, (void *)(table[index].addr + v.offset), v.byte);
    }
    return true;
}
bool MPPM::writeReqSel()
{
    for(int i = 0; i < selectReq.numValues && i < SIZE_VALUES; i++)
    {
        TValueRequest &v = selectReq.values[i];
        int index = v.indexInTable;

        if(index < 0)
            v.err = INDEX_IS_NEG;
        else if(index >= (int)table.size())
            v.err = INDEX_TOO_MACH;
        else if(v.byte > sizeof(v.value) || v.offset + v.byte > table[index].size)
            v.err = OFFSET_TOO_MACH;
        else
            memcpy((void *)(table[index].addr + v.offset), (void *)v.value, v.byte);
    }
    return true;
}
