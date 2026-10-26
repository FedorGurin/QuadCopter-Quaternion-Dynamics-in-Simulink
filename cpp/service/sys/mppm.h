#pragma once

#ifndef MPPM_H
#define MPPM_H

#include "ICalculateElement.h"
#include "IMsg.h"
#include "udpSocket.h"

#include <mutex>
#include <string>
#include <vector>

/*! \defgroup mppm Модуль MPPM
 *  \brief Доступ к данным модулей по UDP (протокол МППМ из SimTVRS)
 *
 *  Структуры пакетов скопированы из SimTVRS (service/sys/mppm.h) без
 *  изменений, поэтому существующие программы-клиенты МППМ работают
 *  с этой моделью так же, как с SimTVRS.
 *  \ingroup kernel
*/

//! Размер буфера для передачи/получения \ingroup mppm
#define SIZE_BUFFER 1024
//! Длина массива с именем программного модуля \ingroup mppm
#define SIZE_NAME 80
//! длина массива с IP адресом \ingroup mppm
#define SIZE_NAME_IP 16
//! кол-во параметров в одном пакете TSelectedRequest \ingroup mppm
#define SIZE_VALUES 64
//! Перечень идентификаторов ошибок \ingroup mppm
enum REQ_ERROR:uint8_t{
    NOT_ERROR            = 0, /*!< \brief Ошибок нет*/
    MISMATCH_SIZE_BUFFER = 1, /*!< \brief Не соответствие размеров буферов между передающими и принимающими программами */
    SMALL_BUFFER         = 2, /*!< \brief Маленький размер буферов*/
    NOT_FOUND_PM         = 3, /*!< \brief Имя программных модулей не совпадает*/
    NULL_PTR_IN_TABLE    = 4, /*!< \brief Нулевой указатель в таблице программных модулей*/
    SIZE_TYPE_IS_ZERO    = 5, /*!< \brief Нулевой размер типа в таблице программных модулей*/
    INDEX_IS_NEG         = 6, /*!< \brief Индекс имеет отрицательное значение*/
    INDEX_TOO_MACH       = 7, /*!< \brief Значение индекса превышает размер таблицы запросов*/
    OFFSET_TOO_MACH      = 8, /*!< \brief смещение слишком большое*/
    FLUSH_INDEX          = 9  /*!< \brief сбросить индексы*/
};
//! Идентификатор назначения запроса(чтение/запись/маска) \ingroup mppm
enum REQ_RWM:uint8_t{
    E_REQ_NO               = 255,/*!< \brief запрос отсутствует                          */
    E_REQ_READ             = 0,  /*!< \brief запрос на запись                            */
    E_REQ_WRITE            = 1,  /*!< \brief запрос на чтение                            */
    E_REQ_MASK             = 2,  /*!< \brief запрос маски                                */
    E_REQ_CYCLIC           = 3,  /*!< \brief запрос на добавление циклического запроса   */
    REM_CYCLIC_REQ         = 4   /*!<                                              */
};
//! Идентификатор типа запроса(модуль/выбранные параметры/список ПМ) \ingroup mppm
enum REQ_TYPE:uint8_t{
    R_T_MODULE               = 0, /*!< \brief пакет содержит все параметры структуры данных программного модуля */
    R_T_SELECTED             = 1, /*!< \brief пакет содержит выбранные параметры(в общем случае из разных программных модулей)*/
    R_T_LISTPM               = 2  /*!< \brief пакет содержит список индексов запрашиваемых программных модулей */
};

/*! \brief Тип пакета содержащий заголовок к любому пакету
 *
 * \ingroup mppm
*/
typedef struct THeadRequest_
{
    //! идентификатор пакета (для идентификации одинаковых пакетов)
    uint32_t id;
    //! идентификатор пакета (для обработки запроса)
    uint32_t uid;
    /*! Назначение сообщения:
     *              0 - чтение,
     *              1 - запись,
     *              2 - маска
     */
    uint8_t rwm;
    //! номер фрейма(в случае дробления пакетов)
    uint8_t curFrame;
    //! кол-во фреймов
    uint8_t maxFrame;
    /*! Тип сообщения
     *              0 - запрос адреса ПМ
     *              1 - запрос для работы со всей структурой
     *              2 - запрос с выбранными параметрами(из разных структур)
    */
    uint8_t type;
    //! ip адрес(отправителя)
    //uint32_t ipInt;
    uint8_t ip[SIZE_NAME_IP];//! например, 127.0.0.1
    //! порт(приемника отправителя)
    uint16_t port;
    //! уникальный индекс узла приема
    int8_t indexNode;
    //! размер пакета, байты
    uint32_t size;
}THeadRequest;
/*! \brief Тип пакета содержащий сразу список всех параметров
 * программных модулей. Параметры могут располагаться только в
 * одном участке памяти программного модуля
 *
 * \ingroup mppm
 */
typedef struct TModuleRequest_
{
    //! заголовок
    THeadRequest head;
    //! код ошибки
    uint8_t err;
    //! идентификатор запроса(элемент массива в таблице запроса)
    int16_t index;
    //! смещение внутри структуры
    uint32_t offset;
    //! размер структуры в буффере
    uint32_t sizeBuf;
    //! буффер с данными
    uint8_t buffer[SIZE_BUFFER];
}TModuleRequest;
/*! \brief Тип пакета запроса адреса(индекса) по строковому идентификатору
 *
 * \ingroup mppm
 */
typedef struct TMemRequest_
{
    //! заголовок пакета
    THeadRequest head;
    //! уникальное имя участка памяти
    uint8_t name[SIZE_NAME];
    //! возвращаемое значение
    uint32_t addr;  //!< \brief прямой адрес в памяти структуры (32раз. адресное пространство)
    uint32_t size;  //!< \brief размер структуры в байтах
    int16_t index;  //!< \brief индекс в таблице
}TMemRequest;

//! Для работы с ограниченным набором параметров \ingroup mppm
typedef struct TValueRequest_
{
    uint8_t err;                //!<\brief код ошибки
    uint8_t byte;               //!<\brief  кол-во байт значения
    int16_t indexInTable;       //!<\brief  индекс в таблице с начальным адресом
    uint32_t offset;            //!<\brief смещение
    //unsigned addr;            //!< прямой адрес в памяти
    uint8_t value[8];           //!<\brief  значение(размер занимаемой памяти зависит от byte)
}TValueRequest;

/*! \brief Тип пакета содержащий список параметров, причем параметры могут
 * располагаться в различных программных модулях
 *
 * \ingroup mppm
 */
typedef struct TSelectedRequest_
{
    //! заголовок пакета
    THeadRequest head;
    //! кол-во параметров
    uint16_t numValues;
    //! список значений
    TValueRequest values[SIZE_VALUES];
}TSelectedRequest;

//! Запись в таблице параметров модуля MPPM \ingroup mppm
typedef struct TTableMPPM_
{
    //! имя записи
    std::string name;
    //! идентификатор
    uint16_t id;
    //! адрес структуры
    uintptr_t addr;
    //! размер структуры
    uint32_t size;
    //! тип адреса структуры
    uint8_t typeAddr;
}TTableMPPM;

//! параметры настройки модуля \ingroup mppm
typedef struct TSettingMPPM_
{
    TSettingMPPM_(uint16_t port)
    {
        portReceive = port;
    }
    //! номер порта приема
    uint16_t portReceive;
}TSettingMPPM;

//! номер порта МППМ по умолчанию (как ETH_PORT_REC_MPPM в SimTVRS) \ingroup mppm
const uint16_t MPPM_DEFAULT_PORT = 9999;

/*! \brief Класс для взаимодействия с модулем MPPM
 *
 *  Работает в своем потоке (Thread без таймера): calculate() ждет
 *  один UDP-пакет (до 100 мс), обрабатывает запрос и отвечает
 *  отправителю на порт из заголовка запроса.
 *
 *  Модули регистрируют свои структуры через интерфейс IRegMPPM,
 *  открытый под идентификатором ID_MPPM:
 *  \code
 *  IRegMPPM *mppm = (IRegMPPM *)findShared(ID_MPPM);
 *  if(mppm != nullptr)
 *      mppm->addReq(ptrToReq("OS4_Setup", &setup));
 *  \endcode
 *  Как и в SimTVRS, данные читаются и пишутся прямо в память модуля
 *  без блокировок.
 * \ingroup mppm
*/
class MPPM : public ICalculateElement, public IRegMPPM
{
public:
    MPPM(TSettingMPPM set, uint32_t idClass);

    //! обработать один входящий запрос (ожидание до 100 мс)
    virtual void calculate() override;
    virtual void finite() override {}
    //! регистрация участка памяти
    virtual void addReq(const TReqMPPM &req) override;
    //! признак успешного открытия сокета
    bool isOpen(){return socketOk;}

    //! обработка запрос на чтение модуля целиком
    bool readReqModule(void);
    //! обработка запрос на запись модуля целиком
    bool writeReqModule(void);
    //! отправка данных
    bool sendData();
    //! обработка запроса на чтение только выделенных данных
    bool readReqSel();
    //! обработка запроса на запись только выделенных данных
    bool writeReqSel();
    //! обработка запроса на определение начала участка памяти
    bool readReqMem();
    //! получение команды и ее обработка
    void getControlCommand();

    //! заголовок
    THeadRequest        headReq;
    //! запрос на получение всей структуры
    TModuleRequest      moduleReq;
    //! запрос на получение списка запросов
    TSelectedRequest    selectReq;
    //! запрос на получение участка памяти
    TMemRequest         memReq;
    //! таблица с зарегистрированными структурами для обмена
    std::vector<TTableMPPM> table;

private:
    //! сокет
    UdpSocket  udpSocket;
    bool       socketOk;
    //! защита таблицы при регистрации из другого потока
    std::mutex tableMutex;
};

#endif // MPPM_H
