#pragma once

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <stdint.h>
#include <string>

/*! \brief Класс сокета UDP (Windows/Linux, без boost)
 *
 *  Повторяет интерфейс BUdpSocket из SimTVRS в объеме, нужном МППМ:
 *  ответ отправляется на адрес последнего отправителя.
 * \ingroup kernel
*/
class UdpSocket
{
public:
    UdpSocket();
    ~UdpSocket();

    /*! Инициализация сокета
     * \param port номер порта
     * \param ip строка с ip-адресом (пустая - любой адрес)
    */
    bool init(uint16_t port, std::string ip);
    //! Связаться с сокетом (bind)
    bool bindTo();
    /*! Время ожидания данных при чтении
     * \param msec время ожидания в мсек
    */
    bool setTimeout(uint32_t msec);
    /*! Получение одного пакета
     * \return кол-во прочитанных байт, 0 - нет данных (таймаут), -1 - ошибка
    */
    int receiveFrom(uint8_t *buf, uint32_t size);
    //! Отправка на адрес последнего отправителя и указанный порт
    int sendTo(const uint8_t *buf, uint32_t size, uint16_t port);
    /*! Отправка на заданный адрес
     * \param ip строка с ip-адресом, например "127.0.0.1"
    */
    int sendTo(const uint8_t *buf, uint32_t size, uint16_t port, const std::string &ip);
    //! закрыть сокет
    void close();

private:
    //! дескриптор сокета (SOCKET в Windows)
    intptr_t sock;
    //! порт приема
    uint16_t numPort;
    //! адрес приема
    std::string ipBind;
    //! адрес последнего отправителя (sockaddr_in, хранится как байты)
    uint8_t  lastFrom[16];
    bool     hasFrom;
};

#endif // UDP_SOCKET_H
