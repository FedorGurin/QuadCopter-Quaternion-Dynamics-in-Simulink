#pragma once

#ifndef ICALCULATEELEMENT_H
#define ICALCULATEELEMENT_H

#include <stdint.h>
#include <list>
#include <string>

/*! \defgroup kernel Ядро
 *  \brief Упрощенное ядро моделирующей системы (по образцу SimTVRS)
 *
 *  Модель собирается из дерева модулей ICalculateElement.
 *  Каждый модуль:
 *   - открывает свои данные другим модулям функцией share();
 *   - заявляет, какие чужие данные ему нужны, функцией bindTo();
 *   - на этапе bind() получает указатели на эти данные;
 *   - выполняет расчет в calculate() и финальные действия в finite().
 *
 *  В отличие от SimTVRS здесь нет потоков, таймеров, сообщений
 *  и аппаратного уровня: остался только каркас, нужный для модели.
*/

//! Открытый участок памяти модуля \ingroup kernel
typedef struct
{
    //! уникальный идентификатор данных (EGlobalProtoID)
    uint32_t id;
    //! адрес данных
    void    *ptr;
}TIdPtr;

//! Запрос на связывание: куда записать адрес данных с идентификатором id \ingroup kernel
typedef struct
{
    //! идентификатор требуемых данных
    uint32_t id;
    //! адрес указателя, в который будет записан результат
    void   **addr;
}TBindRequest;

//! Базовый класс расчетного модуля \ingroup kernel
class ICalculateElement
{
public:
    /*!
     * \param idClass - уникальное имя класса модуля (EGlobalProtoID)
     * \param step_   - шаг расчета, с
    */
    ICalculateElement(uint32_t idClass, double step_ = 0.001);
    virtual ~ICalculateElement();

    //! Функция для выполнения связывания (рекурсивно для потомков)
    virtual bool bind();
    //! Функция инициализации после этапа связывания (рекурсивно для потомков)
    virtual void init();
    //! Функция основного расчета
    virtual void calculate() = 0;
    //! Финальные действия при уже вычисленных переменных
    virtual void finite() = 0;

    /*! Открыть доступ к участку памяти
     * \param id  - уникальный идентификатор участка памяти
     * \param ptr - адрес участка памяти
    */
    void share(uint32_t id, void *ptr);

    /*! Заявить зависимость от данных другого модуля
     * \param id  - идентификатор требуемых данных
     * \param ptr - указатель, в который после bind() будет записан адрес
    */
    template <typename T> void bindTo(uint32_t id, T **ptr)
    {
        *ptr = nullptr;
        TBindRequest req;
        req.id   = id;
        req.addr = reinterpret_cast<void **>(ptr);
        bindList.push_back(req);
    }

    //! поиск открытых данных во всем дереве модулей
    void* findShared(uint32_t id);

    //! добавить подчиненный модуль (модуль становится владельцем потомка)
    uint32_t addElement(ICalculateElement *ele);
    //! список подчиненных модулей
    std::list<ICalculateElement *> *returnElements(){return &elements;}
    //! родительский модуль
    ICalculateElement *parent(){return m_parent;}
    //! корневой модуль
    ICalculateElement *root();

    //! шаг расчета, с
    double timeStep(){return step;}
    //! задать шаг расчета, с
    void setTimeStep(double value){step = value;}

    //! останов модели
    void setStop() {m_stop = true;}
    //! запуск модели
    void setStart(){m_stop = false;}
    //! текущее состояние модели
    bool isStop()  {return m_stop;}

    //! уникальное имя класса элемента
    uint32_t    idClass;
    //! имя модуля (не обязательно)
    std::string stringUId;
    //! признак удачного связывания
    bool        bindSuccess;

protected:
    //! шаг расчета, с
    double step;

private:
    //! поиск в поддереве
    void* findSharedDown(uint32_t id);

    //! список подчиненных модулей
    std::list<ICalculateElement *> elements;
    //! список зависимостей данного модуля
    std::list<TBindRequest>        bindList;
    //! список открытых параметров/интерфейсов
    std::list<TIdPtr>              shareList;
    //! указатель на родителя
    ICalculateElement             *m_parent;
    //! признак останова
    bool                           m_stop;
};

#endif // ICALCULATEELEMENT_H
