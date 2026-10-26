#include "ICalculateElement.h"
#include <iostream>

ICalculateElement::ICalculateElement(uint32_t idClass_, double step_)
{
    idClass     = idClass_;
    step        = step_;
    bindSuccess = false;
    m_parent    = nullptr;
    m_stop      = false;
}
ICalculateElement::~ICalculateElement()
{
    for(auto ele : elements)
        delete ele;
    elements.clear();
}
void ICalculateElement::share(uint32_t id, void *ptr)
{
    TIdPtr idPtr;
    idPtr.id  = id;
    idPtr.ptr = ptr;
    shareList.push_back(idPtr);
}
uint32_t ICalculateElement::addElement(ICalculateElement *ele)
{
    ele->m_parent = this;
    elements.push_back(ele);
    return (uint32_t)elements.size();
}
ICalculateElement* ICalculateElement::root()
{
    ICalculateElement *node = this;
    while(node->m_parent != nullptr)
        node = node->m_parent;
    return node;
}
void* ICalculateElement::findSharedDown(uint32_t id)
{
    for(auto &item : shareList)
    {
        if(item.id == id)
            return item.ptr;
    }
    for(auto ele : elements)
    {
        void *ptr = ele->findSharedDown(id);
        if(ptr != nullptr)
            return ptr;
    }
    return nullptr;
}
void* ICalculateElement::findShared(uint32_t id)
{
    // поиск всегда идет от корня, поэтому данные доступны любому модулю дерева
    return root()->findSharedDown(id);
}
bool ICalculateElement::bind()
{
    bindSuccess = true;
    for(auto &req : bindList)
    {
        *req.addr = findShared(req.id);
        if(*req.addr == nullptr)
        {
            std::cerr << "bind error: module " << stringUId
                      << " can't find data id = " << req.id << std::endl;
            bindSuccess = false;
        }
    }
    for(auto ele : elements)
        bindSuccess = ele->bind() && bindSuccess;
    return bindSuccess;
}
void ICalculateElement::init()
{
    for(auto ele : elements)
        ele->init();
}
